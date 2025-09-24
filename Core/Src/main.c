#include "parser.h"
#include "ringbuffer.h"
#include "usart.h"
#include "retarget.h"
#include <stdio.h>
#include <stdbool.h>

// Terminal/control characters
#define BACKSPACE   '\b'
#define DEL         0x7F
#define CR          '\r'
#define LF          '\n'

extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Error_Handler(void);

int main(void) {
    /* --- HAL + Clocks + Peripherals ------------------------------------- */
    HAL_Init();                    // Reset HAL state & SysTick
    SystemClock_Config();          // Set core clocks (HSI, dividers)
    USR_USART2_UART_Init(115200);  // Init USART2 for 115200-8N1
    setvbuf(stdout, NULL, _IONBF, 0); // Unbuffered stdout (printf -> UART)
    MX_GPIO_Init();                // LED pin (PA5) as output

    /* --- App init -------------------------------------------------------- */
    ring_init();                   // Reset ring buffer indices/state
    printf("\r\n=== ECE-500 Ring Buffer Demo ===\r\n> ");

    /* --- Main loop ------------------------------------------------------- */
    while (1) {
        uint8_t input_char;
        bool buffer_overflow = false;

        /* 1) Non-blocking read from UART (ISR puts 1 byte into a staging var)
              If a byte arrived, echo it and push to ring buffer. */
        if (USR_USART2_TryRead(&input_char)) {

            /* 1a) Handle local line editing for backspace/delete. */
            if (input_char == BACKSPACE || input_char == DEL) {
                ring_backspace();                 // remove last non-CR/LF from ring
                __io_putchar(BACKSPACE);          // move cursor left
                __io_putchar(' ');                // erase char on screen
                __io_putchar(BACKSPACE);          // move cursor left again
            } else {
                __io_putchar((int)input_char);    // realtime terminal echo
                ring_add((char)input_char, &buffer_overflow); // store; flag overflow
            }

            /* 2) If one (or more) full line(s) are present, parse them all. */
            while (ring_has_line()) {
                char parsed_line[32];
                int line_length = ring_pop_line(parsed_line, sizeof(parsed_line));
                if (line_length < 0) break;  // defensive (shouldn’t happen)

                /* 2a) Parse command (or overflow) and build a reply. */
                int command_code = buffer_overflow ? CMD_OVERFLOW : parser(parsed_line);
                char response[64];
                parser_reply(command_code, response, sizeof(response));
                printf("\r\n%s\r\n> ", response);

                /* 2b) CLEAR command also wipes ring contents immediately. */
                if (command_code == CMD_CLEAR) ring_clear();

                /* Only report overflow once per received burst */
                buffer_overflow = false;
            }

            /* 3) Normalize indices if empty; keeps continuous parsing stable. */
            ring_reset_after_parse();
        }

        /* 4) Small heartbeat blink so we know the loop is alive. */
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(200);
    }
}

/* --- Clocks / GPIO / Error ---------------------------------------------- */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
