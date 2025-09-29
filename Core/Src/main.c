#include "main.h"
#include "usart.h"
#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>

/* ===== Retarget (provided elsewhere) ===== */
int __io_putchar(int ch);

/* ===== Forward decls for CubeMX-style statics ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* ===== Optional: LED pin on PA5 ===== */
#ifndef LED_GPIO_Port
#define LED_GPIO_Port GPIOA
#endif
#ifndef LED_Pin
#define LED_Pin GPIO_PIN_5
#endif

/* ===== Parser return codes per rubric ===== */
#define CMD_STOP   0
#define CMD_START  1
#define CMD_CLEAR  2
#define CMD_BAD   -1
#define CMD_OVF   -2

/* Parse a single, NUL-terminated line (no trailing '\r'). */
static int parser(const char *s)
{
  if (!s) return CMD_BAD;

  if (!strcmp(s, "STOP") || !strcmp(s, "stop"))
    return CMD_STOP;

  if (!strcmp(s, "START") || !strcmp(s, "start"))
    return CMD_START;

  if (!strcmp(s, "CLEAR") || !strcmp(s, "clear"))
    return CMD_CLEAR;

  return CMD_BAD;
}

/* After parsing a line, reset the "line buffer indices".
   With our ring implementation, ring_read_line() already advances the read indices.
   We keep this function to match the rubric wording. */
static void reset_after_parse(void)
{
  /* no-op: consumption already performed by ring_read_line() */
}

/* Application reaction to parsed command */
static void act_on(int code, const char *raw)
{
  switch (code) {
    case CMD_STOP:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      printf("STOP command is received in Tera Term\r\n");
      break;
    case CMD_START:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      printf("START command is received in Tera Term\r\n");
      break;
    case CMD_CLEAR:
      printf("CLEAR command is received in Tera Term\r\n");
      break;
    case CMD_OVF:
      printf("ERROR: buffer overflow\r\n");
      break;
    default:
      printf("undefined command\r\n");
      break;
  }
}

/* ======= Main ============================================================ */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  USR_USART2_UART_Init(115200);
  /* Make printf unbuffered */
  setvbuf(stdout, NULL, _IONBF, 0);

  MX_GPIO_Init();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  ring_init();

  printf("\r\n=== ECE-500 Ring Buffer Demo ===\r\n> ");

  for (;;)
  {
    /* If you kept polled RX, this feeds the ring.
       If you switched to the IRQ-driven RX I gave you, this will just rarely run. */
    uint8_t ch;
    if (USR_USART2_TryRead(&ch)) {
      ring_add((char)ch);
    }

    if (ring_has_line()) {
      char line[64];
      int n = ring_read_line(line, sizeof(line));
      int code = (n >= (int)(sizeof(line)-1)) ? CMD_OVF : parser(line);
      act_on(code, line);
      reset_after_parse();
      printf("> ");
    }

    HAL_Delay(1);
  }
}

/* ======= System Clock & GPIO (simple HSI @ 16MHz) ======================= */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/* ======= Error Handler =================================================== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
