#include "main.h"
#include "usart.h"
#include "ring_retarget.h"
#include "fram_spi.h"
#include "i2c1_tmp102.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim14;

/* Private function prototypes -----------------------------------------------*/
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_GPIO_Init(void);

/* =========================
 * Motor + RPM definitions
 * ========================= */
#define MOTOR_PWM_PRESCALER_VALUE   (16u - 1u)   /* 16 MHz / 16 -> 1 MHz timer clock */
#define MOTOR_PWM_PERIOD_TICKS      (1000u - 1u) /* 1 MHz / 1000 -> 1 kHz PWM */

#define ENCODER_PULSES_PER_REV      20u          /* hall encoder edges per mechanical rev */
#define RPM_UPDATE_PERIOD_MS        1000u        /* default 1 Hz reporting */
#define RPM_SCALE_RPM               60000u       /* ms -> minutes (1000 ms * 60) */

/* =========================
 * TMP102 / logging helpers
 * ========================= */

/* Input: raw is the 12-bit signed TMP102 temperature in 0.0625 C units, right justified
 * Output: c_tenths -> temperature in 0.1 C units
 *         f_tenths -> temperature in 0.1 F units
 */
static void convert_tmp102_raw(int16_t raw, int* c_tenths, int* f_tenths)
{
    /* 1 LSB = 0.0625 C = 0.625 tenths of C.
       So 0.0625 * 10 = 0.625 -> temp_c_tenths = raw * 625 / 1000. */
    int c = (int)((raw * 625) / 100);
    /* Multiply C by 1.8 (9/5) and add 32, staying in tenths:
       F = C * 1.8 + 32 => tenths_F = tenths_C * 9/5 + 320. */
    int f = (c * 9) / 5 + 320;

    if (c_tenths) *c_tenths = c;
    if (f_tenths) *f_tenths = f;
}

/* ----------------- Runtime state ----------------- */
static uint32_t next_log_ms = 0u;
static volatile uint32_t encoder_pulse_count = 0u;
/* DEBUG: total number of encoder EXTI callbacks seen */
static volatile uint32_t encoder_irq_count   = 0u;

static uint8_t rpm_reporting_enabled = 0u;
static uint32_t last_rpm_tick_ms = 0u;
static uint32_t last_pulse_snapshot = 0u;
static uint8_t current_pwm_percent = 0u;

static uint32_t compute_scaled_speed(uint32_t delta,
                                     uint32_t elapsed_ms,
                                     uint32_t pulses_per_rev,
                                     uint32_t scale)
{
    if ((pulses_per_rev == 0u) || (elapsed_ms == 0u) || (scale == 0u))
    {
        return 0u;
    }

    uint64_t numerator = (uint64_t)delta * (uint64_t)scale;
    uint64_t denominator = (uint64_t)pulses_per_rev * (uint64_t)elapsed_ms;
    if (denominator == 0u)
    {
        return 0u;
    }

    return (uint32_t)(numerator / denominator);
}

/* =========================
 * Motor PWM helpers
 * ========================= */

static void motor_apply_pwm_percent(uint8_t percent)
{
    if (percent > 100u) percent = 100u;
    current_pwm_percent = percent;

    /* Compute CCR value based on PWM period. */
    uint32_t pulse = ((uint32_t)(MOTOR_PWM_PERIOD_TICKS + 1u) * percent) / 100u;
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pulse);
}

/* =========================
 * CLI / helper printing
 * ========================= */

static void print_banner(void)
{
    printf("\r\n");
    printf("=======================================\r\n");
    printf(" ECE 500: Motor, TMP102, and FRAM Demo \r\n");
    printf("=======================================\r\n");
}

static void print_prompt(void)
{
    printf("> ");
}

static void print_help(void)
{
    printf("Commands:\r\n");
    printf("  HELP                - Show this help\r\n");
    printf("  PWM <0-100>         - Set motor PWM percent\r\n");
    printf("  RPM                 - Enable RPM reporting (1 Hz)\r\n");
    printf("  RPM OFF             - Disable RPM reporting\r\n");
    printf("  TEMP                - Single TMP102 temperature read\r\n");
    printf("  TEMPDBG             - TMP102 debug dump\r\n");
    printf("  FRAM START          - Begin TMP102 logging to FRAM\r\n");
    printf("  FRAM STOP           - Stop TMP102 logging\r\n");
    printf("  FRAM CLEAR          - Clear FRAM and log metadata\r\n");
    printf("  FRAM STATUS         - Print FRAM logging status\r\n");
    printf("  FRAM DUMP           - Dump the most recent session\r\n");
    printf("  ENCODERDBG          - Print encoder debug count\r\n");
    printf("  HALT                - Stop motor and RPM reporting\r\n");
}

static void print_fram_status(void)
{
    uint16_t first = 0, last = 0;
    (void)fram_get_first_last(&first, &last);

    printf("FRAM logging: %s\r\n", fram_logging_enabled() ? "ENABLED" : "DISABLED");
    if (last < first)
    {
        printf("  No valid samples stored.\r\n");
    }
    else
    {
        uint32_t sample_count = (((uint32_t)last - (uint32_t)first) / 2u) + 1u;
        printf("  first=0x%04X, last=0x%04X, count=%lu\r\n",
               first, last, (unsigned long)sample_count);
    }
}

/* Dump data from the MOST RECENT FRAM session to the terminal. */
static void dump_recent_session(void)
{
    uint16_t first = 0, last = 0;
    (void)fram_get_first_last(&first, &last);

    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    printf("Dumping FRAM samples from 0x%04X to 0x%04X:\r\n", first, last);

    uint16_t addr = first;
    uint8_t buf[2];

    while (1)
    {
        if (fram_read(addr, buf, 2) != HAL_OK)
        {
            printf("FRAM read error at 0x%04X\r\n", addr);
            break;
        }

        uint16_t raw16 = (uint16_t)((buf[0] << 8) | buf[1]);
        int16_t raw = (int16_t)raw16;

        int c_tenths = 0;
        int f_tenths = 0;
        convert_tmp102_raw(raw, &c_tenths, &f_tenths);

        printf("  0x%04X: raw=0x%04X => %d.%01d C (%d.%01d F)\r\n",
               addr,
               raw16,
               c_tenths / 10,
               abs(c_tenths % 10),
               f_tenths / 10,
               abs(f_tenths % 10));

        if (addr == last)
        {
            break;
        }

        addr = (uint16_t)(addr + 2u);
    }
}

/* Force the very next logging tick to take a sample (used after START/CLEAR). */
static void mark_next_sample_immediate(void)
{
    next_log_ms = 0u;
}

/* Periodic TMP102 -> FRAM logging. Call once per main loop. */
static void service_logging(void)
{
    if (!fram_logging_enabled())
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* If next_log_ms == 0, we want an immediate sample.
       Otherwise, wait until we reach the next deadline. */
    if ((next_log_ms != 0u) &&
        ((int32_t)(now - next_log_ms) < 0))
    {
        return;
    }

    int16_t raw = 0;
    if (tmp102_read_raw(&raw) != HAL_OK)
    {
        printf("[LOG] TMP102 read failed\r\n");
        print_prompt();
        /* Try again after one period. */
        next_log_ms = now + RPM_UPDATE_PERIOD_MS;
        return;
    }

    int c_tenths = 0;
    int f_tenths = 0;
    convert_tmp102_raw(raw, &c_tenths, &f_tenths);

    if (fram_log_sample((uint16_t)raw) != HAL_OK)
    {
        printf("[LOG] FRAM full or write error, logging stopped\r\n");
        print_prompt();
        return;
    }

    printf("[LOG] %d.%01d C (%d.%01d F)\r\n",
           c_tenths / 10,
           abs(c_tenths % 10),
           f_tenths / 10,
           abs(f_tenths % 10));

    /* Schedule next log */
    next_log_ms = now + RPM_UPDATE_PERIOD_MS;
}

/* =========================
 * Motor speed / encoder
 * ========================= */

/* Called from EXTI callback. */
static void encoder_pulse_seen(void)
{
    encoder_pulse_count++;
    encoder_irq_count++;
}

/* Called periodically from main loop to compute and print RPM, if enabled. */
static void service_rpm_reporting(void)
{
    if (!rpm_reporting_enabled)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if (last_rpm_tick_ms == 0u)
    {
        last_rpm_tick_ms = now;
        last_pulse_snapshot = encoder_pulse_count;
        return;
    }

    uint32_t elapsed = now - last_rpm_tick_ms;
    if (elapsed < RPM_UPDATE_PERIOD_MS)
    {
        return;
    }

    uint32_t pulses = encoder_pulse_count - last_pulse_snapshot;
    last_pulse_snapshot = encoder_pulse_count;
    last_rpm_tick_ms = now;

    /* RPM = pulses * (60000 / (pulses_per_rev * elapsed_ms)) */
    uint32_t rpm = compute_scaled_speed(pulses,
                                        elapsed,
                                        ENCODER_PULSES_PER_REV,
                                        RPM_SCALE_RPM);

    printf("[RPM] %lu rpm (pulses=%lu, elapsed=%lu ms, PWM=%u%%)\r\n",
           (unsigned long)rpm,
           (unsigned long)pulses,
           (unsigned long)elapsed,
           (unsigned)current_pwm_percent);
}

/* =========================
 * CLI command handling
 * ========================= */

static void handle_command(const char* line)
{
    if (line == NULL)
    {
        return;
    }

    if (strcmp(line, "HELP") == 0)
    {
        print_help();
        return;
    }

    if (strncmp(line, "PWM ", 4) == 0)
    {
        int val = atoi(&line[4]);
        if (val < 0) val = 0;
        if (val > 100) val = 100;
        motor_apply_pwm_percent((uint8_t)val);
        printf("PWM set to %d%%\r\n", val);
        return;
    }

    if (strcmp(line, "RPM") == 0)
    {
        rpm_reporting_enabled = 1u;
        last_rpm_tick_ms = 0u;
        last_pulse_snapshot = encoder_pulse_count;
        printf("RPM reporting enabled.\r\n");
        return;
    }

    if (strcmp(line, "RPM OFF") == 0)
    {
        rpm_reporting_enabled = 0u;
        printf("RPM reporting disabled.\r\n");
        return;
    }

    if (strcmp(line, "TEMP") == 0)
    {
        int16_t raw = 0;
        if (tmp102_read_raw(&raw) != HAL_OK)
        {
            printf("TMP102 read error.\r\n");
            return;
        }

        int c_tenths = 0;
        int f_tenths = 0;
        convert_tmp102_raw(raw, &c_tenths, &f_tenths);

        printf("TMP102: raw=0x%04X => %d.%01d C, %d.%01d F\r\n",
               (uint16_t)raw,
               c_tenths / 10,
               abs(c_tenths % 10),
               f_tenths / 10,
               abs(f_tenths % 10));
        return;
    }

    if (strcmp(line, "TEMPDBG") == 0)
    {
        TMP102_DebugDump();
        return;
    }

    if (strcmp(line, "FRAM START") == 0)
    {
        fram_cmd_start();
        mark_next_sample_immediate();
        printf("FRAM logging started.\r\n");
        return;
    }

    if (strcmp(line, "FRAM STOP") == 0)
    {
        fram_cmd_stop();
        printf("FRAM logging stopped.\r\n");
        return;
    }

    if (strcmp(line, "FRAM CLEAR") == 0)
    {
        fram_cmd_clear();
        mark_next_sample_immediate();
        printf("FRAM cleared and logging reset.\r\n");
        return;
    }

    if (strcmp(line, "FRAM STATUS") == 0)
    {
        print_fram_status();
        return;
    }

    if (strcmp(line, "FRAM DUMP") == 0)
    {
        dump_recent_session();
        return;
    }

    if (strcmp(line, "ENCODERDBG") == 0)
    {
        printf("Encoder IRQ count: %lu\r\n", (unsigned long)encoder_irq_count);
        return;
    }

    if (strcmp(line, "HALT") == 0)
    {
        motor_apply_pwm_percent(0u);
        rpm_reporting_enabled = 0u;
        printf("Motor halted and RPM reporting disabled.\r\n");
        return;
    }

    if (line[0] != '\0')
    {
        printf("Unknown command: '%s'\r\n", line);
        print_help();
    }
}

/* =========================
 * Main
 * ========================= */

int main(void)
{
    HAL_Init();

    /* Configure I2C1 on PB8/PB9 at 100 kHz using HSI16 as kernel clock */
    I2C1_Init_PB8PB9_100k_HSI16();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM14_Init();
    MX_USART2_UART_Init();

    if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initialize FRAM state (loads first/last, etc.) */
    fram_init_state();

    print_banner();
    print_help();
    print_prompt();

    uint32_t last_blink_ms = HAL_GetTick();

    while (1)
    {
        /* Blink LED as heartbeat every 500 ms. */
        uint32_t now = HAL_GetTick();
        if ((now - last_blink_ms) >= 500u)
        {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
            last_blink_ms = now;
        }

        /* Handle RPM reporting and logging tasks. */
        service_rpm_reporting();
        service_logging();

        /* Process UART input (line-based) */
        if (ring_has_line())
        {
            char line[64] = {0};
            (void)ring_read_line(line, sizeof(line));
            /* strip trailing CR/LF */
            size_t len = strlen(line);
            while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
            {
                line[--len] = '\0';
            }
            handle_command(line);
            print_prompt();
        }
    }
}

/* =========================
 * Peripheral init
 * ========================= */

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM14_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = MOTOR_PWM_PRESCALER_VALUE;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = MOTOR_PWM_PERIOD_TICKS;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure LED pin */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

    /* FRAM CS / WP pins */
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_WP_GPIO_Port, FRAM_WP_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = FRAM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FRAM_WP_Pin;
    HAL_GPIO_Init(FRAM_WP_GPIO_Port, &GPIO_InitStruct);

    /* Motor encoder input on PA0, EXTI line 0 */
    GPIO_InitStruct.Pin = MOTOR_ENCODER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MOTOR_ENCODER_GPIO_Port, &GPIO_InitStruct);

    /* NVIC for EXTI0_1 (handles PA0/PA1) */
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* =========================
 * Callbacks / error handler
 * ========================= */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_ENCODER_Pin)
    {
        encoder_pulse_seen();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
