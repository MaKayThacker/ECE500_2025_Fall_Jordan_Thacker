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

#define MOTOR_PWM_PRESCALER_VALUE   (16u - 1u)   /* 16 MHz / 16 -> 1 MHz timer clock */
#define MOTOR_PWM_PERIOD_TICKS      (1000u - 1u) /* 1 MHz / 1000 -> 1 kHz PWM */
#define ENCODER_PULSES_PER_REV      20u          /* hall encoder edges per mechanical rev */
#define RPM_UPDATE_PERIOD_MS        1000u        /* default 1 Hz reporting */
#define RPM_SCALE_PER_MINUTE        60000u       /* ms per minute for rpm conversion */
#define RPS_X100_SCALE              100000u      /* (100 * 1000) for rps w/2 decimal places */

static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static uint32_t compute_scaled_speed(uint32_t delta,
                                     uint32_t elapsed_ms,
                                     uint32_t pulses_per_rev,
                                     uint32_t scale);

static void service_logging(void);
static void mark_next_sample_immediate(void);

/* Weak default so the linker is happy even if Cube's MSP file is missing.
 * If stm32g0xx_hal_msp.c defines a strong HAL_TIM_MspPostInit(), it will
 * automatically override this one. */
__attribute__((weak)) void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    (void)htim;
}

static void to_upper(char *s) {
    if (!s) return;
    for (; *s; ++s) {
        if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 'a' + 'A');
    }
}

static void trim(char *s) {
    if (!s) return;
    char *p = s;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n')) {
        s[--n] = '\0';
    }
}

static void convert_tmp102_raw(int16_t raw, int* c_tenths, int* f_tenths)
{
    int c = (int)((int32_t)raw * 625 / 1000);
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

static void print_prompt(void) {
    printf("> ");
}

static void print_help(void) {
    printf("Commands: START, STOP, TEMP, CLEAN, STATUS, DUTYxx, HALT, HELP\r\n");
    printf("Use DUTY<0-100> to set the PWM duty cycle and enable RPM reporting.\r\n");
    printf("Use HALT to stop the motor drive and the RPM output.\r\n");
}

static void print_session_summary(uint16_t first, uint16_t last)
{
    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    uint32_t span = (uint32_t)last - (uint32_t)first;
    uint32_t count = span / 2u + 1u;
    printf("Last session: %lu samples (first=0x%04X last=0x%04X)\r\n",
           (unsigned long)count, first, last);
}

/* This version replaces the broken one that had the @@ diff garbage. */
static void dump_recent_session(void)
{
    uint16_t first = 0, last = 0;
    (void)fram_get_first_last(&first, &last);

    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    int16_t raw = 0;
    if (tmp102_read_raw(&raw) != HAL_OK)
    {
        printf("[LOG] TMP102 read failed\r\n");
        print_prompt();
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
    print_prompt();
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
        /* Stop logging to avoid spamming. */
        fram_cmd_stop();
        rpm_reporting_enabled = 0u;
        return;
    }

    printf("[LOG] %d.%01d C (%d.%01d F)\r\n",
           c_tenths / 10,
           abs(c_tenths % 10),
           f_tenths / 10,
           abs(f_tenths % 10));

    print_prompt();

    next_log_ms = now + RPM_UPDATE_PERIOD_MS;
}

static void motor_apply_pwm_percent(uint32_t percent)
{
    if (percent > 100u)
    {
        percent = 100u;
    }

    current_pwm_percent = (uint8_t)percent;

    uint32_t ticks = ((uint32_t)(MOTOR_PWM_PERIOD_TICKS + 1u) * percent + 50u) / 100u;
    if (ticks > (MOTOR_PWM_PERIOD_TICKS + 1u))
    {
        ticks = MOTOR_PWM_PERIOD_TICKS + 1u;
    }
    if (ticks > MOTOR_PWM_PERIOD_TICKS)
    {
        ticks = MOTOR_PWM_PERIOD_TICKS;
    }

    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ticks);
}

static void motor_enable_reporting(void)
{
    rpm_reporting_enabled = 1u;
    last_rpm_tick_ms = HAL_GetTick();
    last_pulse_snapshot = encoder_pulse_count;
}

static void motor_disable_reporting(void)
{
    rpm_reporting_enabled = 0u;
}

static void service_motor_rpm(void)
{
    if (!rpm_reporting_enabled)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    int32_t elapsed_since_last = (int32_t)(now - last_rpm_tick_ms);
    if (elapsed_since_last < (int32_t)RPM_UPDATE_PERIOD_MS)
    {
        return;
    }

    uint32_t elapsed_ms = (uint32_t)elapsed_since_last;
    last_rpm_tick_ms = now;
    uint32_t pulses = encoder_pulse_count;
    uint32_t delta = pulses - last_pulse_snapshot;
    last_pulse_snapshot = pulses;

    uint32_t rpm = compute_scaled_speed(delta,
                                        elapsed_ms,
                                        ENCODER_PULSES_PER_REV,
                                        RPM_SCALE_PER_MINUTE);

    uint32_t rps_x100 = compute_scaled_speed(delta,
                                             elapsed_ms,
                                             ENCODER_PULSES_PER_REV,
                                             RPS_X100_SCALE);

    /* DEBUG: include raw counts and timing to see if EXTI is firing. */
    printf("[RPM] pulses=%lu irq=%lu delta=%lu elapsed=%lu ms -> %lu.%02lu rps (%lu rpm) duty=%u%%\r\n",
           (unsigned long)pulses,
           (unsigned long)encoder_irq_count,
           (unsigned long)delta,
           (unsigned long)elapsed_ms,
           (unsigned long)(rps_x100 / 100u),
           (unsigned long)(rps_x100 % 100u),
           (unsigned long)rpm,
           current_pwm_percent);
    print_prompt();
}

static void handle_line(char *line) {
    trim(line);
    to_upper(line);

    if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
        print_help();

    } else if (strcmp(line, "STATUS") == 0) {
        uint16_t first = 0xFFFF, last = 0xFFFF;
        (void)fram_get_first_last(&first, &last);
        printf("Logging: %s\r\n", fram_logging_enabled() ? "ON" : "OFF");
        print_session_summary(first, last);

    } else if (strcmp(line, "START") == 0) {
        fram_cmd_start();
        mark_next_sample_immediate();
        printf("Logging STARTED\r\n");

    } else if (strcmp(line, "STOP") == 0) {
        fram_cmd_stop();
        printf("Logging STOPPED\r\n");

    } else if (strcmp(line, "CLEAN") == 0 || strcmp(line, "CLEAR") == 0) {
        fram_cmd_clear();
        mark_next_sample_immediate();
        printf("FRAM CLEARED\r\n");

    } else if (strcmp(line, "TEMP") == 0) {
        dump_recent_session();

    } else if (strncmp(line, "DUTY", 4) == 0) {
        char *arg = line + 4;
        trim(arg);
        if (*arg == '\0') {
            printf("Usage: DUTY<0-100>\r\n");
        } else {
            char *endptr = NULL;
            long duty = strtol(arg, &endptr, 10);
            if ((endptr && *endptr != '\0') || duty < 0 || duty > 100) {
                printf("DUTY expects a value between 0 and 100%%\r\n");
            } else {
                motor_apply_pwm_percent((uint32_t)duty);
                motor_enable_reporting();
                printf("PWM duty set to %ld%%\r\n", duty);
                printf("RPM reporting enabled\r\n");
            }
        }

    } else if (strcmp(line, "HALT") == 0) {
        motor_apply_pwm_percent(0u);
        motor_disable_reporting();
        printf("Motor HALT: PWM disabled and RPM reporting stopped\r\n");

    } else if (line[0] != '\0') {
        printf("Unknown: %s\r\n", line);
        print_help();
    }
}

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
    motor_apply_pwm_percent(0u);

    (void)tmp102_initialize();
    fram_init_state();

    printf("\r\nUART online (115200 8N1)\r\n");
    print_help();

    /* DEBUG: dump basic TIM14 PWM configuration so we know PWM is sane. */
    printf("DEBUG TIM14: PSC=%lu ARR=%lu CCR1=%lu\r\n",
           (unsigned long)htim14.Instance->PSC,
           (unsigned long)htim14.Instance->ARR,
           (unsigned long)htim14.Instance->CCR1);

    print_prompt();

    char line[128];

    while (1) {
        if (ring_has_line()) {
            int n = ring_read_line(line, sizeof line);
            if (n > 0) {
                handle_line(line);
                print_prompt();
            }
        }

        service_logging();
        service_motor_rpm();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO ports A/B/C: A for encoder, B/C for FRAM */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* FRAM chip select and write-protect pins */
    GPIO_InitStruct.Pin = FRAM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FRAM_WP_Pin;
    HAL_GPIO_Init(FRAM_WP_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_WP_GPIO_Port, FRAM_WP_Pin, GPIO_PIN_SET);

    /* Motor encoder input on PA0, EXTI line 0 */
    GPIO_InitStruct.Pin = MOTOR_ENCODER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_ENCODER_GPIO_Port, &GPIO_InitStruct);

    /* NVIC for EXTI0_1 (handles PA0/PA1) */
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM14_Init(void)
{
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = MOTOR_PWM_PRESCALER_VALUE;   /* 1 MHz */
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = MOTOR_PWM_PERIOD_TICKS;         /* 1 kHz PWM */
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0u;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim14);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_ENCODER_Pin)
    {
        encoder_pulse_count++;
        encoder_irq_count++;
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
