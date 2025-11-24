/*
 * helpers_GPIO.c
 *
 *  GPIO / motor / logging / UI helpers moved from main.c
 */

#include "main.h"
#include "fram_spi.h"
#include "i2c1_tmp102.h"
#include "ring_retarget.h"
#include "helper_GPIO.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* ----- Motor / encoder config (copied from main.c so both units see same values) ----- */
#define MOTOR_PWM_PRESCALER_VALUE   (16u - 1u)   /* 16 MHz / 16 -> 1 MHz timer clock */
#define MOTOR_PWM_PERIOD_TICKS      (1000u - 1u) /* 1 MHz / 1000 -> 1 kHz PWM */
#define ENCODER_PULSES_PER_REV      20u          /* pulses per mechanical revolution */
#define RPM_UPDATE_PERIOD_MS        1000u        /* report every 1 second */
#define RPM_SCALE_PER_MINUTE        60000u       /* ms/min for rpm conversion */
#define RPS_X100_SCALE              100000u      /* (100 * 1000) for rps w/2 decimal digits */

/* TIM14 handle lives in main.c; we just reference it here */
extern TIM_HandleTypeDef htim14;

/* ----------------- Runtime state (moved as-is from main.c) ----------------- */
static uint32_t next_log_ms = 0u;

static volatile uint32_t encoder_pulse_count = 0u;
static volatile uint32_t encoder_irq_count   = 0u;

static uint8_t  rpm_reporting_enabled = 0u;
static uint32_t last_rpm_tick_ms      = 0u;
static uint32_t last_pulse_snapshot   = 0u;
static uint8_t  current_pwm_percent   = 0u;

/* Local helper (was static in main.c) */
static void encoder_exti_force_config(void);

/* ---------------- String helpers (moved from main.c) ---------------- */
void to_upper(char *s)
{
    if (!s) return;
    for (; *s; ++s)
    {
        if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 'a' + 'A');
    }
}

void trim(char *s)
{
    if (!s) return;
    char *p = s;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (p != s) memmove(s, p, strlen(s) + 1);
    size_t n = strlen(s);
    while (n > 0 &&
          (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n'))
    {
        s[--n] = '\0';
    }
}

/* ---------------- TMP102 conversion helper (stays app-level) ---------------- */
void convert_tmp102_raw(int16_t raw, int *c_tenths, int *f_tenths)
{
    int c = (int)((int32_t)raw * 625 / 1000);  /* 0.0625°C per LSB */
    int f = (c * 9) / 5 + 320;                 /* tenths of °F */
    if (c_tenths) *c_tenths = c;
    if (f_tenths) *f_tenths = f;
}

/* ------------------------------------------------------------------------ */
uint32_t compute_scaled_speed(uint32_t delta,
                              uint32_t elapsed_ms,
                              uint32_t pulses_per_rev,
                              uint32_t scale)
{
    if ((pulses_per_rev == 0u) || (elapsed_ms == 0u) || (scale == 0u))
        return 0u;

    uint64_t numerator   = (uint64_t)delta * (uint64_t)scale;
    uint64_t denominator = (uint64_t)pulses_per_rev * (uint64_t)elapsed_ms;

    if (denominator == 0u)
        return 0u;

    return (uint32_t)(numerator / denominator);
}

/* UI helpers --------------------------------------------------------------*/
void print_prompt(void)
{
    printf("> ");
}

void print_help(void)
{
    printf("Commands: START, STOP, TEMP, FRAM, CLEAN, STATUS, DUTYxx, HALT, HELP\r\n");
    printf("Use START/STOP to control TMP102 -> FRAM logging.\r\n");
    printf("Use TEMP to force a single new sample.\r\n");
    printf("Use FRAM to dump all logged samples since last CLEAN.\r\n");
    printf("Use DUTY<0-100> to set the PWM duty cycle and enable RPM reporting.\r\n");
    printf("Use HALT to stop the motor drive and the RPM output.\r\n");
}

void print_session_summary(uint16_t first, uint16_t last)
{
    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    uint32_t span  = (uint32_t)last - (uint32_t)first;
    uint32_t count = span / 2u + 1u;
    printf("Last session: %lu samples (first=0x%04X last=0x%04X)\r\n",
           (unsigned long)count, first, last);
}

/* Simple TEMP log function using TMP102 + FRAM --------------------------- */
void dump_recent_session(void)
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
           c_tenths / 10,  abs(c_tenths % 10),
           f_tenths / 10, abs(f_tenths % 10));
    print_prompt();
}

void mark_next_sample_immediate(void)
{
    next_log_ms = 0u;
}

/* Periodic TMP102 -> FRAM logging. Call once per main loop. */
void service_logging(void)
{
    if (!fram_logging_enabled())
        return;

    uint32_t now = HAL_GetTick();

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
        fram_cmd_stop();
        rpm_reporting_enabled = 0u;
        return;
    }

    printf("[LOG] %d.%01d C (%d.%01d F)\r\n",
           c_tenths / 10,  abs(c_tenths % 10),
           f_tenths / 10, abs(f_tenths % 10));
    print_prompt();

    next_log_ms = now + RPM_UPDATE_PERIOD_MS;
}

/* ------------- New: FRAM dump callback + command ------------------------ */

/* Callback used by fram_iterate_samples() to print each stored sample */
static void fram_dump_print_cb(uint16_t index, uint16_t raw)
{
    int c_tenths = 0;
    int f_tenths = 0;
    convert_tmp102_raw((int16_t)raw, &c_tenths, &f_tenths);

    printf("FRAM[%lu]: raw=0x%04X -> %d.%01d C (%d.%01d F)\r\n",
           (unsigned long)index,
           (unsigned int)raw,
           c_tenths / 10,  abs(c_tenths % 10),
           f_tenths / 10, abs(f_tenths % 10));
}

/* Motor control / RPM -----------------------------------------------------*/
void motor_apply_pwm_percent(uint32_t percent)
{
    if (percent > 100u)
        percent = 100u;

    current_pwm_percent = (uint8_t)percent;

    uint32_t ticks = ((uint32_t)(MOTOR_PWM_PERIOD_TICKS + 1u) * percent + 50u) / 100u;
    if (ticks > (MOTOR_PWM_PERIOD_TICKS + 1u))
        ticks = MOTOR_PWM_PERIOD_TICKS + 1u;
    if (ticks > MOTOR_PWM_PERIOD_TICKS)
        ticks = MOTOR_PWM_PERIOD_TICKS;

    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ticks);
}

void motor_enable_reporting(void)
{
    rpm_reporting_enabled = 1u;
    last_rpm_tick_ms      = HAL_GetTick();
    last_pulse_snapshot   = encoder_pulse_count;
}

void motor_disable_reporting(void)
{
    rpm_reporting_enabled = 0u;
}

void service_motor_rpm(void)
{
    if (!rpm_reporting_enabled)
        return;

    uint32_t now   = HAL_GetTick();
    int32_t  diff  = (int32_t)(now - last_rpm_tick_ms);
    if (diff < (int32_t)RPM_UPDATE_PERIOD_MS)
        return;

    uint32_t elapsed_ms = (uint32_t)diff;
    last_rpm_tick_ms    = now;

    uint32_t pulses = encoder_pulse_count;
    uint32_t delta  = pulses - last_pulse_snapshot;
    last_pulse_snapshot = pulses;

    uint32_t rpm = compute_scaled_speed(delta,
                                        elapsed_ms,
                                        ENCODER_PULSES_PER_REV,
                                        RPM_SCALE_PER_MINUTE);

    uint32_t rps_x100 = compute_scaled_speed(delta,
                                             elapsed_ms,
                                             ENCODER_PULSES_PER_REV,
                                             RPS_X100_SCALE);

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

/* Command handler ---------------------------------------------------------*/
void handle_line(char *line)
{
    trim(line);
    to_upper(line);

    if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0)
    {
        print_help();
    }
    else if (strcmp(line, "STATUS") == 0)
    {
        uint16_t first = 0xFFFF, last = 0xFFFF;
        (void)fram_get_first_last(&first, &last);
        printf("Logging: %s\r\n", fram_logging_enabled() ? "ON" : "OFF");
        print_session_summary(first, last);
    }
    else if (strcmp(line, "START") == 0)
    {
        fram_cmd_start();
        mark_next_sample_immediate();
        printf("Logging STARTED\r\n");
    }
    else if (strcmp(line, "STOP") == 0)
    {
        fram_cmd_stop();
        printf("Logging STOPPED\r\n");
    }
    else if (strcmp(line, "CLEAN") == 0 || strcmp(line, "CLEAR") == 0)
    {
        fram_cmd_clear();
        mark_next_sample_immediate();
        printf("FRAM CLEARED\r\n");
    }
    else if (strcmp(line, "TEMP") == 0)
    {
        dump_recent_session();
    }
    else if (strcmp(line, "FRAM") == 0)
    {
        uint16_t first = 0xFFFFu, last = 0xFFFFu;
        (void)fram_get_first_last(&first, &last);

        if (last < first)
        {
            printf("No logged samples. Use START to begin logging.\r\n");
        }
        else
        {
            printf("FRAM log dump (first=0x%04X last=0x%04X)\r\n", first, last);
            HAL_StatusTypeDef st = fram_iterate_samples(fram_dump_print_cb);
            if (st != HAL_OK)
            {
                printf("[FRAM] read error during dump (HAL=%d)\r\n", (int)st);
            }
            else
            {
                printf("[FRAM] end of log\r\n");
            }
        }
    }
    else if (strncmp(line, "DUTY", 4) == 0)
    {
        char *arg = line + 4;
        trim(arg);
        if (*arg == '\0')
        {
            printf("Usage: DUTY<0-100>\r\n");
        }
        else
        {
            char *endptr = NULL;
            long duty = strtol(arg, &endptr, 10);
            if ((endptr && *endptr != '\0') || duty < 0 || duty > 100)
            {
                printf("DUTY expects a value between 0 and 100%%\r\n");
            }
            else
            {
                motor_apply_pwm_percent((uint32_t)duty);
                motor_enable_reporting();
                printf("PWM duty set to %ld%%\r\n", duty);
                printf("RPM reporting enabled\r\n");
            }
        }
    }
    else if (strcmp(line, "HALT") == 0)
    {
        motor_apply_pwm_percent(0u);
        motor_disable_reporting();
        printf("Motor HALT: PWM disabled and RPM reporting stopped\r\n");
    }
    else if (line[0] != '\0')
    {
        printf("Unknown: %s\r\n", line);
        print_help();
    }
}

/* GPIO / EXTI init (moved from main.c) -----------------------------------*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* FRAM CS / WP pins */
    GPIO_InitStruct.Pin   = FRAM_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = FRAM_WP_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FRAM_WP_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_WP_GPIO_Port, FRAM_WP_Pin, GPIO_PIN_SET);

    /* Motor encoder on PA0, EXTI rising edge */
    GPIO_InitStruct.Pin   = MOTOR_ENCODER_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_ENCODER_GPIO_Port, &GPIO_InitStruct);

    /* Configure EXTI NVIC, clear pending flags */
    encoder_exti_force_config();
}

/* EXTI NVIC / flags (moved from main.c) ----------------------------------*/
static void encoder_exti_force_config(void)
{
    uint32_t pinmask = MOTOR_ENCODER_Pin;
    uint32_t line    = 0u;

    while (((pinmask & 1u) == 0u) && (line < 16u))
    {
        pinmask >>= 1u;
        line++;
    }

    if (line >= 16u)
    {
        printf("ERROR: bad encoder pin mask 0x%04lX\r\n",
               (unsigned long)MOTOR_ENCODER_Pin);
        return;
    }

    uint32_t linemask = (1u << line);

    EXTI->RPR1 = linemask;
    EXTI->FPR1 = linemask;

    if (line <= 1u)
    {
        HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
    }
    else if (line <= 3u)
    {
        HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
    }
    else
    {
        HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}

/* IRQ callback from HAL (moved from main.c) -------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_ENCODER_Pin)
    {
        static uint32_t last_ms = 0;
        uint32_t now = HAL_GetTick();

        if (now - last_ms < 2)  // ignore repeats < 2 ms apart
            return;

        last_ms = now;

        encoder_pulse_count++;
        encoder_irq_count++;
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
}
