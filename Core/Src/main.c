#include "main.h"
#include "usart.h"
#include "ring_retarget.h"
#include "fram_spi.h"
#include "i2c1_tmp102.h"
#include "helper_GPIO.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim14;

/* ----- Motor / encoder config ----- */
#define MOTOR_PWM_PRESCALER_VALUE   (16u - 1u)   /* 16 MHz / 16 -> 1 MHz timer clock */
#define MOTOR_PWM_PERIOD_TICKS      (1000u - 1u) /* 1 MHz / 1000 -> 1 kHz PWM */
#define ENCODER_PULSES_PER_REV      20u          /* pulses per mechanical revolution */
#define RPM_UPDATE_PERIOD_MS        1000u        /* report every 1 second */
#define RPM_SCALE_PER_MINUTE        60000u       /* ms/min for rpm conversion */
#define RPS_X100_SCALE              100000u      /* (100 * 1000) for rps w/2 decimal digits */

/* Prototypes (GPIO + helpers now live in helpers_GPIO.c) */
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);

/* Weak default so linker is happy if Cube defines a strong version. */
__attribute__((weak)) void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    (void)htim;
}

/* Main --------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();

    /* I2C + GPIO + SPI + PWM timer + UART */
    I2C1_Init_PB8PB9_100k_HSI16();
    MX_GPIO_Init();         /* now implemented in helpers_GPIO.c */
    MX_SPI1_Init();
    MX_TIM14_Init();
    MX_USART2_UART_Init();

    if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    motor_apply_pwm_percent(0u);

    (void)tmp102_initialize();
    fram_init_state();

    printf("\r\nUART online (115200 8N1)\r\n");
    print_help();
    print_prompt();

    char line[128];

    while (1)
    {
        if (ring_has_line())
        {
            int n = ring_read_line(line, sizeof line);
            if (n > 0)
            {
                handle_line(line);
                print_prompt();
            }
        }

        service_logging();
        service_motor_rpm();
    }
}

/* SPI / TIM init ----------------------------------------------------------*/
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

static void MX_TIM14_Init(void)
{
    htim14.Instance               = TIM14;
    htim14.Init.Prescaler         = MOTOR_PWM_PRESCALER_VALUE;
    htim14.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim14.Init.Period            = MOTOR_PWM_PERIOD_TICKS;
    htim14.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
        Error_Handler();

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0u;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim14);
}

/* Error handler -----------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
