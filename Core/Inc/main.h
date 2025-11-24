/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* ---------------------------------------------------------------------------
 * GPIO Pin Definitions (MATCHED TO YOUR ORIGINAL + SCHEMATIC)
 * ---------------------------------------------------------------------------*/

/* Clock Output */
#define MCO_Pin             GPIO_PIN_0
#define MCO_GPIO_Port       GPIOF

/* UART2 */
#define USART2_TX_Pin       GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin       GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA

/* LED */
#define LED_GREEN_Pin       GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA

/* FRAM */
#define FRAM_CS_Pin         GPIO_PIN_0
#define FRAM_CS_GPIO_Port   GPIOB
#define FRAM_WP_Pin         GPIO_PIN_7
#define FRAM_WP_GPIO_Port   GPIOC

/* MOTOR CONTROL */
#define MOTOR_PWM_Pin        GPIO_PIN_4
#define MOTOR_PWM_GPIO_Port  GPIOA

#define MOTOR_ENCODER_Pin        GPIO_PIN_0
#define MOTOR_ENCODER_GPIO_Port  GPIOA

/* SWD Debug */
#define TMS_Pin            GPIO_PIN_13
#define TMS_GPIO_Port      GPIOA
#define TCK_Pin            GPIO_PIN_14
#define TCK_GPIO_Port      GPIOA

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
