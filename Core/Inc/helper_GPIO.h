/*
 * helper_GPIO.h
 *
 *  GPIO / motor / logging / UI helpers for main.c
 */

#ifndef INC_HELPER_GPIO_H_
#define INC_HELPER_GPIO_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO + EXTI init */
void MX_GPIO_Init(void);

/* UI helpers */
void print_prompt(void);
void print_help(void);
void print_session_summary(uint16_t first, uint16_t last);

/* Command parsing */
void handle_line(char *line);

/* Logging & FRAM/TMP102 integration */
void service_logging(void);
void mark_next_sample_immediate(void);

/* Motor control / RPM reporting */
void motor_apply_pwm_percent(uint32_t percent);
void motor_enable_reporting(void);
void motor_disable_reporting(void);
void service_motor_rpm(void);

/* Speed computation helper (used internally, but exposed if needed) */
uint32_t compute_scaled_speed(uint32_t delta,
                              uint32_t elapsed_ms,
                              uint32_t pulses_per_rev,
                              uint32_t scale);

/* EXTI callback implemented here (called from stm32g0xx_it.c) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* INC_HELPER_GPIO_H_ */
