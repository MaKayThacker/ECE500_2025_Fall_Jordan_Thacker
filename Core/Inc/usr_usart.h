#pragma once
#include "stm32g0xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register-level USART2 init & TX helpers */
void     USR2_USART2_Init(uint32_t baud);
void     usr2_usart2_write(const char *s);
void     usr2_usart2_write_char(char c);

/* Optional polled read (HAL-side file also exposes a try-read API) */
_Bool    USR_USART2_TryRead(uint8_t *ch);  /* from usart.c (HAL path) */

#ifdef __cplusplus
}
#endif
