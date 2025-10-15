#pragma once
#ifndef USR_USART_H
#define USR_USART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* If your project does NOT already declare `UART_HandleTypeDef huart2;`
   in main.c (CubeMX USART2 enabled), then define this macro BEFORE
   including this header (e.g., in usart.c) to allocate the handle here.

   Example:
     #define USR_DEFINE_HUART2
     #include "usart.h"
*/
void USR_USART2_UART_Init(uint32_t baudrate);

/* Non-blocking read: returns true if a byte was read into *ch, false if none. */
bool USR_USART2_TryRead(uint8_t *ch);

#ifdef __cplusplus
}
#endif
#endif /* USR_USART_H */
