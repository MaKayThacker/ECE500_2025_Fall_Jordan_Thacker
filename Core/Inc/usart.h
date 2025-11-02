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


/* Register-level USART2 init & TX helpers */
void     USR2_USART2_Init(uint32_t baud);
void     usr2_usart2_write(const char *s);
void     usr2_usart2_write_char(char c);

void    USR_SystemClock_Config(void);
uint8_t usr_clock_is_32mhz(void);


#ifdef __cplusplus
}
#endif
#endif /* USR_USART_H */
