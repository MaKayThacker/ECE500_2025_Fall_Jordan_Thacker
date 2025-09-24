#ifndef USR_USART_H
#define USR_USART_H

#include "stm32g0xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Public UART handle (USART2) ----------------------------------------- */
extern UART_HandleTypeDef huart2;

/* --- Init + non-blocking read API ---------------------------------------- */
/**
 * Initialize USART2 on PA2/PA3 with the given baud (8N1, no flow control).
 * Also enables IRQ and starts an interrupt-driven single-byte receive.
 */
void USR_USART2_UART_Init(uint32_t baud);

/**
 * Non-blocking read of the most recent byte (if any) captured by ISR.
 * @param out_byte   Pointer to receive the byte.
 * @return           true if a byte was returned, false if none available.
 */
bool USR_USART2_TryRead(uint8_t *out_byte);

#ifdef __cplusplus
}
#endif
#endif /* USR_USART_H */
