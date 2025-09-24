#include "retarget.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

/* --- UART chosen for printf redirection --------------------------------- */
extern UART_HandleTypeDef huart2;
#define PRINTF_UART huart2

/* --- Helpers ------------------------------------------------------------- */
static inline uint32_t tx_ready(USART_TypeDef *U) {
#ifdef USART_ISR_TXE_TXFNF
    return (U->ISR & USART_ISR_TXE_TXFNF); // G0 family
#else
    return (U->ISR & USART_ISR_TXE);       // other families
#endif
}

/* --- Low-level putchar (used by printf) --------------------------------- */
int __io_putchar(int ch) {
    USART_TypeDef *U = PRINTF_UART.Instance;

    if (ch == '\n') {
        while (!tx_ready(U)) {}
        U->TDR = (uint8_t)'\r';
    }
    while (!tx_ready(U)) {}
    U->TDR = (uint8_t)ch;
    return ch;
}

/* --- Direct write (used by printf library) ------------------------------- */
int __io_write(const char *buf, int len) {
    HAL_UART_Transmit(&PRINTF_UART, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* --- syscalls.c stub: _write() ------------------------------------------ */
int _write(int fd, const void *buf, size_t cnt) {
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        const uint8_t *p = (const uint8_t *)buf;
        for (size_t i = 0; i < cnt; ++i) {
            __io_putchar(p[i]);
        }
        return (int)cnt;
    }
    return (int)cnt;
}
