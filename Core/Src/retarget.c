#include "retarget.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

/* MX generated this handle in main.c/usart.c; declare it here */
extern UART_HandleTypeDef huart2;

#define PRINTF_UART  huart2

static inline uint32_t uart_tx_ready(USART_TypeDef *U)
{
#ifdef USART_ISR_TXE_TXFNF  // G0/G4/L4 etc. (FIFO present)
    return (U->ISR & USART_ISR_TXE_TXFNF);
#else                       // F1/F3 etc. (no FIFO)
    return (U->ISR & USART_ISR_TXE);
#endif
}

int __io_putchar(int ch)
{
    USART_TypeDef *U = PRINTF_UART.Instance;

    if (ch == '\n') {
        while (!uart_tx_ready(U)) { /* wait */ }
        U->TDR = (uint8_t)'\r';
    }
    while (!uart_tx_ready(U)) { /* wait */ }
    U->TDR = (uint8_t)ch;

    return ch;
}


int __io_write(const char *buf, int len) {
    HAL_UART_Transmit(&PRINTF_UART, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

int _write(int fd, const void *buf, size_t cnt)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        const uint8_t *p = (const uint8_t *)buf;
        for (size_t i = 0; i < cnt; ++i) {
            uint8_t ch = p[i];
            if (ch == '\n') {
                while ((USART2->ISR & USART_ISR_TXE_TXFNF) == 0u) {;}
                USART2->TDR = (uint8_t)'\r';
            }
            while ((USART2->ISR & USART_ISR_TXE_TXFNF) == 0u) {;}
            USART2->TDR = ch;
        }
        return (int)cnt;
    }
    return (int)cnt; // ignore other fds
}
