#include "retarget.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>

/* MX generated this handle in main.c/usart.c; declare it here */
extern UART_HandleTypeDef huart2;

#define PRINTF_UART  huart2

int __io_putchar(int ch) {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&PRINTF_UART, &c, 1, HAL_MAX_DELAY);
    return ch;
}

int __io_write(const char *buf, int len) {
    HAL_UART_Transmit(&PRINTF_UART, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

int _write(int file, char *ptr, int len) {
    (void)file;
    return __io_write(ptr, len);
}
