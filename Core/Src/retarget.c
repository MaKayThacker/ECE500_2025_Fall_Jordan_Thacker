#include "main.h"
#include "usart.h"
#include <stdio.h>


/* Provided by usart.c */
void USR_USART2_WriteByte(uint8_t b);


int __io_putchar(int ch)
{
if (ch == '\n') {
USR_USART2_WriteByte('\r');
}
USR_USART2_WriteByte((uint8_t)ch);
return ch;
}
