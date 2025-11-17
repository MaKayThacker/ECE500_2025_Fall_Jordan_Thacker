#ifndef RING_RETARGET_H
#define RING_RETARGET_H

#include <stddef.h>
#include <stdint.h>

/* Ring buffer capacity MUST be a power of two. */
#ifndef RING_CAP
#define RING_CAP 256u
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ring_init(void);
void ring_clear(void);
int  ring_is_empty(void);
int  ring_count(void);
int  ring_has_line(void);
void ring_add(char ch);
int  ring_read_line(char *dst, size_t n);

/* Start UART RX interrupts after MX_USART2_UART_Init(). */
void ring_uart_start(void);

/* printf retargets */
int __io_putchar(int ch);
int __io_write(const char *buf, int len);
int _write(int fd, const void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* RING_RETARGET_H */
