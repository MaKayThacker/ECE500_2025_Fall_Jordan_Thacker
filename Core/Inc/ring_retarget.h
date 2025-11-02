
#pragma once
#include <stddef.h>

#define RING_CAP 128u

#ifndef INC_RING_RETARGET_H_
#define INC_RING_RETARGET_H_

int __io_putchar(int ch);
int __io_write(const char *buf, int len);

void  ring_init(void);
void  ring_clear(void);
int   ring_count(void);
int   ring_is_empty(void);
int   ring_has_line(void);
void  ring_add(char ch);                    /* echoes as it receives */
int   ring_read_line(char *dst, size_t n);  /* returns length, 0 if none */


#endif
