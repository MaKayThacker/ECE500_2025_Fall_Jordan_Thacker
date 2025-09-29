#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifndef RING_CAP
#define RING_CAP 128u
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ring_init(void);
void ring_clear(void);
int  ring_count(void);
int  ring_is_empty(void);

/* Feed one character received from UART into the ring. */
void ring_add(char ch);

/* Line helpers: a “line” ends with '\r' (CR). CRLF is coalesced to one CR. */
int  ring_has_line(void);                            /* 1 if at least one complete line is present */
int  ring_read_line(char *dst, size_t dstlen);       /* returns length (excludes terminator), 0 if none */

#ifdef __cplusplus
}
#endif
#endif
