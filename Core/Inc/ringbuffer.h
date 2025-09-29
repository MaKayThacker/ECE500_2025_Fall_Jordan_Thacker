#pragma once
#include <stddef.h>

#define RING_CAP 128u

#ifdef __cplusplus
extern "C" {
#endif

void  ring_init(void);
void  ring_clear(void);
int   ring_count(void);
int   ring_is_empty(void);
int   ring_has_line(void);
void  ring_add(char ch);                    /* echoes as it receives */
int   ring_read_line(char *dst, size_t n);  /* returns length, 0 if none */

#ifdef __cplusplus
}
#endif
