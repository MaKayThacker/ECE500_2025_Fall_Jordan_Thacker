#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    USR_SystemClock_Config(void);
uint8_t usr_clock_is_32mhz(void);

#ifdef __cplusplus
}
#endif
