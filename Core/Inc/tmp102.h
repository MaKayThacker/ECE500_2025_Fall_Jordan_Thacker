#pragma once
#include "main.h"    // HAL_StatusTypeDef + HAL_Delay
#include <stdint.h>

#define TMP102_ADDR7          0x48u    // change if A0/A1 strap differ
#define TMP102_REG_TEMP       0x00u
#define TMP102_REG_CONFIG     0x01u
#define TMP102_CONV_TIME_MS   30u

HAL_StatusTypeDef tmp102_initialize(void);
HAL_StatusTypeDef tmp102_trigger_oneshot(void);
HAL_StatusTypeDef tmp102_read_celsius(float *c_out);
HAL_StatusTypeDef tmp102_read_raw(int16_t *raw_out);
