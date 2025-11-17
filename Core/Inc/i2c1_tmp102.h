#ifndef I2C1_TMP102_H
#define I2C1_TMP102_H

#include <stdint.h>
#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"   /* for HAL_StatusTypeDef */

/* Debug helpers */
void I2C1_DebugScan(void);
void TMP102_DebugDump(void);

#ifdef __cplusplus
extern "C" {
#endif

/* TMP102 register map */
#define TMP102_ADDR7           0x48u
#define TMP102_REG_TEMP        0x00u
#define TMP102_REG_CONFIG      0x01u

/* Typical 12-bit conversion is ~26–35 ms. */
#define TMP102_CONV_TIME_MS    30u

/* I2C init on PB8/PB9 (AF6), HSI16 kernel, 100 kHz */
void I2C1_Init_PB8PB9_100k_HSI16(void);

/* Simple register access helpers; return 0 on success, <0 on error. */
int I2C1_MemRead (uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len);
int I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len);

/* TMP102 high-level helpers (HAL-style statuses) */
HAL_StatusTypeDef tmp102_initialize(void);
HAL_StatusTypeDef tmp102_trigger_oneshot(void);
HAL_StatusTypeDef tmp102_read_celsius(float *c_out);
HAL_StatusTypeDef tmp102_read_raw(int16_t *raw_out);

/* Debug helpers */
void I2C1_DebugScan(void);
void TMP102_DebugDump(void);

#ifdef __cplusplus
}
#endif

#endif
