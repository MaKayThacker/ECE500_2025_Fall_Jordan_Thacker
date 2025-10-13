#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>

/* I2C1 on PB8=SCL / PB9=SDA, AF6, kernel clock = HSI16, 100 kHz */
void I2C1_Init_PB8PB9_100k_HSI16(void);

/* Read "len" bytes from 8-bit register "reg" at 7-bit addr "addr7" */
int I2C1_MemRead(uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len);

/* New write helper to pair with I2C1_MemRead */
int I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len);

/* TMP102 init (returns HAL_OK on success, HAL_ERROR on failure) */
HAL_StatusTypeDef tmp102_initialize(void);

#endif
