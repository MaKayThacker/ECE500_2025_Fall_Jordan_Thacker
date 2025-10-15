#pragma once
#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>

void I2C1_Init_PB8PB9_100k_HSI16(void);
int  I2C1_MemRead (uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len);
int  I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len);

#endif
