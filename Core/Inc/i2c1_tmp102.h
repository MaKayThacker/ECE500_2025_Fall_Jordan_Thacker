/*
 * i2c1_tmp102.h
 *
 *  Created on: Nov 2, 2025
 *      Author: Pengu
 */

#ifndef INC_I2C1_TMP102_H_
#define INC_I2C1_TMP102_H_

#include <stdint.h>

void I2C1_Init_PB8PB9_100k_HSI16(void);
int  I2C1_MemRead (uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len);
int  I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len);

#define TMP102_ADDR7          0x48u    // change if A0/A1 strap differ
#define TMP102_REG_TEMP       0x00u
#define TMP102_REG_CONFIG     0x01u
#define TMP102_CONV_TIME_MS   30u

HAL_StatusTypeDef tmp102_initialize(void);
HAL_StatusTypeDef tmp102_trigger_oneshot(void);
HAL_StatusTypeDef tmp102_read_celsius(float *c_out);
HAL_StatusTypeDef tmp102_read_raw(int16_t *raw_out);


#endif /* INC_I2C1_TMP102_H_ */
