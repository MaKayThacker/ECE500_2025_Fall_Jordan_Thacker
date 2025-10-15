#include "tmp102.h"
#include "i2c1.h"

#define TMP102_CFG_OS_BIT   (1u << 15)
#define TMP102_CFG_SD_BIT   (1u << 0)

static HAL_StatusTypeDef rd_cfg(uint16_t *cfg) {
    uint8_t rx[2];
    if (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_CONFIG, rx, 2)) return HAL_ERROR;
    *cfg = ((uint16_t)rx[0] << 8) | rx[1];
    return HAL_OK;
}
static HAL_StatusTypeDef wr_cfg(uint16_t cfg) {
    uint8_t tx[2] = { (uint8_t)(cfg >> 8), (uint8_t)cfg };
    return I2C1_MemWrite(TMP102_ADDR7, TMP102_REG_CONFIG, tx, 2) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef tmp102_initialize(void)
{
    uint16_t cfg=0;
    if (rd_cfg(&cfg) != HAL_OK) return HAL_ERROR;
    if (wr_cfg(cfg | TMP102_CFG_SD_BIT) != HAL_OK) return HAL_ERROR;

    uint8_t t[2];
    return I2C1_MemRead(TMP102_ADDR7, TMP102_REG_TEMP, t, 2) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef tmp102_trigger_oneshot(void)
{
    uint16_t cfg=0;
    if (rd_cfg(&cfg) != HAL_OK) return HAL_ERROR;
    cfg |= TMP102_CFG_SD_BIT | TMP102_CFG_OS_BIT;
    if (wr_cfg(cfg) != HAL_OK) return HAL_ERROR;
    HAL_Delay(TMP102_CONV_TIME_MS);
    return HAL_OK;
}

HAL_StatusTypeDef tmp102_read_celsius(float *c_out)
{
    if (!c_out) return HAL_ERROR;
    uint8_t rx[2];
    if (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_TEMP, rx, 2)) return HAL_ERROR;

    int16_t raw = ((int16_t)rx[0] << 8) | rx[1];
    raw >>= 4;
    if (raw & 0x0800) raw |= (int16_t)~0x0FFF;
    *c_out = (float)raw * 0.0625f;
    return HAL_OK;
}
