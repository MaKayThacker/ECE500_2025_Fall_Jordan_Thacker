#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"
#include "i2c1_tmp102.h"
#include <stdio.h>

/* ===== Low-level I2C1 (register-level, NO HAL handle) ===== */

void I2C1_Init_PB8PB9_100k_HSI16(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOBEN;
    RCC->APBENR1 |= RCC_APBENR1_I2C1EN;

    /* PB8=SCL, PB9=SDA (AF6), open-drain, pull-ups, medium speed */
    GPIOB->MODER   &= ~((3u<<(8*2)) | (3u<<(9*2)));
    GPIOB->MODER   |=  ((2u<<(8*2)) | (2u<<(9*2)));
    GPIOB->AFR[1]  &= ~((0xFu<<((8-8)*4)) | (0xFu<<((9-8)*4)));
    GPIOB->AFR[1]  |=  ((6u<<((8-8)*4)) | (6u<<((9-8)*4)));
    GPIOB->OTYPER  |=  (1u<<8) | (1u<<9);
    GPIOB->PUPDR   &= ~((3u<<(8*2)) | (3u<<(9*2)));
    GPIOB->PUPDR   |=  ((1u<<(8*2)) | (1u<<(9*2)));
    GPIOB->OSPEEDR |=  ((1u<<(8*2)) | (1u<<(9*2)));

    /* I2C1 kernel clock = HSI16 (stable) */
#if defined(RCC_CCIPR_I2C1SEL_Pos)
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2C1SEL_Msk) | (0x2u << RCC_CCIPR_I2C1SEL_Pos);
#endif

    I2C1->CR1 &= ~I2C_CR1_PE;
    /* Timing: 100 kHz @ 16 MHz kernel clock */
    I2C1->TIMINGR = 0x00503D58u;
    I2C1->CR1 |= I2C_CR1_PE;
}

/* Small bounded-spin helper */
static inline int wait_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t max_iter)
{
    while (((*reg) & mask) == 0u) {
        if (max_iter-- == 0u) return -1;
    }
    return 0;
}
static inline int wait_flag_clr(volatile uint32_t *reg, uint32_t mask, uint32_t max_iter)
{
    while (((*reg) & mask) != 0u) {
        if (max_iter-- == 0u) return -1;
    }
    return 0;
}

#define I2C_SPIN  200000u

int I2C1_MemRead(uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len)
{
    if (!len || !buf) return -1;
    if (wait_flag_clr(&I2C1->ISR, I2C_ISR_BUSY, I2C_SPIN)) return -10;

    /* Write register address */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | (1u << 16); /* NBYTES=1 */
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |=  I2C_CR2_START;
    if (wait_flag_set(&I2C1->ISR, I2C_ISR_TXIS, I2C_SPIN)) return -11;
    I2C1->TXDR = reg;
    if (wait_flag_set(&I2C1->ISR, I2C_ISR_TC, I2C_SPIN))   return -12;

    /* Read N bytes */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | (len << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint32_t i = 0; i < len; ++i) {
        if (wait_flag_set(&I2C1->ISR, I2C_ISR_RXNE, I2C_SPIN)) return -13;
        buf[i] = (uint8_t)I2C1->RXDR;
    }
    if (wait_flag_set(&I2C1->ISR, I2C_ISR_STOPF, I2C_SPIN)) return -14;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

int I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    if (!buf && len) return -1;
    if (wait_flag_clr(&I2C1->ISR, I2C_ISR_BUSY, I2C_SPIN)) return -10;

    /* Send reg + payload in single AUTOEND transfer */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | ((uint32_t)(len + 1u) << 16);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |=  I2C_CR2_START | I2C_CR2_AUTOEND;

    if (wait_flag_set(&I2C1->ISR, I2C_ISR_TXIS, I2C_SPIN)) return -11;
    I2C1->TXDR = reg;

    for (uint32_t i = 0; i < len; ++i) {
        if (wait_flag_set(&I2C1->ISR, I2C_ISR_TXIS, I2C_SPIN)) return -12;
        I2C1->TXDR = buf[i];
    }
    if (wait_flag_set(&I2C1->ISR, I2C_ISR_STOPF, I2C_SPIN)) return -13;
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

/* ===== TMP102 helpers (register-level, bounded waits) ===== */

#define TMP102_CFG_OS_BIT   (1u << 15)
#define TMP102_CFG_SD_BIT   (1u << 0)

static HAL_StatusTypeDef rd_cfg(uint16_t *cfg) {
    if (!cfg) return HAL_ERROR;
    uint8_t rx[2];
    if (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_CONFIG, rx, 2) != 0) return HAL_ERROR;
    *cfg = (uint16_t)((rx[0] << 8) | rx[1]);
    return HAL_OK;
}
static HAL_StatusTypeDef wr_cfg(uint16_t cfg) {
    uint8_t tx[2] = { (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    return (I2C1_MemWrite(TMP102_ADDR7, TMP102_REG_CONFIG, tx, 2) == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef tmp102_initialize(void)
{
    uint16_t cfg=0;
    if (rd_cfg(&cfg) != HAL_OK) return HAL_ERROR;
    /* Put device into shutdown so one-shot works as expected */
    if (wr_cfg(cfg | TMP102_CFG_SD_BIT) != HAL_OK) return HAL_ERROR;

    /* Dummy read to verify bus */
    uint8_t t[2];
    return (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_TEMP, t, 2) == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef tmp102_trigger_oneshot(void)
{
    uint16_t cfg=0;
    if (rd_cfg(&cfg) != HAL_OK) return HAL_ERROR;
    cfg |= TMP102_CFG_SD_BIT | TMP102_CFG_OS_BIT;
    if (wr_cfg(cfg) != HAL_OK) return HAL_ERROR;

    /* Wait typical conversion time; bounded so caller can't hang */
    HAL_Delay(TMP102_CONV_TIME_MS);
    return HAL_OK;
}

HAL_StatusTypeDef tmp102_read_celsius(float *c_out)
{
    if (!c_out)
        return HAL_ERROR;

    uint8_t rx[2];
    if (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_TEMP, rx, 2) != 0)
        return HAL_ERROR;

    int16_t raw = (int16_t)((rx[0] << 8) | rx[1]);
    raw >>= 4;
    if (raw & 0x0800)
        raw |= (int16_t)~0x0FFF;

    *c_out = (float)raw * 0.0625f;
    return HAL_OK;
}

HAL_StatusTypeDef tmp102_read_raw(int16_t *raw_out)
{
    if (!raw_out)
        return HAL_ERROR;

    uint8_t rx[2];
    if (I2C1_MemRead(TMP102_ADDR7, TMP102_REG_TEMP, rx, 2) != 0)
        return HAL_ERROR;

    int16_t raw = (int16_t)((rx[0] << 8) | rx[1]);
    raw >>= 4;
    if (raw & 0x0800)
        raw |= (int16_t)~0x0FFF;

    *raw_out = raw;
    return HAL_OK;
}

void I2C1_DebugScan(void)
{
    printf("I2C1 quick probe:\r\n");
    uint8_t cfg;
    int ok48 = I2C1_MemRead(0x48u, TMP102_REG_CONFIG, &cfg, 1);
    int ok49 = I2C1_MemRead(0x49u, TMP102_REG_CONFIG, &cfg, 1);

    if (ok48 == 0) printf("  - device @ 0x48\r\n");
    if (ok49 == 0) printf("  - device @ 0x49\r\n");
    if (ok48 != 0 && ok49 != 0) printf("  (no TMP102 found at 0x48/0x49)\r\n");
}

void TMP102_DebugDump(void)
{
    uint8_t cfg[2] = {0};
    int ok = I2C1_MemRead(TMP102_ADDR7, TMP102_REG_CONFIG, cfg, 2);

    if (ok == 0) {
        printf("TMP102 config: 0x%02X 0x%02X (addr=0x%02X)\r\n",
               cfg[0], cfg[1], TMP102_ADDR7);
    } else {
        printf("TMP102 config read failed at 0x%02X\r\n", TMP102_ADDR7);
    }
}

