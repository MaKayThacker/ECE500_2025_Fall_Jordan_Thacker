#include "stm32g0xx.h"
#include "i2c1.h"

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

    /* Kernel clock = HSI16 so timing value is stable */
#if defined(RCC_CCIPR_I2C1SEL_Pos)
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2C1SEL_Msk) | (0x2u << RCC_CCIPR_I2C1SEL_Pos);
#endif

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x00503D58; /* 100 kHz @ 16 MHz kernel */
    I2C1->CR1 |= I2C_CR1_PE;
}

int I2C1_MemRead(uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len)
{
    if (!len) return -1;
    while (I2C1->ISR & I2C_ISR_BUSY) { }

    /* write reg addr */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | (1u << 16);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return -2; }
    }
    I2C1->TXDR = reg;
    while (!(I2C1->ISR & I2C_ISR_TC)) {
        if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return -3; }
    }

    /* read len bytes */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | (len << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint32_t i = 0; i < len; ++i) {
        while (!(I2C1->ISR & I2C_ISR_RXNE)) {
            if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return -4; }
        }
        buf[i] = (uint8_t)I2C1->RXDR;
    }
    while (!(I2C1->ISR & I2C_ISR_STOPF)) { }
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

int I2C1_MemWrite(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    while (I2C1->ISR & I2C_ISR_BUSY) { }

    /* Single write: send reg + payload in one AUTOEND transfer */
    I2C1->CR2 = ((uint32_t)addr7 << 1) | ((uint32_t)(len + 1) << 16);
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;                 /* write */
    I2C1->CR2 |=  I2C_CR2_START | I2C_CR2_AUTOEND;

    /* reg */
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return -1; }
    }
    I2C1->TXDR = reg;

    /* payload */
    for (uint32_t i = 0; i < len; ++i) {
        while (!(I2C1->ISR & I2C_ISR_TXIS)) {
            if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return -2; }
        }
        I2C1->TXDR = buf[i];
    }

    while (!(I2C1->ISR & I2C_ISR_STOPF)) { }
    I2C1->ICR = I2C_ICR_STOPCF;
    return 0;
}

