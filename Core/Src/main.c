#include "stm32g0xx.h"
#include "usr_clock.h"
#include "usr_usart.h"
#include "i2c1.h"
#include <stdio.h>
#include <stdarg.h>

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; ++i)
        for (volatile uint32_t j = 0; j < 32000u; ++j) { __NOP(); }
}

static int tmp102_read_celsius(float *out_c)
{
    const uint8_t a = 0x48;
    uint8_t b[2];
    delay_ms(5);
    int rc = I2C1_MemRead(a, 0x00, b, 2);
    if (rc) return rc;

    int16_t raw = ((int16_t)b[0] << 8) | b[1];
    raw >>= 4;
    if (raw & 0x0800) raw |= 0xF000;
    *out_c = (float)raw * 0.0625f;
    return 0;
}

int main(void)
{
    USR_SystemClock_Config();
    USR2_USART2_Init(115200);
    usr2_usart2_write("\r\n=== ECE500 6.2: I2C/TMP102 demo ===\r\n");
    usr2_usart2_write(usr_clock_is_32mhz() ? "Clock OK: 32 MHz\r\n" : "Clock Fallback: 16 MHz\r\n");

    I2C1_Init_PB8PB9_100k_HSI16();

    uint8_t probe[2];
    if (I2C1_MemRead(0x48, 0x00, probe, 2) == 0) usr2_usart2_write("TMP102 present\r\n");
    else                                         usr2_usart2_write("TMP102 not responding\r\n");

    while (1) {
        float tC;
        int rc = tmp102_read_celsius(&tC);
        if (rc == 0) {
            int temp100 = (int)(tC * 100.0f);
            int whole = temp100 / 100;
            int frac  = (temp100 < 0 ? -temp100 : temp100) % 100;
            char msg[32];
            sprintf(msg, "Temp: %d.%02d C\r\n", whole, frac);
            usr2_usart2_write(msg);
        } else {
            char msg[32];
            sprintf(msg, "I2C error: %d\r\n", rc);
            usr2_usart2_write(msg);
        }
        delay_ms(1000);
    }
}
