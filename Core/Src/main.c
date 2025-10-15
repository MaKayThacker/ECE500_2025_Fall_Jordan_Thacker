#include "stm32g0xx.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "usr_clock.h"      // USR_SystemClock_Config, usr_clock_is_32mhz
#include "usr_usart.h"      // USR2_USART2_Init, usr2_usart2_write
#include "i2c1.h"           // I2C1_Init_PB8PB9_100k_HSI16
#include "tmp102.h"         // tmp102_initialize / trigger_oneshot / read_raw

/* tiny printf -> your UART */
static void putf(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    usr2_usart2_write(buf);
}

int main(void)
{
    HAL_Init();
    USR_SystemClock_Config();          // 32 MHz PLL clock
    USR2_USART2_Init(115200);          // UART2 @115200
    I2C1_Init_PB8PB9_100k_HSI16();     // I2C1 on PB8/PB9

    usr2_usart2_write("\r\n=== ECE500: TMP102 One-Shot Demo ===\r\n");
    usr2_usart2_write(usr_clock_is_32mhz()
        ? "Clock OK: 32 MHz\r\n" : "Clock Fallback: 16 MHz\r\n");

    if (tmp102_initialize() != HAL_OK) {
        usr2_usart2_write("TMP102 init FAILED\r\n");
    } else {
        usr2_usart2_write("TMP102 init OK (shutdown + one-shot ready)\r\n");
    }

    for (;;) {
        if (tmp102_trigger_oneshot() == HAL_OK) {

            int16_t raw = 0;
            if (tmp102_read_raw(&raw) == HAL_OK) {
                /* TMP102: 12-bit result left-justified in 16-bit word
                   1 LSB = 0.0625 °C = 0.625 tenths °C */
                int c_tenths = ((raw >> 4) * 625) / 100;   // °C ×10
                int f_tenths = (c_tenths * 9) / 5 + 320;   // °F ×10

                putf("Temp: %d.%01d C (%d.%01d F)\r\n",
                     c_tenths / 10, abs(c_tenths % 10),
                     f_tenths / 10, abs(f_tenths % 10));
            } else {
                usr2_usart2_write("TMP102 read ERROR\r\n");
            }

        } else {
            usr2_usart2_write("TMP102 one-shot trigger ERROR\r\n");
        }

        HAL_Delay(1000);   // print every second
    }
}
