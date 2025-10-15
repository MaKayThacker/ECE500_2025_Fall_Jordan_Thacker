#include "stm32g0xx.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "usr_clock.h"      // USR_SystemClock_Config, usr_clock_is_32mhz
#include "usr_usart.h"      // USR2_USART2_Init, usr2_usart2_write
#include "i2c1.h"            // MX_I2C1_Init (hi2c1 for tmp102.c)
#include "tmp102.h"         // tmp102_initialize/trigger_oneshot/read_celsius

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
    USR_SystemClock_Config();          // your 32 MHz clock
    USR2_USART2_Init(115200);          // your UART init
    I2C1_Init_PB8PB9_100k_HSI16();     // your I2C1 init on PB8/PB9

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
            float c = 0.0f;
            if (tmp102_read_celsius(&c) == HAL_OK) {
                float f = c * 9.0f / 5.0f + 32.0f;
                putf("Temp: %.2f C (%.2f F)\r\n", c, f);
            } else {
                usr2_usart2_write("TMP102 read ERROR\r\n");
            }
        } else {
            usr2_usart2_write("TMP102 one-shot trigger ERROR\r\n");
        }

        HAL_Delay(1000);                 /* print every 1s */
    }
}
