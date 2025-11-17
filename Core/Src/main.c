#include "main.h"
#include "usart.h"
#include "ring_retarget.h"
#include "fram_spi.h"
#include "i2c1_tmp102.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void to_upper(char *s) {
    if (!s) return;
    for (; *s; ++s) {
        if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 'a' + 'A');
    }
}

static void trim(char *s) {
    if (!s) return;
    char *p = s;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n')) {
        s[--n] = '\0';
    }
}

static void print_help(void) {
    printf("Commands: START, TEMP, CLEAN, STOP, STATUS, HELP\r\n");
}

static void handle_line(char *line) {
    trim(line);
    to_upper(line);

    if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
        print_help();

    } else if (strcmp(line, "STATUS") == 0) {
        uint16_t first = 0xFFFF, last = 0xFFFF;
        (void)fram_get_first_last(&first, &last);
        printf("Logging: %s, First=0x%04X Last=0x%04X\r\n",
               fram_logging_enabled() ? "ON" : "OFF",
               (unsigned)first, (unsigned)last);

    } else if (strcmp(line, "START") == 0) {
        fram_cmd_start();
        printf("Logging STARTED\r\n");

    } else if (strcmp(line, "STOP") == 0) {
        fram_cmd_stop();
        printf("Logging STOPPED\r\n");

    } else if (strcmp(line, "CLEAN") == 0 || strcmp(line, "CLEAR") == 0) {
        fram_cmd_clear();
        printf("FRAM CLEARED\r\n");

    } else if (strcmp(line, "TEMP") == 0) {
        int16_t raw = 0;
        HAL_StatusTypeDef st = tmp102_read_raw(&raw);
        if (st == HAL_OK) {
            int c_tenths = ((raw >> 4) * 625) / 100;
            int f_tenths = (c_tenths * 9) / 5 + 320;
            printf("Temp: %d.%01d C (%d.%01d F)\r\n",
                   c_tenths/10, abs(c_tenths%10),
                   f_tenths/10, abs(f_tenths%10));
            fram_log_sample_on_data((uint16_t)raw);
        } else {
            printf("TEMP read ERROR\r\n");
            I2C1_DebugScan();
            TMP102_DebugDump();
            printf("If no device @ 0x%02X, check ADD0 strap & change TMP102_ADDR7.\r\n", TMP102_ADDR7);
        }

    } else if (line[0] != '\0') {
        printf("Unknown: %s\r\n", line);
        print_help();
    }
}

int main(void)
{
    HAL_Init();

    /* Configure I2C1 on PB8/PB9 at 100 kHz using HSI16 as kernel clock */
    I2C1_Init_PB8PB9_100k_HSI16();

    FRAM_SPI_GPIO_Init();
    FRAM_SPI_PeriphInit();

    MX_USART2_UART_Init();

    (void)tmp102_initialize();

    printf("\r\nUART online (115200 8N1)\r\n");
    print_help();
    printf("> ");

    char line[128];

    while (1) {
        int n = ring_read_line(line, sizeof line);
        if (n > 0) {
            handle_line(line);
            printf("> ");
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
