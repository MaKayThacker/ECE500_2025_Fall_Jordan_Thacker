#include "main.h"
#include "usart.h"
#include "ring_retarget.h"
#include "fram_spi.h"
#include "i2c1_tmp102.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

SPI_HandleTypeDef hspi1;

static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

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

static void convert_tmp102_raw(int16_t raw, int* c_tenths, int* f_tenths)
{
    int c = (int)((int32_t)raw * 625 / 1000);
    int f = (c * 9) / 5 + 320;
    if (c_tenths) *c_tenths = c;
    if (f_tenths) *f_tenths = f;
}

static uint32_t next_log_ms = 0u;

static void print_prompt(void) {
    printf("> ");
}

static void print_help(void) {
    printf("Commands: START, STOP, TEMP, CLEAN, STATUS, HELP\r\n");
}

static void print_session_summary(uint16_t first, uint16_t last)
{
    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    uint32_t span = (uint32_t)last - (uint32_t)first;
    uint32_t count = span / 2u + 1u;
    printf("Last session: %lu samples (first=0x%04X last=0x%04X)\r\n",
           (unsigned long)count, first, last);
}

static void dump_recent_session(void)
{
    uint16_t first = 0, last = 0;
    (void)fram_get_first_last(&first, &last);

    if (last < first)
    {
        printf("No logged samples. Use START to begin logging.\r\n");
        return;
    }

    uint32_t span = (uint32_t)last - (uint32_t)first;
    uint32_t count = span / 2u + 1u;
    printf("Dumping %lu samples (first=0x%04X last=0x%04X)\r\n",
           (unsigned long)count, first, last);

    uint16_t addr = first;
    for (uint32_t idx = 0; idx < count; ++idx, addr = (uint16_t)(addr + 2u))
    {
        uint8_t be[2] = {0};
        if (fram_read(addr, be, 2) != HAL_OK)
        {
            printf("FRAM read error @0x%04X\r\n", addr);
            break;
        }

        int16_t raw = (int16_t)(((uint16_t)be[0] << 8) | be[1]);
        int c_tenths = 0;
        int f_tenths = 0;
        convert_tmp102_raw(raw, &c_tenths, &f_tenths);
        printf("[%03lu] 0x%04X -> %d.%01d C (%d.%01d F)\r\n",
               (unsigned long)idx,
               addr,
               c_tenths / 10,
               abs(c_tenths % 10),
               f_tenths / 10,
               abs(f_tenths % 10));
    }
}

static void mark_next_sample_immediate(void)
{
    next_log_ms = 0u;
}

static void service_logging(void)
{
    if (!fram_logging_enabled())
    {
        next_log_ms = 0u;
        return;
    }

    uint32_t now = HAL_GetTick();
    if (next_log_ms == 0u)
    {
        next_log_ms = now;
    }

    if ((int32_t)(now - next_log_ms) < 0)
    {
        return;
    }

    next_log_ms = now + 1000u;

    if (tmp102_trigger_oneshot() != HAL_OK)
    {
        printf("[LOG] TMP102 trigger failed\r\n");
        print_prompt();
        return;
    }

    int16_t raw = 0;
    if (tmp102_read_raw(&raw) != HAL_OK)
    {
        printf("[LOG] TMP102 read failed\r\n");
        print_prompt();
        return;
    }

    int c_tenths = 0;
    int f_tenths = 0;
    convert_tmp102_raw(raw, &c_tenths, &f_tenths);

    if (fram_log_sample((uint16_t)raw) != HAL_OK)
    {
        printf("[LOG] FRAM full or write error, logging stopped\r\n");
        print_prompt();
        return;
    }

    printf("[LOG] %d.%01d C (%d.%01d F)\r\n",
           c_tenths / 10,
           abs(c_tenths % 10),
           f_tenths / 10,
           abs(f_tenths % 10));
    print_prompt();
}

static void handle_line(char *line) {
    trim(line);
    to_upper(line);

    if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
        print_help();

    } else if (strcmp(line, "STATUS") == 0) {
        uint16_t first = 0xFFFF, last = 0xFFFF;
        (void)fram_get_first_last(&first, &last);
        printf("Logging: %s\r\n", fram_logging_enabled() ? "ON" : "OFF");
        print_session_summary(first, last);

    } else if (strcmp(line, "START") == 0) {
        fram_cmd_start();
        mark_next_sample_immediate();
        printf("Logging STARTED\r\n");

    } else if (strcmp(line, "STOP") == 0) {
        fram_cmd_stop();
        printf("Logging STOPPED\r\n");

    } else if (strcmp(line, "CLEAN") == 0 || strcmp(line, "CLEAR") == 0) {
        fram_cmd_clear();
        mark_next_sample_immediate();
        printf("FRAM CLEARED\r\n");

    } else if (strcmp(line, "TEMP") == 0) {
        dump_recent_session();

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

    MX_GPIO_Init();
    MX_SPI1_Init();

    MX_USART2_UART_Init();

    (void)tmp102_initialize();
    fram_init_state();

    printf("\r\nUART online (115200 8N1)\r\n");
    print_help();
    print_prompt();

    char line[128];

    while (1) {
        if (ring_has_line()) {
            int n = ring_read_line(line, sizeof line);
            if (n > 0) {
                handle_line(line);
                print_prompt();
            }
        }

        service_logging();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = FRAM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FRAM_WP_Pin;
    HAL_GPIO_Init(FRAM_WP_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_WP_GPIO_Port, FRAM_WP_Pin, GPIO_PIN_SET);
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
