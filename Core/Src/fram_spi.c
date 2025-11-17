
#include "fram_spi.h"
#include "stm32g0xx_hal.h"
#include <string.h>

/* ---- FRAM layout (addresses in bytes) ---- */
#define FRAM_WORD_FIRST_ADDR   0x0000u  /* 1st word: first data address */
#define FRAM_WORD_LAST_ADDR    0x0002u  /* 2nd word: last  data address */
#define FRAM_DATA_START        0x0004u  /* first 16-bit sample goes here */
#define FRAM_BYTES             8192u
#define FRAM_MAX_ADDR          (FRAM_BYTES - 1u)

SPI_HandleTypeDef hspi1;

/* runtime state */
static volatile uint8_t  g_logging_enabled = 0u;
static uint16_t g_first_addr = FRAM_DATA_START;
static uint16_t g_last_addr  = FRAM_DATA_START - 2u;   /* “no data yet” sentinel */

/* ---- Optional WP pin: define here if routed; otherwise this is a no-op ---- */
#ifndef FRAM_WP_GPIO_Port
#define FRAM_WP_GPIO_Port  GPIOC
#endif
#ifndef FRAM_WP_Pin
#define FRAM_WP_Pin        GPIO_PIN_7
#endif

/* =========================
 * Minimal bring-up helpers
 * ========================= */
void FRAM_SPI_PeriphInit(void)
{
    /* Ensure GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB3 as SPI1_SCK AF0 */
    GPIO_InitTypeDef gi = {0};
    gi.Pin       = GPIO_PIN_3;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_HIGH;
    gi.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOB, &gi);

    /* Configure PA6 MISO, PA7 MOSI AF0 */
    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gi.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &gi);
    

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   // Mode 0
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;       // Mode 0
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // ~500 kbit/s at 16 MHz
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    // If you have an error handler, call it here
    // Error_Handler();
  }
}

void FRAM_SPI_GPIO_Init(void)
{
    /* Enable GPIO clocks for CS/WP ports */
    if (FRAM_CS_GPIO_Port == GPIOA || FRAM_WP_GPIO_Port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (FRAM_CS_GPIO_Port == GPIOB || FRAM_WP_GPIO_Port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if (FRAM_CS_GPIO_Port == GPIOC || FRAM_WP_GPIO_Port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin = FRAM_CS_Pin; HAL_GPIO_Init(FRAM_CS_GPIO_Port, &gi);
    gi.Pin = FRAM_WP_Pin; HAL_GPIO_Init(FRAM_WP_GPIO_Port, &gi);

    /* Deselect FRAM and disable write-protect -> CS=HIGH, WP=HIGH */
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_WP_GPIO_Port, FRAM_WP_Pin, GPIO_PIN_SET);
}

/* =========================
 * Low-level FRAM helpers
 * ========================= */
static HAL_StatusTypeDef fram_wren(void)
{
    uint8_t cmd = 0x06u; /* WREN */
    FRAM_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return st;
}

uint8_t fram_rdsr(void)
{
    uint8_t cmd = 0x05u; /* RDSR */
    uint8_t sr  = 0xFFu;
    FRAM_CS_LOW();
    (void)HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    (void)HAL_SPI_Receive (&hspi1, &sr,  1, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return sr;
}

HAL_StatusTypeDef fram_read(uint16_t addr, uint8_t* buf, size_t len)
{
    uint8_t hdr[3] = { 0x03u, (uint8_t)(addr >> 8), (uint8_t)addr };
    FRAM_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return st;
}

HAL_StatusTypeDef fram_write(uint16_t addr, const uint8_t* buf, size_t len)
{
    HAL_StatusTypeDef st = fram_wren();
    if (st != HAL_OK) return st;
    uint8_t hdr[3] = { 0x02u, (uint8_t)(addr >> 8), (uint8_t)addr };
    FRAM_CS_LOW();
    st = HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return st;
}

/* 16-bit big-endian read/write of a word at 'a' */
static HAL_StatusTypeDef fram_write_u16(uint16_t a, uint16_t v)
{
    uint8_t be[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    return fram_write(a, be, 2);
}
static HAL_StatusTypeDef fram_read_u16(uint16_t a, uint16_t* out)
{
    uint8_t be[2] = {0,0};
    HAL_StatusTypeDef st = fram_read(a, be, 2);
    if (st == HAL_OK && out) *out = ((uint16_t)be[0] << 8) | be[1];
    return st;
}

/* returns 1 if the two address words look uninitialized (0xFFFF/0xFFFF or out of range) */
static int fram_address_words_empty(void)
{
    uint16_t f = 0xFFFF, l = 0xFFFF;
    (void)fram_read_u16(FRAM_WORD_FIRST_ADDR, &f);
    (void)fram_read_u16(FRAM_WORD_LAST_ADDR,  &l);
    if (f == 0xFFFF && l == 0xFFFF) return 1;                  /* factory/cleared */
    if (f < FRAM_DATA_START || f > FRAM_MAX_ADDR) return 1;    /* nonsense */
    if (l < (FRAM_DATA_START - 2) || l > FRAM_MAX_ADDR) return 1;
    if ((uint32_t)l + 2u > (uint32_t)FRAM_MAX_ADDR + 1u) return 1;
    return 0;
}

/* =========================
 * Assignment commands
 * ========================= */
bool fram_logging_enabled(void){ return g_logging_enabled != 0u; }

HAL_StatusTypeDef fram_get_first_last(uint16_t* first, uint16_t* last)
{
    if (first) *first = g_first_addr;
    if (last)  *last  = g_last_addr;
    return HAL_OK;
}

void fram_cmd_start(void)
{
    g_logging_enabled = 1u;

    if (fram_address_words_empty()) {
        g_first_addr = FRAM_DATA_START;
        g_last_addr  = FRAM_DATA_START - 2u;   /* no samples yet */
        (void)fram_write_u16(FRAM_WORD_FIRST_ADDR, g_first_addr);
        (void)fram_write_u16(FRAM_WORD_LAST_ADDR,  g_last_addr);
    } else {
        (void)fram_read_u16(FRAM_WORD_FIRST_ADDR, &g_first_addr);
        (void)fram_read_u16(FRAM_WORD_LAST_ADDR,  &g_last_addr);
    }
}

void fram_cmd_stop(void)
{
    g_logging_enabled = 0u;
}

void fram_cmd_clear(void)
{
    /* Mark header as empty by writing 0xFFFF to both words */
    uint16_t ff = 0xFFFFu;
    (void)fram_write(FRAM_WORD_FIRST_ADDR, (uint8_t*)&ff, 2);
    (void)fram_write(FRAM_WORD_LAST_ADDR,  (uint8_t*)&ff, 2);
    g_first_addr = FRAM_DATA_START;
    g_last_addr  = FRAM_DATA_START - 2u;
}

/* Pass the raw 16-bit TMP102 reading here once per second */
void fram_log_sample_on_data(uint16_t tmp102_raw)
{
    if (!g_logging_enabled) return;

    uint16_t next = (uint16_t)(g_last_addr + 2u);
    if (next > (FRAM_MAX_ADDR - 1u)) {
        /* out of space – stop logging gracefully */
        g_logging_enabled = 0u;
        return;
    }

    /* store as two bytes, big-endian */
    (void)fram_write_u16(next, (uint16_t)tmp102_raw);

    /* update last address word in FRAM */
    g_last_addr = next;
    (void)fram_write_u16(FRAM_WORD_LAST_ADDR, g_last_addr);
}
