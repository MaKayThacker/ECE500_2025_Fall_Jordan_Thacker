#include "fram_spi.h"
#include <string.h>

/* ---- FRAM layout (addresses in bytes) ---- */
#define FRAM_WORD_FIRST_ADDR   0x0000u  /* 1st word: first data address */
#define FRAM_WORD_LAST_ADDR    0x0002u  /* 2nd word: last  data address */
#define FRAM_DATA_START        0x0004u  /* first 16-bit sample goes here */
#define FRAM_BYTES             8192u
#define FRAM_MAX_ADDR          (FRAM_BYTES - 1u)

/* runtime state */
static volatile uint8_t  g_logging_enabled = 0u;
static uint16_t g_first_addr = FRAM_DATA_START;
static uint16_t g_last_addr  = FRAM_DATA_START - 2u;   /* “no data yet” sentinel */

static void fram_reload_address_words(void);
static HAL_StatusTypeDef fram_write_u16(uint16_t addr, uint16_t value);
static HAL_StatusTypeDef fram_read_u16(uint16_t addr, uint16_t* out);

/* =========================
 * Helper utilities
 * ========================= */
static void fram_reload_address_words(void)
{
    uint16_t first = 0xFFFFu;
    uint16_t last  = 0xFFFFu;

    (void)fram_read_u16(FRAM_WORD_FIRST_ADDR, &first);
    (void)fram_read_u16(FRAM_WORD_LAST_ADDR,  &last);

    if (first == 0xFFFFu && last == 0xFFFFu)
    {
        g_first_addr = FRAM_DATA_START;
        g_last_addr  = FRAM_DATA_START - 2u;
        (void)fram_write_u16(FRAM_WORD_FIRST_ADDR, g_first_addr);
        (void)fram_write_u16(FRAM_WORD_LAST_ADDR,  g_last_addr);
        return;
    }

    if (first < FRAM_DATA_START || first > FRAM_MAX_ADDR)
    {
        first = FRAM_DATA_START;
    }

    uint16_t min_last = (uint16_t)(first - 2u);
    if (last < min_last || last > FRAM_MAX_ADDR)
    {
        last = min_last;
    }

    g_first_addr = first;
    g_last_addr  = last;
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

/* FRAM API: raw read/write */
HAL_StatusTypeDef fram_read(uint16_t addr, uint8_t* buf, size_t len)
{
    if (buf == NULL || len == 0u)
    {
        return HAL_OK; /* nothing to do */
    }

    if (addr > FRAM_MAX_ADDR)
    {
        return HAL_ERROR;
    }

    /* Clamp length so we don't run past FRAM end */
    size_t maxLen = (size_t)(FRAM_BYTES - addr);
    if (len > maxLen)
    {
        len = maxLen;
    }

    uint8_t hdr[3] = { 0x03u, (uint8_t)(addr >> 8), (uint8_t)addr }; /* READ opcode */
    FRAM_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    }
    FRAM_CS_HIGH();
    return st;
}

HAL_StatusTypeDef fram_write(uint16_t addr, const uint8_t* buf, size_t len)
{
    if (buf == NULL || len == 0u)
    {
        return HAL_OK; /* nothing to do */
    }

    if (addr > FRAM_MAX_ADDR)
    {
        return HAL_ERROR;
    }

    /* Clamp length so we don't run past FRAM end */
    size_t maxLen = (size_t)(FRAM_BYTES - addr);
    if (len > maxLen)
    {
        len = maxLen;
    }

    HAL_StatusTypeDef st = fram_wren();
    if (st != HAL_OK) return st;

    uint8_t hdr[3] = { 0x02u, (uint8_t)(addr >> 8), (uint8_t)addr }; /* WRITE opcode */
    FRAM_CS_LOW();
    st = HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
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
    if (st == HAL_OK && out)
    {
        *out = ((uint16_t)be[0] << 8) | be[1];
    }
    return st;
}

/* returns 1 if the two address words look uninitialized (0xFFFF/0xFFFF or out of range) */
void fram_init_state(void)
{
    g_logging_enabled = 0u;
    fram_reload_address_words();
}

/* =========================
 * Assignment commands
 * ========================= */
bool fram_logging_enabled(void)
{
    return g_logging_enabled != 0u;
}

HAL_StatusTypeDef fram_get_first_last(uint16_t* first, uint16_t* last)
{
    if (first) *first = g_first_addr;
    if (last)  *last  = g_last_addr;
    return HAL_OK;
}

void fram_cmd_start(void)
{
    fram_reload_address_words();
    g_logging_enabled = 1u;
}

void fram_cmd_stop(void)
{
    g_logging_enabled = 0u;
}

void fram_cmd_clear(void)
{
    g_logging_enabled = 0u;

    uint8_t blank[32];
    memset(blank, 0xFF, sizeof(blank));
    for (uint16_t addr = 0u; addr < FRAM_BYTES; addr = (uint16_t)(addr + (uint16_t)sizeof(blank)))
    {
        size_t remaining = (size_t)(FRAM_BYTES - addr);
        size_t chunk = remaining < sizeof(blank) ? remaining : sizeof(blank);
        (void)fram_write(addr, blank, chunk);
    }

    g_first_addr = FRAM_DATA_START;
    g_last_addr  = FRAM_DATA_START - 2u;
    (void)fram_write_u16(FRAM_WORD_FIRST_ADDR, g_first_addr);
    (void)fram_write_u16(FRAM_WORD_LAST_ADDR,  g_last_addr);
}

/* Pass the raw 16-bit TMP102 reading here once per second */
HAL_StatusTypeDef fram_log_sample(uint16_t tmp102_raw)
{
    if (!g_logging_enabled)
    {
        return HAL_OK;
    }

    uint16_t next = (uint16_t)(g_last_addr + 2u);
    if (next > (FRAM_MAX_ADDR - 1u))
    {
        g_logging_enabled = 0u;
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = fram_write_u16(next, tmp102_raw);
    if (st != HAL_OK)
    {
        g_logging_enabled = 0u;
        return st;
    }

    if (g_last_addr < g_first_addr)
    {
        g_first_addr = next;
        (void)fram_write_u16(FRAM_WORD_FIRST_ADDR, g_first_addr);
    }

    g_last_addr = next;
    (void)fram_write_u16(FRAM_WORD_LAST_ADDR, g_last_addr);
    return HAL_OK;
}
