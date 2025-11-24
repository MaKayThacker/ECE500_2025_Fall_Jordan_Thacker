#ifndef FRAM_SPI_H
#define FRAM_SPI_H

#include "main.h"
#include <stdint.h>
#include <stddef.h>

#include <stdbool.h>

/* Session/command handlers */
void fram_init_state(void);
void fram_cmd_start(void);
void fram_cmd_stop(void);
void fram_cmd_clear(void);
HAL_StatusTypeDef fram_log_sample(uint16_t tmp102_raw);
bool fram_logging_enabled(void);
HAL_StatusTypeDef fram_get_first_last(uint16_t* first, uint16_t* last);

HAL_StatusTypeDef fram_iterate_samples(void (*cb)(uint16_t index, uint16_t raw));


/* ======= EDIT TO MATCH YOUR BOARD WIRING ======= */
/* Chip Select (FRAM /CS) */
#ifndef FRAM_CS_GPIO_Port
#define FRAM_CS_GPIO_Port   GPIOB
#endif
#ifndef FRAM_CS_Pin
#define FRAM_CS_Pin         GPIO_PIN_0
#endif


/* WP pin (active low) */
#ifndef FRAM_WP_GPIO_Port
#define FRAM_WP_GPIO_Port   GPIOC
#endif
#ifndef FRAM_WP_Pin
#define FRAM_WP_Pin         GPIO_PIN_7
#endif
/* SPI1 pins (default: PA5=SCK, PA6=MISO, PA7=MOSI, AF0 on STM32G0) */
#ifndef FRAM_SPI_GPIO_Port
#define FRAM_SPI_GPIO_Port  GPIOA /* MOSI/MISO live on A; SCK on PB3 handled separately */
#endif
#ifndef FRAM_SPI_SCK_Pin
#define FRAM_SPI_SCK_Pin    GPIO_PIN_3 /* on GPIOB */
#endif
#ifndef FRAM_SPI_MISO_Pin
#define FRAM_SPI_MISO_Pin   GPIO_PIN_6 /* PA6 */
#endif
#ifndef FRAM_SPI_MOSI_Pin
#define FRAM_SPI_MOSI_Pin   GPIO_PIN_7 /* PA7 */
#endif
#ifndef FRAM_SPI_AF
#define FRAM_SPI_AF         GPIO_AF0_SPI1
#endif
/* =============================================== */

/* Global SPI handle (defined in main.c) */
extern SPI_HandleTypeDef hspi1;

/* CS helpers (active low) */
#define FRAM_CS_LOW()   HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET)
#define FRAM_CS_HIGH()  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET)

/* FRAM API */
uint8_t  fram_rdsr(void);
HAL_StatusTypeDef fram_read (uint16_t addr, uint8_t* buf, size_t len);
HAL_StatusTypeDef fram_write(uint16_t addr, const uint8_t* buf, size_t len);

#endif /* FRAM_SPI_H */
