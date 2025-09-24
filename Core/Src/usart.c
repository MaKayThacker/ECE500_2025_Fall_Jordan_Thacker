#include "usart.h"
#include <stdbool.h>

/* --- Private state ------------------------------------------------------- */
UART_HandleTypeDef huart2;

/* Staging byte for interrupt-driven receive */
static volatile uint8_t rx_byte;
static volatile uint8_t rx_ready = 0;

/* --- Init ---------------------------------------------------------------- */
void USR_USART2_UART_Init(uint32_t baud) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA2 (TX), PA3 (RX) as AF1 for USART2 */
    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_cfg.Mode      = GPIO_MODE_AF_PP;
    gpio_cfg.Pull      = GPIO_NOPULL;
    gpio_cfg.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_cfg.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &gpio_cfg);

    /* Basic UART parameters */
    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = baud;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1) {}  // Fatal error
    }

    /* Kick off first interrupt-driven receive */
    rx_ready = 0;
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);

    /* Enable IRQ */
#ifdef USART2_LPUART2_IRQn
    HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
#else
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
}

/* --- Non-blocking read API ----------------------------------------------- */
bool USR_USART2_TryRead(uint8_t *out_byte) {
    if (!rx_ready) return false;
    *out_byte = rx_byte;
    rx_ready = 0;
    return true;
}

/* --- HAL callbacks ------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        rx_ready = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1); // re-arm
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1); // recover
    }
}

/* --- IRQ handler --------------------------------------------------------- */
#ifdef USART2_LPUART2_IRQn
void USART2_LPUART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
#else
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
#endif
