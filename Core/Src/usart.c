#include "main.h"
#include "usart.h"
#include "ringbuffer.h"
#include <stdbool.h>
#include <stdint.h>

UART_HandleTypeDef huart2;

/* --- Low-level helpers --------------------------------------------------- */
static inline void uart2_clear_rx_errors(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        (void)USART2->RDR; /* read clears ORE */
        USART2->ICR = (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF);
    }
}

/* Optional: direct TX putchar used by retarget.c */
void USR_USART2_WriteByte(uint8_t b)
{
#ifdef USART_ISR_TXE_TXFNF
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) { }
#else
    while (!(USART2->ISR & USART_ISR_TXE)) { }
#endif
    USART2->TDR = b;
}

/* --- Init ----------------------------------------------------------------- */
void USR_USART2_UART_Init(uint32_t baudrate)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2=TX, PA3=RX (AF1) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = baudrate ? baudrate : 115200u;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
#ifdef UART_ONE_BIT_SAMPLE_DISABLE
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    /* Enable RXNE/RXFNE interrupt so we never miss bytes */
#ifdef UART_IT_RXNE_RXFNE
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE_RXFNE);
#else
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
#endif

    /* NVIC */
#ifdef USART2_LPUART2_IRQn
    HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
#else
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif

    uart2_clear_rx_errors();
}

/* --- IRQ handler: drain RX FIFO fast and push to ring -------------------- */
static inline void usart2_rx_drain(void)
{
    for (;;)
    {
#ifdef USART_ISR_RXNE_RXFNE
        if (!(USART2->ISR & USART_ISR_RXNE_RXFNE)) break;
#else
        if (!(USART2->ISR & USART_ISR_RXNE)) break;
#endif
        uint8_t b = (uint8_t)USART2->RDR;
        ring_add((char)b);              /* push immediately; ring handles CR/LF/backspace */
    }
}

#ifdef USART2_LPUART2_IRQn
void USART2_LPUART2_IRQHandler(void)
#else
void USART2_IRQHandler(void)
#endif
{
    /* RX data ready? Drain all */
    usart2_rx_drain();

    /* Errors? Clear and keep going */
    uart2_clear_rx_errors();

    /* Let HAL clear any other flags it expects */
    HAL_UART_IRQHandler(&huart2);
}

/* Polled read still available for compatibility (rare) */
bool USR_USART2_TryRead(uint8_t *ch)
{
    if (!ch) return false;
    uart2_clear_rx_errors();
#ifdef USART_ISR_RXNE_RXFNE
    if (USART2->ISR & USART_ISR_RXNE_RXFNE) {
        *ch = (uint8_t)USART2->RDR;
        return true;
    }
#else
    if (USART2->ISR & USART_ISR_RXNE) {
        *ch = (uint8_t)USART2->RDR;
        return true;
    }
#endif
    return false;
}
