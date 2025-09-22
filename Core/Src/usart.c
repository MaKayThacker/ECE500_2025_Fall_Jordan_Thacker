#include "usart.h"

UART_HandleTypeDef huart2;

static volatile uint8_t rx_byte;
static volatile uint8_t rx_has = 0;

void USR_USART2_UART_Init(uint32_t baud)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_2 | GPIO_PIN_3;           // PA2=TX, PA3=RX
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;                      // if line is noisy, try GPIO_PULLUP
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = baud;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { while (1) {} }

  rx_has = 0;
  // Arm the first byte reception
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);

  // G0 series: correct IRQ name is USART2_LPUART2_IRQn
#ifdef USART2_LPUART2_IRQn
  HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
#else
  // Fallback for other families / Cube projects
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
}

bool USR_USART2_TryRead(uint8_t *out_byte)
{
  if (!rx_has) return false;
  *out_byte = rx_byte;
  rx_has = 0;
  return true;
}

// Called by HAL when a byte completes
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    rx_has = 1;
    // Immediately arm the next byte so we don't miss anything
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);
  }
}

// Recover from UART errors (overrun, framing, noise) and re-arm RX
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    __HAL_UART_CLEAR_OREFLAG(huart);  // clear ORE if set
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);
  }
}

// Correct IRQ handler (G0)
#ifdef USART2_LPUART2_IRQn
void USART2_LPUART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
#else
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
#endif
