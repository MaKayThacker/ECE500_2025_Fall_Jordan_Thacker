// usart.c
#include "usart.h"
#include "stm32g0xx.h"
#include <stdint.h>


#ifndef USART2_BAUD_DEFAULT
#define USART2_BAUD_DEFAULT 115200u
#endif

// GPIO pins on NUCLEO-G071RB VCP
#define USART2_TX_PORT      GPIOA
#define USART2_TX_PIN       2u     // PA2
#define USART2_RX_PORT      GPIOA
#define USART2_RX_PIN       3u     // PA3
#define USART2_TX_AF        1u     // AF1 = USART2_TX
#define USART2_RX_AF        1u     // AF1 = USART2_RX

static inline void gpio_cfg_alt(GPIO_TypeDef *port, uint32_t pin, uint32_t af) {
    // MODER: 10 = Alternate
    port->MODER &= ~(0x3u << (pin * 2));
    port->MODER |=  (0x2u << (pin * 2));
    // OTYPER: 0 = Push-pull
    port->OTYPER &= ~(1u << pin);
    // OSPEEDR: 10 = High (good signal edges for UART)
    port->OSPEEDR &= ~(0x3u << (pin * 2));
    port->OSPEEDR |=  (0x2u << (pin * 2));
    // PUPDR: RX pull-up, TX no-pull (both okay as no-pull too)
    if (port == USART2_RX_PORT && pin == USART2_RX_PIN) {
        port->PUPDR &= ~(0x3u << (pin * 2));
        port->PUPDR |=  (0x1u << (pin * 2)); // 01 = pull-up
    } else {
        port->PUPDR &= ~(0x3u << (pin * 2)); // 00 = no-pull
    }
    // AFRL/AFRH
    volatile uint32_t *afr = (pin < 8) ? &port->AFR[0] : &port->AFR[1];
    uint32_t shift = (pin & 7u) * 4u;
    *afr &= ~(0xFu << shift);
    *afr |=  ((af & 0xFu) << shift);
}

static inline uint32_t usart2_get_pclk(void) {
    // We select PCLK as USART2 kernel clock below, so BRR uses APB clock.
    // On G071RB default Cube configs often run at 64 MHz; adapt if needed.
    // Use SystemCoreClock if APB prescaler == 1; otherwise compute from RCC.
    // For G0, APB prescaler commonly equals AHB prescaler; keep simple:
    return SystemCoreClock; // adjust if your APB prescaler != 1
}

void USR_USART2_UART_Init(uint32_t baud)
{
    if (baud == 0u) baud = USART2_BAUD_DEFAULT;

    // --- Enable clocks ---
    RCC->IOPENR   |= RCC_IOPENR_GPIOAEN;        // GPIOA clock
    RCC->APBENR1  |= RCC_APBENR1_USART2EN;      // USART2 clock

    // Select USART2 kernel clock = PCLK (00)
    // CCIPR USART2SEL bits [3:2] for USART2 (consult header – G0 family)
    RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);       // 00: PCLK selected

    // --- Configure PA2/PA3 as AF1 (USART2) ---
    gpio_cfg_alt(USART2_TX_PORT, USART2_TX_PIN, USART2_TX_AF);
    gpio_cfg_alt(USART2_RX_PORT, USART2_RX_PIN, USART2_RX_AF);

    // --- Disable USART before config ---
    USART2->CR1 &= ~USART_CR1_UE;

    // Word length: 8N1
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE);
    USART2->CR2 &= ~(USART_CR2_STOP);           // 00 = 1 stop bit
    USART2->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE); // no HW flow

    // Oversampling by 16 (default); compute BRR
    uint32_t pclk = usart2_get_pclk();
    USART2->BRR = (pclk + (baud/2u)) / baud;    // rounded

    // Enable FIFOs (optional on G0; default FIFO mode enabled in many libs)
    // Leave default; we poll TXFNF in ISR.

    // Enable transmitter and receiver
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // Enable USART
    USART2->CR1 |= USART_CR1_UE;

    // (Optional) Wait for TEACK/REACK (acknowledge bits) if desired
    // while(((USART2->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) !=
    //       (USART_ISR_TEACK | USART_ISR_REACK))) {;}
}
