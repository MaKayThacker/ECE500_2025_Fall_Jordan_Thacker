#include "usr_usart.h"

/* PA2=USART2_TX (AF1), PA3=USART2_RX (AF1) */
static void usart2_gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    /* Alternate function mode */
    GPIOA->MODER &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->MODER |=  ((0x2u<<(2*2)) | (0x2u<<(3*2)));

    /* AF1 on PA2/PA3 */
    GPIOA->AFR[0] &= ~((0xFu<<(4*2)) | (0xFu<<(4*3)));
    GPIOA->AFR[0] |=  ((0x1u<<(4*2)) | (0x1u<<(4*3)));

    /* Push-pull, no-pull, medium speed (optional but tidy) */
    GPIOA->OTYPER  &= ~((1u<<2) | (1u<<3));
    GPIOA->PUPDR   &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->OSPEEDR |=  ((0x1u<<(2*2)) | (0x1u<<(3*2)));
}

/* Return the current USART2 kernel clock, assuming default PCLK unless overridden */
static uint32_t usart2_kernel_clk_hz(void)
{
    /* USART2SEL bits choose: 00=PCLK, 01=SYSCLK, 10=HSI16, 11=LSE */
    uint32_t sel = (RCC->CCIPR >> RCC_CCIPR_USART2SEL_Pos) & 0x3u;

    switch (sel) {
        case 0x0: /* PCLK (APB1) == HCLK when PPRE=/1 */ return SystemCoreClock;
        case 0x1: /* SYSCLK */                           return SystemCoreClock;
        case 0x2: /* HSI16 */                            return 16000000u;
        case 0x3: /* LSE (rare here) */                  return 32768u;
        default:                                        return 32768u;
    }
}

static void usart2_select_pclk(void)
{
    /* Explicitly source USART2 from PCLK to keep BRR math simple */
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL_Msk;  /* 00 = PCLK */
}

void USR2_USART2_Init(uint32_t baud)
{
    usart2_gpio_init();

    RCC->APBENR1 |= RCC_APBENR1_USART2EN;

    /* Ensure USART2 uses PCLK unless you have a reason otherwise */
    usart2_select_pclk();

    USART2->CR1 &= ~USART_CR1_UE;           /* disable to configure */

    /* 16x oversampling (OVER8=0), 8-N-1, no flow control */
    USART2->CR1  = USART_CR1_TE | USART_CR1_RE; /* keep OVER8=0 */
    USART2->CR2  = 0;
    USART2->CR3  = 0;

    /* Compute BRR from the ACTUAL kernel clock */
    uint32_t fck = usart2_kernel_clk_hz();
    USART2->BRR  = (fck + (baud/2u)) / baud;   /* rounded */

    USART2->CR1 |= USART_CR1_UE;               /* enable */
    while ((USART2->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) !=
           (USART_ISR_TEACK | USART_ISR_REACK)) { }
}

void usr2_usart2_write_char(char c)
{
    while ((USART2->ISR & USART_ISR_TXE_TXFNF) == 0) { }
    USART2->TDR = (uint8_t)c;
    while ((USART2->ISR & USART_ISR_TC) == 0) { }
}

/* >>> Missing in your file — this is what main.c calls <<< */
void usr2_usart2_write(const char *s)
{
    if (!s) return;
    while (*s) {
        usr2_usart2_write_char(*s++);
    }
}
