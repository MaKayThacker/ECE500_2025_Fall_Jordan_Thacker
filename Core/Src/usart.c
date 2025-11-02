#include "main.h"
#include "usart.h"
#include "ring_retarget.h"
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

/* Exported status so main() can print the actual clock in use */
static uint8_t g_clk_32mhz_ok = 0u;
uint8_t usr_clock_is_32mhz(void) { return g_clk_32mhz_ok; }

/* Custom 32 MHz clock setup from HSI16 via PLL (R output /4).
   Also initializes LD2 (PA5) via registers, as required. */
void USR_SystemClock_Config(void)
{
    /* ----- LED (PA5) via registers so we can see life right away ----- */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER   &= ~(0x3u << (5u * 2u));
    GPIOA->MODER   |=  (0x1u << (5u * 2u));   /* output */
    GPIOA->OTYPER  &= ~(1u << 5u);            /* push-pull */
    GPIOA->PUPDR   &= ~(0x3u << (5u * 2u));   /* no pull */
    GPIOA->OSPEEDR |=  (0x1u << (5u * 2u));   /* medium */
    GPIOA->BSRR = (1u << 5u);                 /* LED ON */

    /* ----- Enable HSI16, ensure undivided ----- */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) { }
#if defined(RCC_CR_HSIDIV)
    RCC->CR &= ~RCC_CR_HSIDIV;                /* HSIDIV = /1 */
#endif

    /* Optional: default trim 64 for HSI */
#if defined(RCC_ICSCR_HSITRIM_Pos)
    RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_HSITRIM) | (64u << RCC_ICSCR_HSITRIM_Pos);
#endif

    /* ----- Raise Vcore performance: VOS Range-1 allows >16 MHz ----- */
    RCC->APBENR1 |= RCC_APBENR1_PWREN;        /* PWR clock */
#if defined(PWR_CR1_VOS_Pos)
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | (0x1u << PWR_CR1_VOS_Pos); /* Range 1 */
    while (PWR->SR2 & PWR_SR2_VOSF) { }       /* wait until applied */
#endif

    /* ----- Make sure PLL is off before configuring ----- */
    if (RCC->CR & RCC_CR_PLLON) {
        RCC->CR &= ~RCC_CR_PLLON;
        while (RCC->CR & RCC_CR_PLLRDY) { }
    }

    /* ----- Prescalers: AHB=/1, APB=/1; keep SYSCLK=HSI for now ----- */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE | RCC_CFGR_SW)) |
                (0u << RCC_CFGR_HPRE_Pos) |
                (0u << RCC_CFGR_PPRE_Pos) |
                (0b01u << RCC_CFGR_SW_Pos);   /* 01 = HSI */

    /* ----- Flash wait state: 1 WS for 32 MHz + enable prefetch ----- */
#if defined(FLASH_ACR_LATENCY_Pos)
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | (0x1u << FLASH_ACR_LATENCY_Pos);
#endif
#if defined(FLASH_ACR_PRFTEN)
    FLASH->ACR |= FLASH_ACR_PRFTEN;           /* prefetch recommended at higher freq */
#endif

    /* ----- PLL: HSI16 * N=8 / R=4 => 32 MHz ----- */
    uint32_t pll = 0;
#if defined(RCC_PLLCFGR_PLLSRC_Pos)
    pll |= (0b10u << RCC_PLLCFGR_PLLSRC_Pos); /* HSI16 */
#endif
#if defined(RCC_PLLCFGR_PLLM_Pos)
    pll |= (0u    << RCC_PLLCFGR_PLLM_Pos);   /* M=1 */
#endif
#if defined(RCC_PLLCFGR_PLLN_Pos)
    pll |= (8u    << RCC_PLLCFGR_PLLN_Pos);   /* N=8 */
#endif
#if defined(RCC_PLLCFGR_PLLR_Pos)
    pll |= (1u    << RCC_PLLCFGR_PLLR_Pos);   /* R=/4 (01) */
#endif
#if defined(RCC_PLLCFGR_PLLREN)
    pll |= RCC_PLLCFGR_PLLREN;                /* enable R output */
#endif
#if defined(RCC_PLLCFGR_PLLPEN)
    pll &= ~RCC_PLLCFGR_PLLPEN;
#endif
#if defined(RCC_PLLCFGR_PLLQEN)
    pll &= ~RCC_PLLCFGR_PLLQEN;
#endif
    RCC->PLLCFGR = pll;

    /* ----- Enable PLL and wait ready ----- */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) { }

    /* ----- Switch SYSCLK to PLLR ----- */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (0b11u << RCC_CFGR_SW_Pos); /* 11 = PLLR */
    uint32_t guard = 1000000u;
    while ((((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 0b11u) && guard--) { }

    if (guard == 0u) {
        /* Switch failed: stay on HSI16 so the rest of the app runs */
#ifdef SystemCoreClock
        SystemCoreClock = 16000000u;
#endif
        g_clk_32mhz_ok = 0u;
        return;
    }

    /* ----- Success: on PLL @ 32 MHz ----- */
#ifdef SystemCoreClock
    SystemCoreClock = 32000000u;
#endif
    g_clk_32mhz_ok = 1u;

    /* Ensure USART2 kernel clock uses PCLK (00) for predictable BRR math */
#if defined(RCC_CCIPR_USART2SEL_Msk)
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL_Msk;
#endif
}
