#include "stm32g0xx.h"
#include <stdint.h>

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
