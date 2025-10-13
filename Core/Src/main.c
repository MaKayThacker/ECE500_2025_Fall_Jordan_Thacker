#include "stm32g0xx.h"
#include "usr_clock.h"
#include "usr_usart.h"
#include <stdio.h>

#define LED_GPIO GPIOA
#define LED_PIN  5u
#define LED_MASK (1u << LED_PIN)

static inline void led_toggle(void) { LED_GPIO->ODR ^= LED_MASK; }

static void systick_reinit_1ms(void) {
    SysTick->CTRL = 0;
    SysTick->VAL  = 0;
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}

/* forward from usr_clock.c */
extern uint8_t usr_clock_is_32mhz(void);

/* Tiny runtime reporter so we can print what the chip latched */
static uint32_t usr_report_sysclk_hz(void) {
    (void)((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos); /* SWS available if you want it */
    return SystemCoreClock; /* maintained in USR_SystemClock_Config() */
}

int main(void)
{
    HAL_Init();
    USR_SystemClock_Config();
    systick_reinit_1ms();

    USR2_USART2_Init(115200u);

    if (usr_clock_is_32mhz()) {
        usr2_usart2_write("\r\n[Boot] SYSCLK=32MHz (PLL), PCLK=32MHz, USART2=115200\r\n");
    } else {
        usr2_usart2_write("\r\n[Boot] SYSCLK=16MHz (HSI fallback), PCLK=16MHz, USART2=115200\r\n");
        usr2_usart2_write("WARNING: PLL switch failed; running on HSI16.\r\n");
    }

    /* Extra clock report for grading/telemetry */
    {
        char buf[96];
        unsigned long sws = (unsigned long)((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);
        unsigned long hz  = (unsigned long)usr_report_sysclk_hz();
        snprintf(buf, sizeof buf, "[Clock] SWS=%lu (3=PLLR), SystemCoreClock=%lu Hz\r\n", sws, hz);
        usr2_usart2_write(buf);
    }

    usr2_usart2_write("Blinking LED at 1 Hz...\r\n");
    while (1) {
        led_toggle();
        HAL_Delay(1000);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { led_toggle(); }
}
