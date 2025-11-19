#include "main.h"
#include <stdio.h>
#include <stdint.h>

/*
 * ADC_DMA.c - Mod 9 ADC + DMA support
 *
 * - Configures ADC1 to sample PC0 (ADC1_IN6) at 8-bit resolution.
 * - Uses DMA1 Channel 1 (via DMAMUX1 Channel 0) to transfer 10 samples
 *   from ADC1->DR into g_adc_buf[].
 * - EXTI4_15 (blue user button on PC13) starts a 10-sample burst.
 * - DMA1_Channel1_IRQHandler averages the 10 samples and prints the voltage.
 */

#define ADC_SAMPLE_COUNT  (10u)

/* Global buffer used as DMA destination. */
volatile uint16_t g_adc_buf[ADC_SAMPLE_COUNT];

/* Helper macro from pseudo code: fully disable ADC before configuring. */
#define __USR_ADC_DISABLE(__HANDLE__)                                                \
    do {                                                                             \
        (__HANDLE__)->CR &= ~(ADC_CR_ADEN    | ADC_CR_ADDIS   | ADC_CR_ADSTP   |     \
                              ADC_CR_ADSTART | ADC_CR_ADCAL   | ADC_CR_ADVREGEN);    \
    } while (0)

/* Forward declarations (used from main.c) */
void USR_ADC1_Init(void);
void USR_DMA_Init(void);

/* ------------------------------------------------------------------------- */
/* ADC configuration                                                         */
/* ------------------------------------------------------------------------- */

void USR_ADC1_Init(void)
{
    /* === Enable clocks === */

    /* Enable ADC1 clock (APBENR2, see RM0444) */
    RCC->APBENR2 |= RCC_APBENR2_ADCEN;
    HAL_Delay(1);

    /* Enable GPIOC clock for PC0 analog input */
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    HAL_Delay(1);

    /* PC0 analog mode: MODER0 = 11b, no pull-ups / pull-downs */
    GPIOC->MODER |= (3u << (0u * 2u));
    GPIOC->PUPDR &= ~(3u << (0u * 2u));

    /* === Disable ADC and enable internal regulator === */

    __USR_ADC_DISABLE(ADC1);

    /* Enable the ADC voltage regulator.  On the G0 this is a single bit
       macro, but we also handle the mask/pos style just in case. */
#if defined(ADC_CR_ADVREGEN_Msk)
    ADC1->CR &= ~ADC_CR_ADVREGEN_Msk;
    ADC1->CR |= (0x1u << ADC_CR_ADVREGEN_Pos); /* ADVREGEN = 01b */
#else
    ADC1->CR |= ADC_CR_ADVREGEN;
#endif

    HAL_Delay(1);

    /* === Calibration === */

    /* Disable DMAEN before calibration (RM0444) */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    /* Start calibration (ADCAL = 1) */
    ADC1->CR |= ADC_CR_ADCAL;

    /* Wait for calibration to finish (ADCAL cleared by HW) */
    while (ADC1->CR & ADC_CR_ADCAL)
    {
        /* busy-wait */
    }

    /* Read calibration factor for debug. */
    uint32_t cal = ADC1->CALFACT;
    printf("ADC calibration factor: %lu\r\n", (unsigned long)cal);

    /* === Resolution and clock === */

#ifdef ADC_CFGR1_RES
    /* 8-bit resolution: RES[1:0] = 10b (see RM0444, ADC_CFGR1.RES) */
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;
    ADC1->CFGR1 |= (0x2u << ADC_CFGR1_RES_Pos);
#endif

#ifdef ADC_CFGR2_CKMODE
    /* Clock source: PCLK/4 (CKMODE = 10b in CFGR2, RM0444) */
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;
    ADC1->CFGR2 |= (0x2u << ADC_CFGR2_CKMODE_Pos); /* PCLK/4 */
#endif

    /* === Channel selection === */

#ifdef ADC_CFGR1_CHSELRMOD
    /* Single-ended channel selection mode (CHSELRMOD = 0) */
    ADC1->CFGR1 &= ~ADC_CFGR1_CHSELRMOD;
#endif

    /* Select channel 6 (PC0 = ADC1_IN6 on Nucleo-G071). */
    ADC1->CHSELR = ADC_CHSELR_CHSEL6;

#ifdef ADC_ISR_CCRDY
    /* Wait for CCRDY flag then clear it (channel config ready) */
    while (!(ADC1->ISR & ADC_ISR_CCRDY))
    {
    }
    ADC1->ISR = ADC_ISR_CCRDY;
#endif

    /* === Sample time === */

#ifdef ADC_SMPR_SMP1
    /* SMP1 = 0b010 (7.5 ADC cycles); then map channel 6 to SMP1. */
    ADC1->SMPR &= ~ADC_SMPR_SMP1;                      /* clear SMP1[2:0] */
    ADC1->SMPR |= (0x2u << ADC_SMPR_SMP1_Pos);         /* SMP1 = 0b010    */

    /* Bit 6 of SMPR selects SMP1 (0) vs SMP2 (1) for channel 6; set to 0. */
    ADC1->SMPR &= ~(1u << 6);
#endif

    /* === Conversion mode, trigger, DMA === */

#ifdef ADC_CFGR1_CONT
    /* Continuous conversion mode */
    ADC1->CFGR1 |= ADC_CFGR1_CONT;
#endif

#ifdef ADC_CFGR1_EXTEN
    /* Software trigger only: EXTEN = 00b */
    ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
#endif

#ifdef ADC_CFGR1_DMACFG
    /* Non-circular DMA (DMACFG = 0) */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMACFG;
#endif

#ifdef ADC_CFGR1_DMAEN
    /* Enable DMA requests from ADC */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
#endif

    /* === Enable ADC === */

#ifdef ADC_ISR_ADRDY
    /* Clear ADRDY then set ADEN; wait for ADRDY again. */
    ADC1->ISR = ADC_ISR_ADRDY;
#endif

    ADC1->CR |= ADC_CR_ADEN;

#ifdef ADC_ISR_ADRDY
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    {
    }
#endif

    printf("ADC1 initialized (PC0 / IN6, 8-bit, DMA enabled).\r\n");
}

/* ------------------------------------------------------------------------- */
/* DMA configuration                                                         */
/* ------------------------------------------------------------------------- */

void USR_DMA_Init(void)
{
    /* Enable DMA1 clock (AHBENR) */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* Disable DMA1 Channel 1 before configuration */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    /* Peripheral address: ADC1 data register */
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;

    /* Memory address: our sample buffer */
    DMA1_Channel1->CMAR = (uint32_t)g_adc_buf;

    /* Number of transfers for each burst */
    DMA1_Channel1->CNDTR = ADC_SAMPLE_COUNT;

    /* --- Channel configuration (RM0444, DMA section) --- */

    /* Memory-to-memory disabled */
    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM;

    /* Priority: high (PL = 10b) */
    DMA1_Channel1->CCR &= ~DMA_CCR_PL;
    DMA1_Channel1->CCR |= DMA_CCR_PL_1;

    /* Memory size = 16-bit (MSIZE = 01b) */
    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;

    /* Peripheral size = 16-bit (PSIZE = 01b) */
    DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;

    /* Memory increment enabled, peripheral increment disabled */
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    DMA1_Channel1->CCR &= ~DMA_CCR_PINC;

    /* Circular mode disabled (CIRC = 0): we reload CNDTR in EXTI ISR */
    DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;

    /* Direction: read from peripheral (DIR = 0) */
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;

    /* Disable half-transfer & transfer-error interrupts, enable TC. */
    DMA1_Channel1->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;

    /* Clear any pending flags on channel 1 */
    DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;

    /* DMAMUX1 Channel 0: connect ADC request to DMA1 Channel 1.
       Request ID value comes from RM0444 Table 52 (ADC). For the G071,
       ADC typically uses request ID 5. */
#if defined(DMAMUX1_Channel0)
# if defined(DMAMUX_CxCR_DMAREQ_ID_Msk)
    DMAMUX1_Channel0->CCR &= ~DMAMUX_CxCR_DMAREQ_ID_Msk;
    DMAMUX1_Channel0->CCR |= (5u << DMAMUX_CxCR_DMAREQ_ID_Pos);
# else
    DMAMUX1_Channel0->CCR = 5u;
# endif
#endif

    /* Enable DMA1 Channel 1 IRQ in NVIC; priority 1 is fine. */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Do not enable the channel here; EXTI ISR arms it per burst. */
    printf("DMA1 Channel 1 initialized for ADC1->g_adc_buf[%u].\r\n",
           (unsigned)ADC_SAMPLE_COUNT);
}

/* ------------------------------------------------------------------------- */
/* Interrupt handlers                                                        */
/* ------------------------------------------------------------------------- */

/* Blue user button on PC13 is on EXTI line 13, handled by EXTI4_15_IRQn.   */
/* When pressed, start a 10-sample ADC+DMA burst.                           */

void EXTI4_15_IRQHandler(void)
{
    /* Check and clear EXTI line 13 pending flag. */
#if defined(EXTI_RPR1_RPIF13)
    if (EXTI->RPR1 & EXTI_RPR1_RPIF13)
    {
        EXTI->RPR1 = EXTI_RPR1_RPIF13;
#else
    if (EXTI->RPR1 & (1u << 13))
    {
        EXTI->RPR1 = (1u << 13);
#endif
        /* Optional: make sure ADC is not already running. */
        if (!(ADC1->CR & ADC_CR_ADSTART))
        {
            /* Disable DMA channel 1 before modifying CNDTR. */
            DMA1_Channel1->CCR &= ~DMA_CCR_EN;

            /* Reload number of samples for this burst. */
            DMA1_Channel1->CNDTR = ADC_SAMPLE_COUNT;

            /* Re-enable DMA channel 1. */
            DMA1_Channel1->CCR |= DMA_CCR_EN;

            /* Start ADC conversions (continuous mode). */
            ADC1->CR |= ADC_CR_ADSTART;
        }
    }
}

/* DMA1 Channel1 IRQ: fired when 10 ADC samples have been transferred
   from ADC1->DR into g_adc_buf[]. */
void DMA1_Channel1_IRQHandler(void)
{
    /* Transmission complete for channel 1? */
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        /* Clear the TC/HT/TE flags for channel 1. */
        DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;

        /* Stop ADC conversions if still running. */
        if (ADC1->CR & ADC_CR_ADSTART)
        {
            ADC1->CR |= ADC_CR_ADSTP;
            while (ADC1->CR & ADC_CR_ADSTP)
            {
                /* wait for ADSTP to clear */
            }
        }

        /* Compute average of the 10 samples. ADC is configured for 8-bit
           resolution, stored in a 16-bit DR; mask to 0xFF if desired. */
        uint32_t sum = 0u;
        for (uint32_t i = 0; i < ADC_SAMPLE_COUNT; ++i)
        {
            sum += (uint32_t)(g_adc_buf[i] & 0x00FFu);
        }

        float avg_counts = (float)sum / (float)ADC_SAMPLE_COUNT;

        /* Convert to volts assuming Vref = 3.3 V and 8-bit range [0..255]. */
        float volts = (avg_counts / 255.0f) * 3.3f;

        printf("ADC DMA: avg = %.1f counts, V = %.3f V\r\n",
               avg_counts, volts);
    }
}
