#include "BLDC_BEMF.h"

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)

static volatile uint8_t s_adc_ready;

uint32_t BLDC_BEMF_NowUs(void)
{
    return TIM5->CNT;
}

void BLDC_BEMF_DisableTrigger(void)
{
    if (s_adc_ready != 0U)
    {
        ADC1->CR2 &= ~ADC_CR2_JEXTEN;
    }
}

void BLDC_BEMF_Init(void)
{
    RCC_ClkInitTypeDef clock;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clock, &latency);
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clock.APB1CLKDivider != RCC_HCLK_DIV1) timer_clock *= 2UL;
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0U;
    TIM5->DIER = 0U;
    TIM5->PSC = timer_clock / 1000000UL - 1UL;
    TIM5->ARR = 0xFFFFFFFFUL;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CNT = 0U;
    TIM5->SR = 0U;
    TIM5->CR1 = TIM_CR1_CEN; /* Free-running: never reset at commutation. */

    s_adc_ready = 0U;
#if BLDC_BEMF_HARDWARE_READY
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    pins.Mode = GPIO_MODE_ANALOG;
    pins.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &pins);
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    ADC1->CR2 = 0U;
    ADC2->CR2 = 0U;
    /* RM0090: dual injected simultaneous = MULTI 00101; APB2/4=21MHz.
     * Injected JEXTSEL=0000 is TIM1_CC4 (regular ADC cannot use this trigger). */
    ADC->CCR = ADC_CCR_MULTI_2 | ADC_CCR_MULTI_0 | ADC_CCR_ADCPRE_0;
    ADC1->CR1 = ADC_CR1_JEOCIE;
    ADC2->CR1 = 0U;
    /* 15-cycle acquisition + 12-cycle conversion = 1.286us at 21MHz.
     * Channel 5/6/7 source impedance must be about 3.2k or lower. */
    ADC1->SMPR2 = (1UL << 15U) | (1UL << 18U) | (1UL << 21U);
    ADC2->SMPR2 = ADC1->SMPR2;
    ADC1->JSQR = 5UL << 15U;
    ADC2->JSQR = 7UL << 15U;
    ADC1->SR = 0U;
    ADC2->SR = 0U;
    ADC2->CR2 = ADC_CR2_ADON;
    ADC1->CR2 = ADC_CR2_ADON;
    /* Allow >3us ADC stabilization before an injected trigger. */
    HAL_Delay(1U); /* Initialization only; no waits in motor/ADC interrupts. */
    TIM1->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M);
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM1->CCR4 = 1U;
    HAL_NVIC_SetPriority(ADC_IRQn, 1U, 0U); /* TIM2 remains priority 0. */
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    s_adc_ready = 1U;
#endif
}

void BLDC_BEMF_SetSamplePoint(uint16_t duty)
{
    /* Sample during high-MOS ON, after the switching edge has settled.
     * At 6% duty the trigger is 1.5us after PWM start; acquisition ends
     * at about 2.2us, before the 3us pulse ends. CCR4 follows CCR1..3 preload. */
    TIM1->CCR4 = duty > 2U ? duty / 2U : 1U;
}

void BLDC_BEMF_SelectPhases(BLDC_Phase floating_phase, BLDC_Phase high_phase)
{
    BLDC_BEMF_DisableTrigger();
    if (s_adc_ready == 0U || floating_phase > BLDC_PHASE_C || high_phase > BLDC_PHASE_C)
        return;
    /* A=IN7, B=IN6, C=IN5. With JL=0, the single injected rank is JSQ4.
     * Previous conversion has finished during the >=5us all-off interval. */
    ADC1->JSQR = (7UL - (uint32_t)floating_phase) << 15U;
    ADC2->JSQR = (7UL - (uint32_t)high_phase) << 15U;
    ADC1->SR = 0U;
    ADC2->SR = 0U;
    ADC1->CR2 |= ADC_CR2_JEXTEN_0;
}

void BLDC_BEMF_ADC_IRQHandler(void)
{
    if ((ADC1->SR & ADC_SR_JEOC) == 0U) return;
    uint32_t adc2_done = ADC2->SR & ADC_SR_JEOC;
    uint16_t floating_adc = (uint16_t)ADC1->JDR1;
    uint16_t high_adc = (uint16_t)ADC2->JDR1;
    ADC1->SR &= ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR &= ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    if (s_adc_ready != 0U && adc2_done != 0U)
    {
        /* Approximate acquisition timestamp rather than IRQ service time. */
        BLDC_BEMF_Sample(floating_adc, high_adc, BLDC_BEMF_NowUs() - 1UL);
    }
}

#endif /* BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF */
