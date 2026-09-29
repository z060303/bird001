#ifndef BLDC_BEMF_H
#define BLDC_BEMF_H
#include "BLDC.h"

/* This BSP owns ADC1+ADC2, TIM1_CC4 and the otherwise unused TIM5.
 * Register access is intentional: this project has no ADC HAL module. */
void BLDC_BEMF_Init(void);
void BLDC_BEMF_DisableTrigger(void);
void BLDC_BEMF_SelectPhases(BLDC_Phase floating_phase, BLDC_Phase high_phase);
void BLDC_BEMF_SetSamplePoint(uint16_t duty);
void BLDC_BEMF_ADC_IRQHandler(void);
uint32_t BLDC_BEMF_NowUs(void);
#endif
