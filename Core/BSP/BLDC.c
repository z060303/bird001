#include "BLDC.h"

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)

#include "tim.h"
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
#include "BLDC_BEMF.h"
#endif

#include <stdio.h>

/*
 * Six-step BLDC driver for STM32F405 + TIM1 + TIM2 + EG2133.
 * The compile-time mode in BLDC.h decides whether ADC1/ADC2/TIM5 back-EMF
 * feedback is present in this image. The open-loop image retains the
 * 16:37 startup/ramp path and has no BEMF peripheral side effects.
 *
 * IMPORTANT HARDWARE MAPPING:
 *   A = TIM1_CH3  (PA10) / TIM1_CH3N (PB15)
 *   B = TIM1_CH2  (PA9)  / TIM1_CH2N (PB14)
 *   C = TIM1_CH1  (PA8)  / TIM1_CH1N (PB13)
 *
 * EG2133 input logic:
 *   HIN high -> high-side MOS ON
 *   LIN low  -> low-side MOS ON
 *   HIN=0, LIN=1 -> phase floating
 *
 * TIM1 strategy:
 *   High phase : CHx = PWM1, CHxN disabled, with CHxN off-state HIGH.
 *   Low phase  : CHx disabled, CHxN enabled, OCREF forced HIGH and
 *                CCxNP=1 -> CHxN becomes LOW -> EG2133 low MOS ON.
 *   Float      : CHx disabled, CHxN enabled, OCREF forced LOW and
 *                CCxNP=1 -> CHxN becomes HIGH -> EG2133 low MOS OFF.
 *
 * Therefore CHx and CHxN are NEVER intentionally enabled together.
 * EG2133 itself provides dead-time and interlock protection.
 */

typedef enum
{
    PHASE_MODE_FLOAT = 0,
    PHASE_MODE_HIGH_PWM,
    PHASE_MODE_LOW_ON
} BLDC_PhaseMode;

typedef struct
{
    BLDC_Phase high_phase;
    BLDC_Phase low_phase;
} BLDC_StepDefinition;

volatile BLDC_Handle motor =
{
    .duty = 0U,
    .max_duty = 0U,
    .duty_percent = 0U,
    .step = 0U,
    .commutation_period_us = BLDC_START_PERIOD_US,
    .dir = MOTOR_FORWARD,
    .state = BLDC_STATE_STOPPED
};

static volatile uint8_t s_blank_active = 0U;
static volatile uint8_t s_pending_step = 0U;
static volatile uint8_t s_debug_pending = 0U;
static volatile uint32_t s_debug_period_us = 0U;
static uint32_t s_alignment_elapsed_us = 0U;
static uint32_t s_ramp_start_ms = 0U;
static uint32_t s_ramp_from_rate_millihz = 0U;
static uint16_t s_ramp_from_duty = 0U;
static uint32_t s_target_period_us = BLDC_TARGET_PERIOD_US;
static uint16_t s_target_duty = 0U;
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
static uint32_t s_ramp_start_us = 0U;
static volatile uint8_t s_sensorless = 0U;
static volatile BLDC_Fault s_fault = BLDC_FAULT_NONE;
static volatile uint32_t s_measured_period_us;
static volatile uint32_t s_zc_total;
static volatile uint16_t s_last_float_adc;
static volatile uint16_t s_last_high_adc;
static uint32_t s_startup_start_us;
static uint32_t s_sector_start_us;
static uint32_t s_previous_zc_us;
static uint32_t s_crossing_us;
static uint32_t s_before_cross_us;
static uint32_t s_duty_ramp_start_us;
static uint8_t s_closed_duty_ramping;
static uint16_t s_closed_from_duty;
static int32_t s_before_cross_diff;
static uint8_t s_zc_streak;
static uint8_t s_previous_zc_valid;
static uint8_t s_sector_zc_seen;
static uint8_t s_expected_rising;
static uint8_t s_detector_armed;
static uint8_t s_cross_candidate;
static uint8_t s_confirm_samples;
static uint8_t s_bad_adc_samples;

static void BLDC_BEMF_BeginSector(uint8_t step, BLDC_Phase floating_phase,
                                 BLDC_Phase high_phase);
static void BLDC_Fail(BLDC_Fault fault);
#endif
static void BLDC_StartSequence(void);

#define BLDC_BLEND_SCALE (1024UL)

#define BLDC_CCER_OUTPUT_MASK (TIM_CCER_CC1E  | TIM_CCER_CC1NE | \
                               TIM_CCER_CC2E  | TIM_CCER_CC2NE | \
                               TIM_CCER_CC3E  | TIM_CCER_CC3NE)

#define BLDC_CCER_HIGH_MASK   (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)
#define BLDC_CCER_LOW_MASK    (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)

static uint16_t BLDC_GetArr(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}

static uint16_t BLDC_PercentToCcr(uint8_t percent)
{
    uint32_t arr_plus_1 = (uint32_t)BLDC_GetArr() + 1UL;

    if (percent > 100U)
    {
        percent = 100U;
    }

    return (uint16_t)((arr_plus_1 * percent) / 100UL);
}

static uint16_t BLDC_LimitRunDuty(uint16_t duty)
{
    uint16_t max_safe = BLDC_PercentToCcr(BLDC_MAX_RUN_DUTY_PERCENT);

    if (duty > max_safe)
    {
        duty = max_safe;
    }

    return duty;
}

static uint8_t BLDC_LimitRunPercent(uint8_t percent)
{
    if (percent > BLDC_MAX_RUN_DUTY_PERCENT)
    {
        percent = BLDC_MAX_RUN_DUTY_PERCENT;
    }

    return percent;
}

static uint32_t BLDC_ChannelFromPhase(BLDC_Phase phase)
{
    switch (phase)
    {
        case BLDC_PHASE_A: return TIM_CHANNEL_3; /* PA10 / PB15 */
        case BLDC_PHASE_B: return TIM_CHANNEL_2; /* PA9  / PB14 */
        case BLDC_PHASE_C: return TIM_CHANNEL_1; /* PA8  / PB13 */
        default:           return 0UL;
    }
}

static uint32_t BLDC_CcerHighBit(BLDC_Phase phase)
{
    switch (phase)
    {
        case BLDC_PHASE_A: return TIM_CCER_CC3E;
        case BLDC_PHASE_B: return TIM_CCER_CC2E;
        case BLDC_PHASE_C: return TIM_CCER_CC1E;
        default:           return 0UL;
    }
}

static uint32_t BLDC_CcerLowBit(BLDC_Phase phase)
{
    switch (phase)
    {
        case BLDC_PHASE_A: return TIM_CCER_CC3NE;
        case BLDC_PHASE_B: return TIM_CCER_CC2NE;
        case BLDC_PHASE_C: return TIM_CCER_CC1NE;
        default:           return 0UL;
    }
}

static void BLDC_SetCompareByPhase(BLDC_Phase phase, uint16_t duty)
{
    uint32_t channel = BLDC_ChannelFromPhase(phase);

    /*
     * 注意：STM32 HAL 中 TIM_CHANNEL_1 的值就是 0x00000000。
     * 不能用 channel != 0 判断 CH1 是否有效，否则 C 相不会写入 CCR1。
     */
    if (phase <= BLDC_PHASE_C)
    {
        __HAL_TIM_SET_COMPARE(&htim1, channel, duty);
    }
}

/*
 * Set OCxM without touching the other channel.
 * TIM1_CH1 -> CCMR1 bits 6:4
 * TIM1_CH2 -> CCMR1 bits 14:12
 * TIM1_CH3 -> CCMR2 bits 6:4
 */
static void BLDC_SetChannelMode(uint32_t channel, uint32_t mode)
{
    switch (channel)
    {
        case TIM_CHANNEL_1:
            MODIFY_REG(TIM1->CCMR1, TIM_CCMR1_OC1M, mode);
            break;

        case TIM_CHANNEL_2:
            MODIFY_REG(TIM1->CCMR1, TIM_CCMR1_OC2M, mode << 8U);
            break;

        case TIM_CHANNEL_3:
            MODIFY_REG(TIM1->CCMR2, TIM_CCMR2_OC3M, mode);
            break;

        default:
            break;
    }
}

static uint32_t BLDC_GetTIM2ClockHz(void)
{
    RCC_ClkInitTypeDef clk_config;
    uint32_t flash_latency;
    uint32_t pclk1;

    HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
    pclk1 = HAL_RCC_GetPCLK1Freq();

    /* On STM32F4, APB timer clock is doubled when APB prescaler != 1. */
    if (clk_config.APB1CLKDivider == RCC_HCLK_DIV1)
    {
        return pclk1;
    }

    return pclk1 * 2UL;
}

static void BLDC_ConfigTIM2Counter(void)
{
    uint32_t tim_clk = BLDC_GetTIM2ClockHz();
    uint32_t divider = tim_clk / BLDC_TIM2_COUNTER_HZ;

    /* TIM2 belongs to this driver. Cache/configure the 1 us timebase once,
     * rather than reading RCC and dividing 64-bit values in every ISR. */
    if ((divider == 0UL) || (divider > 65536UL) ||
        ((tim_clk % BLDC_TIM2_COUNTER_HZ) != 0UL))
    {
        Error_Handler();
        return;
    }
    htim2.Init.Prescaler = divider - 1UL;
    TIM2->PSC = divider - 1UL;
    TIM2->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

static void BLDC_SetTim2PeriodUs(uint32_t period_us)
{
    uint32_t was_running = TIM2->CR1 & TIM_CR1_CEN;
    uint32_t update_irq = TIM2->DIER & TIM_IT_UPDATE;

    if (period_us < 2UL)
    {
        period_us = 2UL;
    }

    TIM2->CR1 &= ~TIM_CR1_CEN;
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period_us - 1UL);

    /* TIM2 has ARR preload enabled.  Force an update event so this ARR takes
     * effect now. Clear the synthetic UIF before restoring the real IRQ. */
    TIM2->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim2, 0UL);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    TIM2->DIER |= update_irq;
    TIM2->CR1 |= was_running;

    s_debug_period_us = period_us;
}

static void BLDC_ConfigTimerSafeState(void)
{
    uint32_t cc2 = TIM1->CR2;

    /*
     * Idle/off-state:
     *   CHx  -> LOW  (HIN=0, high MOS OFF)
     *   CHxN -> HIGH (LIN=1, low MOS OFF)
     */
    cc2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N |
             TIM_CR2_OIS2 | TIM_CR2_OIS2N |
             TIM_CR2_OIS3 | TIM_CR2_OIS3N);

    cc2 |= TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N;
    /* OCxM and CCxE/CCxNE are preloaded together and committed by COMG.
     * Leave CCUS clear so only software initiates a commutation event. */
    TIM1->CR2 = (cc2 | TIM_CR2_CCPC) & ~TIM_CR2_CCUS;

    /*
     * OSSI/OSSR keep the timer outputs in their defined inactive/off state
     * instead of falling back to the GPIO ODR during commutation/idle.
     */
    TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;

    /*
     * CHx active high.
     * CHxN active low. This is important because EG2133 LIN is active low.
     */
    TIM1->CCER &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P  | TIM_CCER_CC3P |
                    TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

    TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
}

static void BLDC_AllOutputsOff(void)
{
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    BLDC_BEMF_DisableTrigger();
#endif
    /* Unconditionally gate off first. Keep each CHxN selected in idle so
     * OSSI/OISxN can actively hold LIN high instead of relying on a Hi-Z pin. */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    BLDC_SetChannelMode(TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
    BLDC_SetChannelMode(TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
    BLDC_SetChannelMode(TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
    MODIFY_REG(TIM1->CCER, BLDC_CCER_OUTPUT_MASK, BLDC_CCER_LOW_MASK);
    TIM1->EGR = TIM_EGR_COMG;
}

static void BLDC_StartTimerOnce(void)
{
    BLDC_ConfigTimerSafeState();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG | TIM_EGR_COMG;

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    BLDC_AllOutputsOff();
}

/*
 * Logical six-step table:
 *   0: A+ B-
 *   1: A+ C-
 *   2: B+ C-
 *   3: B+ A-
 *   4: C+ A-
 *   5: C+ B-
 *
 * Direction is applied only here. The ISR itself always advances 0..5.
 */
static void BLDC_GetStepPhases(uint8_t logical_step,
                               BLDC_Phase *high_pwm,
                               BLDC_Phase *low_on)
{
    static const BLDC_StepDefinition forward_table[6] =
    {
        { BLDC_PHASE_A, BLDC_PHASE_B },
        { BLDC_PHASE_A, BLDC_PHASE_C },
        { BLDC_PHASE_B, BLDC_PHASE_C },
        { BLDC_PHASE_B, BLDC_PHASE_A },
        { BLDC_PHASE_C, BLDC_PHASE_A },
        { BLDC_PHASE_C, BLDC_PHASE_B }
    };

    uint8_t step = logical_step % 6U;

    if (motor.dir == MOTOR_REVERSE)
    {
        /* Reverse sequence = forward sequence traversed backwards. */
        step = (uint8_t)((6U - step) % 6U);
    }

    *high_pwm = forward_table[step].high_phase;
    *low_on   = forward_table[step].low_phase;
}

static void BLDC_SetPhaseOutput(BLDC_Phase phase, BLDC_PhaseMode mode)
{
    uint32_t channel = BLDC_ChannelFromPhase(phase);
    uint32_t high_bit = BLDC_CcerHighBit(phase);
    uint32_t low_bit = BLDC_CcerLowBit(phase);
    uint32_t mode_value;

    /*
     * TIM_CHANNEL_1 == 0 是合法值，不能把 channel == 0 当成错误。
     * 这里用 phase 范围判断有效性。
     */
    if ((phase > BLDC_PHASE_C) || (high_bit == 0UL) || (low_bit == 0UL))
    {
        return;
    }

    switch (mode)
    {
        case PHASE_MODE_HIGH_PWM:
            /* OCxREF = PWM1. CHx is HIN. CHxN is left in its safe high off-state. */
            mode_value = TIM_OCMODE_PWM1;
            BLDC_SetChannelMode(channel, mode_value);
            BLDC_SetCompareByPhase(phase, motor.duty);
            TIM1->CCER &= ~(high_bit | low_bit);
            TIM1->CCER |= high_bit;
            break;

        case PHASE_MODE_LOW_ON:
            /*
             * With CCxNP=1 and only CHxN enabled:
             *   OCREF=1 -> CHxN=0 -> LIN=0 -> low MOS ON.
             */
            mode_value = TIM_OCMODE_FORCED_ACTIVE;
            BLDC_SetChannelMode(channel, mode_value);
            TIM1->CCER &= ~(high_bit | low_bit);
            TIM1->CCER |= low_bit;
            break;

        case PHASE_MODE_FLOAT:
        default:
            /*
             * With CCxNP=1 and only CHxN enabled:
             *   OCREF=0 -> CHxN=1 -> LIN=1 -> low MOS OFF.
             * CHx is disabled, so HIN is in the configured safe off-state.
             */
            mode_value = TIM_OCMODE_FORCED_INACTIVE;
            BLDC_SetChannelMode(channel, mode_value);
            TIM1->CCER &= ~(high_bit | low_bit);
            TIM1->CCER |= low_bit;
            break;
    }
}

static void BLDC_ApplyStep(uint8_t step)
{
    BLDC_Phase high_pwm = BLDC_PHASE_NONE;
    BLDC_Phase low_on = BLDC_PHASE_NONE;
    BLDC_Phase float_phase = BLDC_PHASE_NONE;

    step %= 6U;
    BLDC_GetStepPhases(step, &high_pwm, &low_on);

    if ((high_pwm == BLDC_PHASE_NONE) ||
        (low_on == BLDC_PHASE_NONE) ||
        (high_pwm == low_on))
    {
        BLDC_AllOutputsOff();
        motor.state = BLDC_STATE_FAULT;
        return;
    }

    /* Find the third phase. */
    if ((BLDC_PHASE_A != high_pwm) && (BLDC_PHASE_A != low_on))
        float_phase = BLDC_PHASE_A;
    else if ((BLDC_PHASE_B != high_pwm) && (BLDC_PHASE_B != low_on))
        float_phase = BLDC_PHASE_B;
    else
        float_phase = BLDC_PHASE_C;

    /*
     * Disable MOE and every channel before changing the six-step state.
     * This guarantees a true all-off interval even if this function is called
     * directly while the motor is already running.
     */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    TIM1->CCER &= ~BLDC_CCER_OUTPUT_MASK;

    /* Configure all three phases while their outputs are disabled. */
    BLDC_SetPhaseOutput(float_phase, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(low_on, PHASE_MODE_LOW_ON);
    BLDC_SetPhaseOutput(high_pwm, PHASE_MODE_HIGH_PWM);

    /* Only after the whole state is prepared, enable the main output. */
    /* COMG commits all three modes/enables together without resetting the
     * PWM counter or truncating/restarting its carrier on every sector. */
    TIM1->EGR = TIM_EGR_COMG;
    TIM1->CR1 |= TIM_CR1_CEN;
    __HAL_TIM_MOE_ENABLE(&htim1);

    motor.step = step;
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    BLDC_BEMF_BeginSector(step, float_phase, high_pwm);
#endif
}

static void BLDC_ApplyDuty(uint16_t duty)
{
    uint32_t update_disabled = TIM1->CR1 & TIM_CR1_UDIS;

    motor.duty = BLDC_LimitRunDuty(duty);
    motor.duty_percent = (uint8_t)(((uint32_t)motor.duty * 100UL) /
                                   ((uint32_t)BLDC_GetArr() + 1UL));
    /* Prevent a PWM update halfway through these writes. All CCR values
     * latch on the next natural PWM update, with no carrier restart. */
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CCR1 = motor.duty;
    TIM1->CCR2 = motor.duty;
    TIM1->CCR3 = motor.duty;
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    BLDC_BEMF_SetSamplePoint(motor.duty);
#endif
    if (update_disabled == 0UL)
    {
        TIM1->CR1 &= ~TIM_CR1_UDIS;
    }
}

static uint32_t BLDC_Interpolate(uint32_t from, uint32_t to, uint32_t blend)
{
    if (to >= from)
    {
        return from + ((to - from) * blend) / BLDC_BLEND_SCALE;
    }
    return from - ((from - to) * blend) / BLDC_BLEND_SCALE;
}

static void BLDC_BeginRamp(void)
{
    s_ramp_start_ms = HAL_GetTick();
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    s_ramp_start_us = BLDC_BEMF_NowUs();
#endif
    s_ramp_from_rate_millihz = 1000000000UL / motor.commutation_period_us;
    s_ramp_from_duty = motor.duty;
    motor.state = BLDC_STATE_OPEN_LOOP_STARTING;
}

static void BLDC_RampOpenLoop(void)
{
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    uint32_t elapsed_ms = s_sensorless != 0U ?
                          (uint32_t)(BLDC_BEMF_NowUs() - s_ramp_start_us) / 1000UL :
                          HAL_GetTick() - s_ramp_start_ms;
    uint32_t ramp_ms = s_sensorless != 0U ? BLDC_BEMF_STARTUP_RAMP_MS : BLDC_RAMP_DURATION_MS;
#else
    uint32_t elapsed_ms = HAL_GetTick() - s_ramp_start_ms;
    uint32_t ramp_ms = BLDC_RAMP_DURATION_MS;
#endif
    uint32_t blend;
    uint32_t rate_millihz;

    if (elapsed_ms >= ramp_ms)
    {
        motor.commutation_period_us = s_target_period_us;
        BLDC_ApplyDuty(s_target_duty);
        motor.state = BLDC_STATE_OPEN_LOOP_RUNNING;
        return;
    }

    /* Smoothstep: frequency (not period) accelerates with zero initial and
     * final slope. Its duration depends on wall time, not number of sectors. */
    blend = (elapsed_ms * BLDC_BLEND_SCALE) / ramp_ms;
    blend = blend * blend * (3UL * BLDC_BLEND_SCALE - 2UL * blend) /
            (BLDC_BLEND_SCALE * BLDC_BLEND_SCALE);
    rate_millihz = BLDC_Interpolate(s_ramp_from_rate_millihz,
                                    1000000000UL / s_target_period_us, blend);
    motor.commutation_period_us = 1000000000UL / rate_millihz;
    BLDC_ApplyDuty((uint16_t)BLDC_Interpolate(s_ramp_from_duty,
                                            s_target_duty, blend));
}

static void BLDC_PrechargeAllPhases(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_LOW_ON);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_LOW_ON);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_LOW_ON);
    TIM1->EGR = TIM_EGR_COMG;
    __HAL_TIM_MOE_ENABLE(&htim1);
}

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
static void BLDC_Fail(BLDC_Fault fault)
{
    BLDC_Stop();
    s_fault = fault;
    motor.state = BLDC_STATE_FAULT;
    s_debug_pending = 1U;
}

static void BLDC_ClosedLoopDuty(void)
{
    if (s_closed_duty_ramping == 0U) return;
    uint32_t elapsed = (uint32_t)(BLDC_BEMF_NowUs() - s_duty_ramp_start_us) / 1000UL;
    uint32_t blend = elapsed >= BLDC_BEMF_DUTY_RAMP_MS ? BLDC_BLEND_SCALE :
                     elapsed * BLDC_BLEND_SCALE / BLDC_BEMF_DUTY_RAMP_MS;
    BLDC_ApplyDuty((uint16_t)BLDC_Interpolate(s_closed_from_duty, s_target_duty, blend));
    if (elapsed >= BLDC_BEMF_DUTY_RAMP_MS) s_closed_duty_ramping = 0U;
}

static void BLDC_BEMF_BeginSector(uint8_t step, BLDC_Phase floating_phase,
                                 BLDC_Phase high_phase)
{
    if (s_sensorless == 0U ||
        ((motor.state != BLDC_STATE_OPEN_LOOP_STARTING) &&
         (motor.state != BLDC_STATE_OPEN_LOOP_RUNNING) &&
         (motor.state != BLDC_STATE_CLOSED_LOOP_RUNNING))) return;
    if (s_sector_zc_seen == 0U)
    {
        s_zc_streak = 0U;
        s_previous_zc_valid = 0U;
    }
    s_sector_zc_seen = 0U;
    s_detector_armed = 0U;
    s_cross_candidate = 0U;
    s_confirm_samples = 0U;
    s_bad_adc_samples = 0U;
    s_sector_start_us = BLDC_BEMF_NowUs();
    uint8_t physical_step = motor.dir == MOTOR_REVERSE ? (6U - step) % 6U : step;
    /* For A+B- -> A+C-, floating C crosses downward before becoming low.
     * The other sectors alternate; reverse traversal reverses the slope. */
    s_expected_rising = (physical_step & 1U) ^ (motor.dir == MOTOR_REVERSE ? 1U : 0U);
    BLDC_BEMF_SelectPhases(floating_phase, high_phase);
}

static uint8_t BLDC_BEMF_ScheduleCommutation(uint32_t crossing_us)
{
    uint32_t period = s_measured_period_us;
    if (period < BLDC_BEMF_MIN_PERIOD_US)
    {
        BLDC_Fail(BLDC_FAULT_OVERSPEED);
        return 0U;
    }
    if (period > BLDC_BEMF_MAX_PERIOD_US)
    {
        BLDC_Fail(BLDC_FAULT_BEMF_LOST);
        return 0U;
    }
    /* Successive ZCs are 60 electrical degrees apart. Delay half that
     * interval (30 degrees). TIM2 expiry begins the 5us all-off gap. */
    uint32_t deadline = crossing_us + period / 2UL - BLDC_COMMUTATION_BLANK_US;
    int32_t remaining = (int32_t)(deadline - BLDC_BEMF_NowUs());
    if (remaining < 10)
    {
        BLDC_Fail(BLDC_FAULT_TIMING);
        return 0U;
    }
    motor.commutation_period_us = period;
    BLDC_SetTim2PeriodUs((uint32_t)remaining);
    return 1U;
}

void BLDC_BEMF_Sample(uint16_t floating_adc, uint16_t high_adc, uint32_t time_us)
{
    /* TIM2 (priority 0) can change a sector. Keep one ADC sample and its
     * rearm atomic relative to it. No HAL waits/UART calls in this section. */
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    if (s_sensorless == 0U || s_blank_active != 0U ||
        ((motor.state != BLDC_STATE_OPEN_LOOP_STARTING) &&
         (motor.state != BLDC_STATE_OPEN_LOOP_RUNNING) &&
         (motor.state != BLDC_STATE_CLOSED_LOOP_RUNNING))) goto done;
    s_last_float_adc = floating_adc;
    s_last_high_adc = high_adc;
    uint32_t elapsed = time_us - s_sector_start_us;
    uint32_t demag = motor.commutation_period_us / 8UL;
    if (demag > BLDC_BEMF_DEMAG_MAX_US) demag = BLDC_BEMF_DEMAG_MAX_US;
    if (demag < 50UL) demag = 50UL;
    if (elapsed < demag || elapsed > BLDC_BEMF_MAX_PERIOD_US * 2UL) goto done;
    if (high_adc < 512U || high_adc >= 4090U || floating_adc >= 4090U)
    {
        if (++s_bad_adc_samples >= 8U) BLDC_Fail(BLDC_FAULT_ADC_RANGE);
        goto done;
    }
    s_bad_adc_samples = 0U;
    if (s_sector_zc_seen != 0U) goto done;

    /* On-time neutral approximation: (high phase + grounded low phase)/2.
     * ADC1 and ADC2 see floating and high phase at the same trigger instant.
     * Normalize so all expected crossings move from negative to positive. */
    int32_t diff = 2L * (int32_t)floating_adc - (int32_t)high_adc;
    if (s_expected_rising == 0U) diff = -diff;
    if (s_detector_armed == 0U)
    {
        if (diff <= -BLDC_BEMF_HYSTERESIS)
        {
            s_detector_armed = 1U;
            s_before_cross_diff = diff;
            s_before_cross_us = time_us;
        }
        goto done;
    }
    if (diff < 0)
    {
        s_before_cross_diff = diff;
        s_before_cross_us = time_us;
        s_cross_candidate = 0U;
        s_confirm_samples = 0U;
        goto done;
    }
    if (s_cross_candidate == 0U)
    {
        uint32_t dt = time_us - s_before_cross_us;
        if (dt > 100UL)
        {
            s_detector_armed = 0U;
            goto done; /* Missing samples: do not invent a crossing time. */
        }
        uint32_t before = (uint32_t)(-s_before_cross_diff);
        s_crossing_us = s_before_cross_us + before * dt / (before + (uint32_t)diff);
        s_cross_candidate = 1U;
    }
    if (diff < BLDC_BEMF_HYSTERESIS)
    {
        s_confirm_samples = 0U;
        goto done;
    }
    if (++s_confirm_samples < BLDC_BEMF_CONFIRM_SAMPLES) goto done;
    s_sector_zc_seen = 1U;
    uint32_t offset = s_crossing_us - s_sector_start_us;
    uint32_t forced_period = motor.commutation_period_us;
    if (offset < forced_period / 6UL || offset > forced_period * 5UL / 6UL)
    {
        if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING) BLDC_Fail(BLDC_FAULT_BEMF_LOST);
        else { s_zc_streak = 0U; s_previous_zc_valid = 0U; }
        goto done;
    }
    uint32_t interval = s_crossing_us - s_previous_zc_us;
    uint8_t plausible = s_previous_zc_valid != 0U &&
                        interval >= BLDC_BEMF_MIN_PERIOD_US &&
                        interval <= BLDC_BEMF_MAX_PERIOD_US &&
                        interval >= forced_period * 2UL / 3UL &&
                        interval <= forced_period * 4UL / 3UL;
    if (plausible != 0U)
    {
        s_measured_period_us = s_zc_streak < 2U ? interval :
                               (s_measured_period_us * 3UL + interval) / 4UL;
        if (s_zc_streak < 255U) ++s_zc_streak;
    }
    else
    {
        if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING)
        {
            BLDC_Fail(interval < BLDC_BEMF_MIN_PERIOD_US ? BLDC_FAULT_OVERSPEED : BLDC_FAULT_BEMF_LOST);
            goto done;
        }
        s_zc_streak = 1U;
        s_measured_period_us = 0U;
    }
    s_previous_zc_us = s_crossing_us;
    s_previous_zc_valid = 1U;
    ++s_zc_total;
    if (motor.state != BLDC_STATE_CLOSED_LOOP_RUNNING &&
        s_zc_streak >= BLDC_BEMF_HANDOFF_CROSSINGS &&
        s_measured_period_us <= BLDC_BEMF_HANDOFF_MAX_PERIOD_US &&
        offset >= forced_period / 3UL && offset <= forced_period * 2UL / 3UL)
    {
        motor.state = BLDC_STATE_CLOSED_LOOP_RUNNING;
        s_closed_from_duty = motor.duty;
        s_duty_ramp_start_us = BLDC_BEMF_NowUs();
        s_closed_duty_ramping = 1U;
        s_debug_pending = 1U;
    }
    if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING)
        (void)BLDC_BEMF_ScheduleCommutation(s_crossing_us);
done:
    __set_PRIMASK(irq_mask);
}
#else
void BLDC_BEMF_Sample(uint16_t floating_adc, uint16_t high_adc, uint32_t time_us)
{
    (void)floating_adc;
    (void)high_adc;
    (void)time_us;
}
#endif

void BLDC_Init(void)
{
    motor.max_duty = BLDC_PercentToCcr(BLDC_MAX_RUN_DUTY_PERCENT);
    motor.dir = MOTOR_FORWARD;
    motor.step = 0U;
    motor.commutation_period_us = BLDC_START_PERIOD_US;
    motor.state = BLDC_STATE_STOPPED;
    BLDC_StartTimerOnce();
    HAL_TIM_Base_Stop_IT(&htim2);
    BLDC_ConfigTIM2Counter();
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    BLDC_BEMF_Init();
    s_sensorless = 0U;
    s_fault = BLDC_FAULT_NONE;
    s_measured_period_us = 0U;
#endif
    s_target_period_us = BLDC_TARGET_PERIOD_US;
    s_target_duty = BLDC_PercentToCcr(BLDC_TARGET_DUTY_PERCENT);
    BLDC_ApplyDuty(0U);

    printf("\r\n=== BLDC INIT ===\r\n");
    printf("Mapping: A=CH3/CH3N, B=CH2/CH2N, C=CH1/CH1N\r\n");
    printf("TIM1 PSC=%lu ARR=%lu CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
           (uint32_t)TIM1->PSC,
           (uint32_t)TIM1->ARR,
           (uint32_t)TIM1->CCR1,
           (uint32_t)TIM1->CCR2,
           (uint32_t)TIM1->CCR3);
    printf("TIM1 BDTR=0x%08lX CCER=0x%08lX\r\n",
           (uint32_t)TIM1->BDTR,
           (uint32_t)TIM1->CCER);
    printf("TIM2 timer clock=%lu Hz, counter=%lu Hz, PSC=%lu\r\n",
           BLDC_GetTIM2ClockHz(),
           BLDC_TIM2_COUNTER_HZ,
           (uint32_t)TIM2->PSC);
    printf("Control mode: %s\r\n", BLDC_CONTROL_MODE_NAME);
}

void BLDC_Task(void)
{
    static uint32_t last_debug_ms = 0U;

    /* UART output is outside the motor ISR and limited to 2 Hz. */
    if ((uint32_t)(HAL_GetTick() - last_debug_ms) < 500UL)
    {
        return;
    }
    if (s_debug_pending != 0U)
    {
        last_debug_ms = HAL_GetTick();
        s_debug_pending = 0U;

        printf("BLDC: state=%u step=%u duty=%u%% CCR1=%lu CCR2=%lu CCR3=%lu period=%lu us\r\n",
               motor.state,
               motor.step,
               motor.duty_percent,
               (uint32_t)TIM1->CCR1,
               (uint32_t)TIM1->CCR2,
               (uint32_t)TIM1->CCR3,
               s_debug_period_us);

        printf("      CR1=0x%08lX CR2=0x%08lX CCER=0x%08lX BDTR=0x%08lX CNT=%lu\r\n",
               (uint32_t)TIM1->CR1,
               (uint32_t)TIM1->CR2,
               (uint32_t)TIM1->CCER,
               (uint32_t)TIM1->BDTR,
               (uint32_t)TIM1->CNT);
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
        printf("      BEMF ready=%u fault=%u ZC=%lu streak=%u float=%u high=%u measured=%lu us RPM=%lu\r\n",
               BLDC_BEMF_HARDWARE_READY, (unsigned int)s_fault,
               (uint32_t)s_zc_total, s_zc_streak,
               s_last_float_adc, s_last_high_adc,
               (uint32_t)s_measured_period_us, BLDC_GetMeasuredRpm());
#endif
    }
}

void BLDC_SetDuty(uint16_t duty)
{
    uint32_t irq_mask = __get_PRIMASK();
    uint16_t target = BLDC_LimitRunDuty(duty);
    __disable_irq();
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if (s_sensorless != 0U && target == 0U)
    {
        BLDC_Stop();
        __set_PRIMASK(irq_mask);
        return;
    }
    if (s_sensorless != 0U && target != 0U && target < BLDC_PercentToCcr(BLDC_START_DUTY_PERCENT))
        target = BLDC_PercentToCcr(BLDC_START_DUTY_PERCENT);
    if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING)
    {
        if (target == 0U)
        {
            BLDC_Stop();
        }
        else if (target != s_target_duty)
        {
            s_target_duty = target;
            s_closed_from_duty = motor.duty;
            s_duty_ramp_start_us = BLDC_BEMF_NowUs();
            s_closed_duty_ramping = 1U;
        }
    }
    else if ((motor.state == BLDC_STATE_OPEN_LOOP_STARTING) ||
        (motor.state == BLDC_STATE_OPEN_LOOP_RUNNING))
    {
        if (target != s_target_duty)
        {
            s_target_duty = target;
            /* Sensorless startup retains its acquisition clock/profile. */
            if (s_sensorless == 0U) BLDC_BeginRamp();
        }
    }
#else
    if ((motor.state == BLDC_STATE_OPEN_LOOP_STARTING) ||
        (motor.state == BLDC_STATE_OPEN_LOOP_RUNNING))
    {
        if (target != s_target_duty)
        {
            s_target_duty = target;
            BLDC_BeginRamp();
        }
    }
#endif
    else
    {
        s_target_duty = target;
        if ((motor.state != BLDC_STATE_ALIGNING) &&
            (motor.state != BLDC_STATE_BOOTSTRAP_PRECHARGE))
        {
            BLDC_ApplyDuty(target);
        }
    }
    __set_PRIMASK(irq_mask);
}

void BLDC_SetDutyPercent(uint8_t percent)
{
    BLDC_SetDuty(BLDC_PercentToCcr(BLDC_LimitRunPercent(percent)));
}

void BLDC_SetSpeed(uint16_t duty)
{
    /* Kept for compatibility with the previous interface. */
    BLDC_SetDuty(duty);
}

void BLDC_SetDirection(Motor_Direction dir)
{
    if ((dir != MOTOR_FORWARD) && (dir != MOTOR_REVERSE))
    {
        return;
    }

    /* Do not reverse a running motor abruptly. Stop first. */
    if ((motor.state != BLDC_STATE_STOPPED) &&
        (motor.state != BLDC_STATE_FAULT))
    {
        return;
    }

    motor.dir = dir;
}

void BLDC_SetCommutationPeriodUs(uint32_t period_us)
{
    uint32_t irq_mask = __get_PRIMASK();

    if (period_us < BLDC_MIN_PERIOD_US)
    {
        period_us = BLDC_MIN_PERIOD_US;
    }
    if (period_us > BLDC_MAX_PERIOD_US)
    {
        period_us = BLDC_MAX_PERIOD_US;
    }
    __disable_irq();
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if (s_sensorless != 0U)
    {
        __set_PRIMASK(irq_mask);
        return; /* Startup profile and rotor feedback own the sector time. */
    }
#endif
    if (period_us != s_target_period_us)
    {
        s_target_period_us = period_us;
        if ((motor.state == BLDC_STATE_OPEN_LOOP_STARTING) ||
            (motor.state == BLDC_STATE_OPEN_LOOP_RUNNING))
        {
            BLDC_BeginRamp();
        }
    }
    __set_PRIMASK(irq_mask);
}

void BLDC_Start(void)
{
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    BLDC_SensorlessStart();
#else
    BLDC_OpenLoopStart();
#endif
}

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
void BLDC_SensorlessStart(void)
{
    BLDC_Stop();
    if (BLDC_BEMF_HARDWARE_READY == 0U)
    {
        BLDC_Fail(BLDC_FAULT_BEMF_HARDWARE);
        printf("BLDC NOT STARTED: verify 10k/4.7k dividers and C<=100pF, then set BEMF_HARDWARE_READY=1\r\n");
        return;
    }
    BLDC_StartSequence();
    printf("BLDC SENSORLESS: acquire %u crossings; commutate 30 electrical degrees after ZC\r\n",
           BLDC_BEMF_HANDOFF_CROSSINGS);
}

void BLDC_OpenLoopStart(void)
{
    BLDC_Stop();
    printf("BLDC open-loop implementation is excluded by BLDC_CONTROL_MODE\r\n");
}
#else
void BLDC_SensorlessStart(void)
{
    BLDC_Stop();
    printf("BLDC sensorless implementation is excluded by BLDC_CONTROL_MODE\r\n");
}

void BLDC_OpenLoopStart(void)
{
    BLDC_StartSequence();
}
#endif

static void BLDC_StartSequence(void)
{
    BLDC_EmergencyStop();
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    s_sensorless = 1U;
    s_fault = BLDC_FAULT_NONE;
    s_startup_start_us = BLDC_BEMF_NowUs();
    s_zc_streak = 0U;
    s_zc_total = 0U;
    s_measured_period_us = 0U;
    s_previous_zc_valid = 0U;
    s_sector_zc_seen = 0U;
    s_bad_adc_samples = 0U;
#endif
    motor.state = BLDC_STATE_BOOTSTRAP_PRECHARGE;
    motor.step = 0U;
    motor.commutation_period_us = BLDC_START_PERIOD_US;
    s_alignment_elapsed_us = 0UL;
    BLDC_ApplyDuty(0U);
    /* All low sides on, all high sides off: zero phase-to-phase voltage. */
    BLDC_PrechargeAllPhases();

    s_debug_period_us = motor.commutation_period_us;
    s_debug_pending = 1U;

    BLDC_SetTim2PeriodUs(BLDC_BOOTSTRAP_PRECHARGE_US);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    __set_PRIMASK(irq_mask);

    printf("BLDC %s START: precharge, soft alignment, %lu ms ramp, target=%lu us/%u%%\r\n",
           BLDC_CONTROL_MODE_NAME,
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
           BLDC_BEMF_STARTUP_RAMP_MS,
#else
           BLDC_RAMP_DURATION_MS,
#endif
           s_target_period_us,
           (unsigned int)((uint32_t)s_target_duty * 100UL / (BLDC_GetArr() + 1UL)));
}

void BLDC_Stop(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    HAL_TIM_Base_Stop_IT(&htim2);
    s_blank_active = 0U;
    s_pending_step = 0U;
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    s_sensorless = 0U;
    s_previous_zc_valid = 0U;
    s_measured_period_us = 0U;
    s_closed_duty_ramping = 0U;
#endif
    BLDC_AllOutputsOff();

    motor.state = BLDC_STATE_STOPPED;
    BLDC_ApplyDuty(0U);
    __set_PRIMASK(irq_mask);
}

void BLDC_EmergencyStop(void)
{
    BLDC_Stop();
}

void BLDC_Commutation(uint8_t step)
{
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if (s_sensorless != 0U) return; /* Feedback owns the running sequence. */
#endif
    if (step >= 6U)
    {
        step %= 6U;
    }

    BLDC_ApplyStep(step);
    s_debug_period_us = motor.commutation_period_us;
    s_debug_pending = 1U;
}

uint8_t BLDC_GetStep(void)
{
    return motor.step;
}

uint16_t BLDC_GetDuty(void)
{
    return motor.duty;
}

uint8_t BLDC_GetDutyPercent(void)
{
    return motor.duty_percent;
}

BLDC_RunningState BLDC_GetRunningState(void)
{
    return motor.state;
}

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
BLDC_Fault BLDC_GetFault(void) { return s_fault; }

uint32_t BLDC_GetMeasuredRpm(void)
{
    uint32_t period = s_measured_period_us;
    return period != 0U ? 10000000UL / (period * BLDC_MOTOR_POLE_PAIRS) : 0U;
}
#else
BLDC_Fault BLDC_GetFault(void) { return BLDC_FAULT_NONE; }
uint32_t BLDC_GetMeasuredRpm(void) { return 0U; }
#endif

void BLDC_TestHighPWM(BLDC_Phase phase, uint8_t duty_percent)
{
    if (phase > BLDC_PHASE_C)
    {
        return;
    }

    BLDC_Stop();

    duty_percent = BLDC_LimitRunPercent(duty_percent);
    motor.duty_percent = duty_percent;
    motor.duty = BLDC_PercentToCcr(duty_percent);

    /* Put all phases in float state first. */
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_FLOAT);

    BLDC_SetPhaseOutput(phase, PHASE_MODE_HIGH_PWM);

    TIM1->EGR = TIM_EGR_COMG;
    TIM1->CR1 |= TIM_CR1_CEN;
    __HAL_TIM_MOE_ENABLE(&htim1);

    motor.state = BLDC_STATE_PWM_TEST;

    printf("BLDC HIGH PWM TEST: phase=%d duty=%u%%\r\n",
           (int)phase,
           duty_percent);
}

void BLDC_TestLowSide(BLDC_Phase phase)
{
    if (phase > BLDC_PHASE_C)
    {
        return;
    }

    BLDC_Stop();

    /* Put all phases in float state first. */
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_FLOAT);

    BLDC_SetPhaseOutput(phase, PHASE_MODE_LOW_ON);

    TIM1->EGR = TIM_EGR_COMG;
    TIM1->CR1 |= TIM_CR1_CEN;
    __HAL_TIM_MOE_ENABLE(&htim1);

    motor.state = BLDC_STATE_LOW_SIDE_TEST;

    printf("BLDC LOW SIDE TEST: phase=%d (LIN should be LOW)\r\n",
           (int)phase);
}

void BLDC_TestStep(uint8_t step, uint8_t duty_percent)
{
    if (step >= 6U)
    {
        step %= 6U;
    }

    BLDC_Stop();

    duty_percent = BLDC_LimitRunPercent(duty_percent);
    motor.duty_percent = duty_percent;
    motor.duty = BLDC_PercentToCcr(duty_percent);

    motor.state = BLDC_STATE_PWM_TEST;
    BLDC_ApplyDuty(motor.duty);
    BLDC_ApplyStep(step);

    printf("BLDC STEP TEST: step=%u duty=%u%%\r\n",
           step,
           duty_percent);
}

void BLDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM2))
    {
        return;
    }
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if (s_sensorless != 0U && motor.state != BLDC_STATE_CLOSED_LOOP_RUNNING &&
        (uint32_t)(BLDC_BEMF_NowUs() - s_startup_start_us) >= BLDC_BEMF_STARTUP_TIMEOUT_MS * 1000UL)
    {
        BLDC_Fail(BLDC_FAULT_BEMF_STARTUP);
        return;
    }
#endif

    if (motor.state == BLDC_STATE_BOOTSTRAP_PRECHARGE)
    {
        BLDC_AllOutputsOff();
        motor.state = BLDC_STATE_ALIGNING;
        BLDC_ApplyStep(0U);
        BLDC_SetTim2PeriodUs(BLDC_ALIGNMENT_UPDATE_US);
        s_debug_pending = 1U;
        return;
    }

    if (motor.state == BLDC_STATE_ALIGNING)
    {
        s_alignment_elapsed_us += BLDC_ALIGNMENT_UPDATE_US;
        if (s_alignment_elapsed_us <= BLDC_ALIGNMENT_RAMP_US)
        {
            BLDC_ApplyDuty((uint16_t)((uint32_t)BLDC_PercentToCcr(BLDC_START_DUTY_PERCENT) *
                           s_alignment_elapsed_us / BLDC_ALIGNMENT_RAMP_US));
        }
        if (s_alignment_elapsed_us >= (BLDC_ALIGNMENT_RAMP_US + BLDC_ALIGNMENT_HOLD_US))
        {
            BLDC_BeginRamp();
            s_pending_step = 1U;
            BLDC_AllOutputsOff();
            s_blank_active = 1U;
            BLDC_SetTim2PeriodUs(BLDC_COMMUTATION_BLANK_US);
        }
        return;
    }

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if ((motor.state != BLDC_STATE_OPEN_LOOP_STARTING) &&
        (motor.state != BLDC_STATE_OPEN_LOOP_RUNNING) &&
        (motor.state != BLDC_STATE_CLOSED_LOOP_RUNNING))
#else
    if ((motor.state != BLDC_STATE_OPEN_LOOP_STARTING) &&
        (motor.state != BLDC_STATE_OPEN_LOOP_RUNNING))
#endif
    {
        return;
    }

    if (s_blank_active != 0U)
    {
        s_blank_active = 0U;

        BLDC_ApplyStep(s_pending_step);
        /* Compute the next smooth frequency/duty with the phase already
         * conducting, so profile arithmetic does not extend the all-off gap. */
        if (motor.state == BLDC_STATE_OPEN_LOOP_STARTING)
        {
            BLDC_RampOpenLoop();
        }
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
        if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING)
        {
            BLDC_ClosedLoopDuty();
            /* This is a missing-crossing watchdog, not a forced commutation.
             * A valid ZC replaces it with the actual 30-degree deadline. */
            BLDC_SetTim2PeriodUs(motor.commutation_period_us * 3UL / 2UL);
        }
        else
        {
            BLDC_SetTim2PeriodUs(motor.commutation_period_us - BLDC_COMMUTATION_BLANK_US);
        }
#else
        BLDC_SetTim2PeriodUs(motor.commutation_period_us - BLDC_COMMUTATION_BLANK_US);
#endif

        s_debug_period_us = motor.commutation_period_us;
        s_debug_pending = 1U;
        return;
    }

    /*
     * Finish the current electrical sector.
     * Always advance logical step forward; direction is handled in the table.
     */
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF)
    if (motor.state == BLDC_STATE_CLOSED_LOOP_RUNNING && s_sector_zc_seen == 0U)
    {
        BLDC_Fail(BLDC_FAULT_BEMF_LOST);
        return;
    }
#endif
    if (motor.step >= 5U)
    {
        s_pending_step = 0U;
    }
    else
    {
        s_pending_step = (uint8_t)(motor.step + 1U);
    }

    /* Remove all gate commands and wait for the blanking interval. */
    BLDC_AllOutputsOff();
    s_blank_active = 1U;
    BLDC_SetTim2PeriodUs(BLDC_COMMUTATION_BLANK_US);
}

#endif /* BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_SENSORLESS_BEMF */
