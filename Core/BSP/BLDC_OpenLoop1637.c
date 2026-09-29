#include "BLDC.h"

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_OPEN_LOOP_1637)

#include "tim.h"

#include <stdio.h>

/*
 * Open-loop six-step BLDC driver for:
 * STM32F405 + TIM1 + TIM2 + EG2133
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
 * RM0090: with only CHxN enabled it follows OCREF with CCxNP polarity;
 * it is complemented only when CHx and CHxN are enabled together.
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
static volatile uint32_t s_target_period_us = BLDC_TARGET_PERIOD_US;
static uint32_t s_commanded_rpm;
static uint32_t s_slew_credit;
static uint32_t s_duty_elapsed_us;
static volatile uint16_t s_target_duty;
static uint8_t s_pullin_sectors;
static uint32_t s_align_elapsed_us;

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
     * 不能用 channel != 0 判断 CH1 是否有效，否则 A 相永远不会写入 CCR1。
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
    if (tim_clk < BLDC_TIM2_COUNTER_HZ ||
        (tim_clk % BLDC_TIM2_COUNTER_HZ) != 0UL ||
        (tim_clk / BLDC_TIM2_COUNTER_HZ) > 65536UL)
    {
        Error_Handler();
        return;
    }
    htim2.Init.Prescaler = tim_clk / BLDC_TIM2_COUNTER_HZ - 1UL;
    TIM2->PSC = htim2.Init.Prescaler;
    TIM2->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

static void BLDC_SetTim2PeriodUs(uint32_t period_us)
{
    if (period_us < 2UL)
    {
        period_us = 2UL;
    }

    uint32_t was_running = TIM2->CR1 & TIM_CR1_CEN;
    uint32_t update_irq = TIM2->DIER & TIM_IT_UPDATE;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period_us - 1UL);

    /* TIM2 has ARR preload enabled.  Force an update event so this ARR takes
     * effect now; otherwise the initial 0xFFFFFFFF period lasts about 51 s. */
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
    TIM1->CR2 = cc2;

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
}

static void BLDC_AllOutputsOff(void)
{
    /* Gate off first, then keep N outputs in their defined idle-high state.
     * A disabled N pin can otherwise be high impedance at the driver input. */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    BLDC_SetChannelMode(TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
    BLDC_SetChannelMode(TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
    BLDC_SetChannelMode(TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
    MODIFY_REG(TIM1->CCER, BLDC_CCER_OUTPUT_MASK, BLDC_CCER_LOW_MASK);
}

static void BLDC_StartTimerOnce(void)
{
    BLDC_ConfigTimerSafeState();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CNT = 0U;

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
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    __HAL_TIM_MOE_ENABLE(&htim1);

    motor.step = step;
}

static uint32_t BLDC_PeriodToRpm(uint32_t period_us)
{
    return (10000000UL + BLDC_MOTOR_POLE_PAIRS * period_us / 2UL) /
           (BLDC_MOTOR_POLE_PAIRS * period_us);
}

static void BLDC_RampOpenLoop(void)
{
    /* One atomic period snapshot avoids a half-updated period/RPM request. */
    uint32_t target_period_us = s_target_period_us;
    uint32_t target_rpm = BLDC_PeriodToRpm(target_period_us);
    uint32_t acceleration = s_commanded_rpm < target_rpm ?
                            BLDC_ACCEL_RPM_PER_S : BLDC_DECEL_RPM_PER_S;
    s_slew_credit += motor.commutation_period_us * acceleration;
    uint32_t rpm_step = s_slew_credit / 1000000UL;
    s_slew_credit %= 1000000UL;
    if (s_commanded_rpm < target_rpm)
    {
        uint32_t remaining = target_rpm - s_commanded_rpm;
        s_commanded_rpm += rpm_step < remaining ? rpm_step : remaining;
    }
    else if (s_commanded_rpm > target_rpm)
    {
        uint32_t remaining = s_commanded_rpm - target_rpm;
        s_commanded_rpm -= rpm_step < remaining ? rpm_step : remaining;
    }
    if (s_commanded_rpm == target_rpm)
    {
        motor.commutation_period_us = target_period_us;
        motor.state = BLDC_STATE_OPEN_LOOP_RUNNING;
    }
    else
    {
        motor.commutation_period_us = 10000000UL /
                                      (BLDC_MOTOR_POLE_PAIRS * s_commanded_rpm);
        motor.state = BLDC_STATE_OPEN_LOOP_STARTING;
    }

    s_duty_elapsed_us += motor.commutation_period_us;
    if (s_duty_elapsed_us >= BLDC_DUTY_RAMP_INTERVAL_US)
    {
        uint16_t increment = BLDC_PercentToCcr(1U);
        s_duty_elapsed_us -= BLDC_DUTY_RAMP_INTERVAL_US;
        if (motor.duty < s_target_duty)
            motor.duty = s_target_duty - motor.duty < increment ? s_target_duty : motor.duty + increment;
        else if (motor.duty > s_target_duty)
            motor.duty = motor.duty - s_target_duty < increment ? s_target_duty : motor.duty - increment;
        motor.duty_percent = (uint8_t)((uint32_t)motor.duty * 100UL / ((uint32_t)BLDC_GetArr() + 1UL));
        TIM1->CCR1 = motor.duty;
        TIM1->CCR2 = motor.duty;
        TIM1->CCR3 = motor.duty;
    }
}

void BLDC_Init(void)
{
    motor.max_duty = BLDC_PercentToCcr(BLDC_MAX_RUN_DUTY_PERCENT);
    motor.dir = MOTOR_FORWARD;
    motor.step = 0U;
    motor.commutation_period_us = BLDC_START_PERIOD_US;
    motor.state = BLDC_STATE_STOPPED;
    motor.duty = 0U;
    motor.duty_percent = 0U;
    s_target_duty = BLDC_PercentToCcr(BLDC_TARGET_DUTY_PERCENT);
    s_target_period_us = BLDC_TARGET_PERIOD_US;

    BLDC_ConfigTIM2Counter();
    BLDC_StartTimerOnce();
    HAL_TIM_Base_Stop_IT(&htim2);

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
}

void BLDC_Task(void)
{
    if (s_debug_pending != 0U)
    {
        s_debug_pending = 0U;

        printf("BLDC: step=%u duty=%u%% CCR1=%lu CCR2=%lu CCR3=%lu period=%lu us\r\n",
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
    }
}

void BLDC_SetDuty(uint16_t duty)
{
    s_target_duty = BLDC_LimitRunDuty(duty);
    if (s_target_duty == 0U && motor.state != BLDC_STATE_STOPPED)
    {
        BLDC_Stop();
        return;
    }
    if (motor.state == BLDC_STATE_STOPPED || motor.state == BLDC_STATE_FAULT)
    {
        motor.duty = s_target_duty;
        motor.duty_percent = (uint8_t)(((uint32_t)motor.duty * 100UL) /
                                       ((uint32_t)BLDC_GetArr() + 1UL));
    }
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
    if (period_us < BLDC_MIN_PERIOD_US) period_us = BLDC_MIN_PERIOD_US;
    if (period_us > BLDC_MAX_PERIOD_US) period_us = BLDC_MAX_PERIOD_US;
    s_target_period_us = period_us;
}

void BLDC_Start(void)
{
    BLDC_OpenLoopStart();
}

void BLDC_OpenLoopStart(void)
{
    BLDC_EmergencyStop();
    if (s_target_duty == 0U) return;

    motor.state = BLDC_STATE_BOOTSTRAP_PRECHARGE;
    motor.step = 0U;
    motor.commutation_period_us = BLDC_START_PERIOD_US;
    motor.duty = 0U;
    motor.duty_percent = 0U;
    s_commanded_rpm = BLDC_PeriodToRpm(BLDC_START_PERIOD_US);
    s_slew_credit = 0U;
    s_duty_elapsed_us = 0U;
    s_pullin_sectors = 0U;
    s_align_elapsed_us = 0U;

    /* Charge the three high-side bootstrap capacitors with all low sides on. */
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_LOW_ON);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_LOW_ON);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_LOW_ON);
    TIM1->CR1 |= TIM_CR1_CEN;
    __HAL_TIM_MOE_ENABLE(&htim1);

    s_debug_period_us = motor.commutation_period_us;
    s_debug_pending = 1U;

    BLDC_SetTim2PeriodUs(BLDC_BOOTSTRAP_PRECHARGE_US);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    printf("BLDC OPEN LOOP START: target period=%lu us\r\n", s_target_period_us);
}

void BLDC_Stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    s_blank_active = 0U;
    s_pending_step = 0U;
    BLDC_AllOutputsOff();

    motor.state = BLDC_STATE_STOPPED;
    motor.duty = 0U;
    motor.duty_percent = 0U;
}

void BLDC_EmergencyStop(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    s_blank_active = 0U;
    s_pending_step = 0U;

    BLDC_AllOutputsOff();
    motor.state = BLDC_STATE_STOPPED;
    motor.duty = 0U;
    motor.duty_percent = 0U;
}

void BLDC_Commutation(uint8_t step)
{
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

void BLDC_TestHighPWM(BLDC_Phase phase, uint8_t duty_percent)
{
    if (phase > BLDC_PHASE_C)
    {
        return;
    }

    HAL_TIM_Base_Stop_IT(&htim2);
    BLDC_AllOutputsOff();

    duty_percent = BLDC_LimitRunPercent(duty_percent);
    motor.duty_percent = duty_percent;
    motor.duty = BLDC_PercentToCcr(duty_percent);

    /* Put all phases in float state first. */
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_FLOAT);

    BLDC_SetPhaseOutput(phase, PHASE_MODE_HIGH_PWM);

    TIM1->EGR = TIM_EGR_UG;
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

    HAL_TIM_Base_Stop_IT(&htim2);
    BLDC_AllOutputsOff();

    /* Put all phases in float state first. */
    BLDC_SetPhaseOutput(BLDC_PHASE_A, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_B, PHASE_MODE_FLOAT);
    BLDC_SetPhaseOutput(BLDC_PHASE_C, PHASE_MODE_FLOAT);

    BLDC_SetPhaseOutput(phase, PHASE_MODE_LOW_ON);

    TIM1->EGR = TIM_EGR_UG;
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

    HAL_TIM_Base_Stop_IT(&htim2);
    BLDC_AllOutputsOff();

    duty_percent = BLDC_LimitRunPercent(duty_percent);
    motor.duty_percent = duty_percent;
    motor.duty = BLDC_PercentToCcr(duty_percent);

    BLDC_ApplyStep(step);
    motor.state = BLDC_STATE_PWM_TEST;

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

    if (motor.state == BLDC_STATE_BOOTSTRAP_PRECHARGE)
    {
        BLDC_AllOutputsOff();
        motor.duty = 0U;
        motor.duty_percent = 0U;
        BLDC_ApplyStep(0U);
        motor.state = BLDC_STATE_ALIGNING;
        BLDC_SetTim2PeriodUs(BLDC_ALIGNMENT_UPDATE_US);
        return;
    }

    if (motor.state == BLDC_STATE_ALIGNING)
    {
        s_align_elapsed_us += BLDC_ALIGNMENT_UPDATE_US;
        if (s_align_elapsed_us < BLDC_ALIGNMENT_RAMP_US + BLDC_ALIGNMENT_HOLD_US)
        {
            uint32_t ramp_us = s_align_elapsed_us < BLDC_ALIGNMENT_RAMP_US ?
                               s_align_elapsed_us : BLDC_ALIGNMENT_RAMP_US;
            motor.duty = (uint16_t)((uint32_t)BLDC_PercentToCcr(BLDC_ALIGN_DUTY_PERCENT) *
                                    ramp_us / BLDC_ALIGNMENT_RAMP_US);
            motor.duty_percent = (uint8_t)((uint32_t)motor.duty * 100UL /
                                            ((uint32_t)BLDC_GetArr() + 1UL));
            TIM1->CCR1 = motor.duty;
            TIM1->CCR2 = motor.duty;
            TIM1->CCR3 = motor.duty;
            return;
        }
        BLDC_AllOutputsOff();
        motor.duty = BLDC_PercentToCcr(BLDC_START_DUTY_PERCENT);
        motor.duty_percent = BLDC_START_DUTY_PERCENT;
        motor.state = BLDC_STATE_OPEN_LOOP_STARTING;
        s_pending_step = 1U;
        s_blank_active = 1U;
        BLDC_SetTim2PeriodUs(BLDC_COMMUTATION_BLANK_US);
        return;
    }

    if ((motor.state != BLDC_STATE_OPEN_LOOP_STARTING) &&
        (motor.state != BLDC_STATE_OPEN_LOOP_RUNNING))
    {
        return;
    }

    if (s_blank_active != 0U)
    {
        s_blank_active = 0U;

        BLDC_ApplyStep(s_pending_step);
        if (s_pullin_sectors < BLDC_PULLIN_SECTORS) ++s_pullin_sectors;
        BLDC_SetTim2PeriodUs(motor.commutation_period_us - BLDC_COMMUTATION_BLANK_US);

        s_debug_period_us = motor.commutation_period_us;
        s_debug_pending = 1U;
        return;
    }

    /*
     * Finish the current electrical sector.
     * Always advance logical step forward; direction is handled in the table.
     */
    if (motor.step >= 5U)
    {
        s_pending_step = 0U;
    }
    else
    {
        s_pending_step = (uint8_t)(motor.step + 1U);
    }

    /* Ramp speed and duty before the next electrical sector. */
    if (s_pullin_sectors >= BLDC_PULLIN_SECTORS) BLDC_RampOpenLoop();

    /* Remove all gate commands and wait for the blanking interval. */
    BLDC_AllOutputsOff();
    s_blank_active = 1U;
    BLDC_SetTim2PeriodUs(BLDC_COMMUTATION_BLANK_US);
}



/* The public feedback API is intentionally inert in the open-loop image. */
void BLDC_SensorlessStart(void)
{
    BLDC_Stop();
    printf("BLDC sensorless implementation is excluded by BLDC_CONTROL_MODE\\r\\n");
}

BLDC_Fault BLDC_GetFault(void)
{
    return BLDC_FAULT_NONE;
}

uint32_t BLDC_GetMeasuredRpm(void)
{
    return 0U;
}

void BLDC_BEMF_Sample(uint16_t floating_adc, uint16_t high_adc, uint32_t time_us)
{
    (void)floating_adc;
    (void)high_adc;
    (void)time_us;
}

#endif /* BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_OPEN_LOOP_1637 */
