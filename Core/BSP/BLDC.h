#ifndef __BLDC_H
#define __BLDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * Hardware mapping for the current STM32F405 board:
 *
 * Motor phase A -> EG2133 HIN1/LIN1 -> PA10/PB15 -> TIM1_CH3/CH3N
 * Motor phase B -> EG2133 HIN2/LIN2 -> PA9 /PB14 -> TIM1_CH2/CH2N
 * Motor phase C -> EG2133 HIN3/LIN3 -> PA8 /PB13 -> TIM1_CH1/CH1N
 *
 * EG2133:
 *   HIN = high-side input, active high
 *   LIN = low-side input, active low
 *   HIN=1, LIN=1 -> high MOS ON
 *   HIN=0, LIN=0 -> low MOS ON
 *   HIN=0, LIN=1 -> both OFF
 *
 * STM32 TIM1 is used only as a waveform generator. The code does NOT
 * enable CHx and CHxN simultaneously. This avoids relying on STM32
 * complementary polarity for the EG2133 LIN input.
 */

typedef enum
{
    BLDC_PHASE_A = 0,
    BLDC_PHASE_B,
    BLDC_PHASE_C,
    BLDC_PHASE_NONE
} BLDC_Phase;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} Motor_Direction;

typedef enum
{
    BLDC_STATE_STOPPED = 0,
    BLDC_STATE_PWM_TEST,
    BLDC_STATE_LOW_SIDE_TEST,
    BLDC_STATE_OPEN_LOOP_STARTING,
    BLDC_STATE_OPEN_LOOP_RUNNING,
    BLDC_STATE_FAULT,
    BLDC_STATE_BOOTSTRAP_PRECHARGE,
    BLDC_STATE_ALIGNING,
    BLDC_STATE_CLOSED_LOOP_RUNNING
} BLDC_RunningState;

typedef enum
{
    BLDC_FAULT_NONE = 0,
    BLDC_FAULT_BEMF_HARDWARE, /* Divider/filter have not been verified. */
    BLDC_FAULT_BEMF_STARTUP,  /* No reliable rotor synchronisation. */
    BLDC_FAULT_BEMF_LOST,     /* Missing/implausible zero crossing. */
    BLDC_FAULT_ADC_RANGE,     /* Clipped or invalid PWM-on reference. */
    BLDC_FAULT_OVERSPEED,
    BLDC_FAULT_TIMING
} BLDC_Fault;

typedef struct
{
    uint16_t duty;
    uint16_t max_duty;
    uint8_t duty_percent;
    uint8_t step;
    uint32_t commutation_period_us;
    Motor_Direction dir;
    BLDC_RunningState state;
} BLDC_Handle;

/*
 * Select exactly one motor-control implementation at compile time.
 *
 * OPEN_LOOP_1637 uses six-step commutation with precharge, alignment and
 * a bounded open-loop speed ramp:
 * TIM1 + TIM2 only; it neither initializes nor reads ADC1/ADC2/TIM5.
 * SENSORLESS_BEMF enables the newer ADC1/ADC2 back-EMF implementation.
 * It owns ADC1, ADC2, TIM5 and TIM1_CC4.
 */
#define BLDC_CONTROL_MODE_OPEN_LOOP_1637  (1U)
#define BLDC_CONTROL_MODE_SENSORLESS_BEMF (2U)
#ifndef BLDC_CONTROL_MODE
#define BLDC_CONTROL_MODE BLDC_CONTROL_MODE_OPEN_LOOP_1637
#endif

#if ((BLDC_CONTROL_MODE != BLDC_CONTROL_MODE_OPEN_LOOP_1637) && \
     (BLDC_CONTROL_MODE != BLDC_CONTROL_MODE_SENSORLESS_BEMF))
#error "BLDC_CONTROL_MODE must select OPEN_LOOP_1637 or SENSORLESS_BEMF"
#endif

#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_OPEN_LOOP_1637)
#define BLDC_CONTROL_MODE_NAME "open-loop-six-step"
#else
#define BLDC_CONTROL_MODE_NAME "sensorless-bemf"
#endif

/* Expected TIM1 configuration from CubeMX.
 * TIM1 clock = 168 MHz, PSC = 0, ARR = 8399 -> 20 kHz.
 * The driver also prints the actual timer values at runtime.
 */
#define BLDC_TIM1_EXPECTED_CLOCK_HZ       (168000000UL)
#define BLDC_PWM_FREQUENCY_HZ             (20000UL)
#define BLDC_TIM1_PRESCALER               (0U)
#define BLDC_TIM1_PERIOD                  (8399U)

/* Initial unloaded bench settings, not a current limit or a motor rating. */
#define BLDC_MAX_RUN_DUTY_PERCENT         (20U)
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_OPEN_LOOP_1637)
/* Conservative, adjustable open-loop pull-in. A sector is 60 electrical degrees. */
#define BLDC_START_DUTY_PERCENT           (12U)
#define BLDC_TARGET_DUTY_PERCENT          (16U)
#define BLDC_ALIGN_DUTY_PERCENT           (10U)
#define BLDC_PULLIN_SECTORS               (6U)
#define BLDC_START_PERIOD_US              (20000UL)
#define BLDC_TARGET_PERIOD_US             (3000UL)
#define BLDC_ACCEL_RPM_PER_S              (250UL)
#define BLDC_DECEL_RPM_PER_S              (350UL)
#define BLDC_DUTY_RAMP_INTERVAL_US        (100000UL)
#else
#define BLDC_START_DUTY_PERCENT           (6U)
#define BLDC_TARGET_DUTY_PERCENT          (12U)
#define BLDC_START_PERIOD_US              (20000UL)
#define BLDC_TARGET_PERIOD_US             (750UL)
#endif

#define BLDC_MOTOR_POLE_PAIRS             (6UL) /* EMAX ECO 1404: 9N12P */

/* Open-loop request limits; sensorless startup uses its own ramp. */
#define BLDC_MIN_PERIOD_US                (500UL)
#define BLDC_MAX_PERIOD_US                (100000UL)
#define BLDC_RAMP_DURATION_MS             (50000UL)

#define BLDC_BOOTSTRAP_PRECHARGE_US        (2000UL)
#define BLDC_ALIGNMENT_RAMP_US            (100000UL)
#define BLDC_ALIGNMENT_HOLD_US            (200000UL)
#define BLDC_ALIGNMENT_UPDATE_US          (1000UL)
#define BLDC_TIM2_COUNTER_HZ              (1000000UL)

/* Blank time between disabling the old state and enabling the new state. */
#define BLDC_COMMUTATION_BLANK_US         (5UL)

/* PA7=A, PA6=B, PA5=C. Used only in SENSORLESS_BEMF mode. All three dividers must have the same ratio.
 * REQUIRED before setting READY=1: upper 10k, lower 4.7k (1%), C <=100pF
 * or no capacitor, with 2S <=8.4V and VDDA=3.3V verified on the board.
 * 100nF and the original 133k/133k divider cannot be used with this sampler.
 * This flag is a manual hardware acknowledgement, not an ADC safety check. */
#ifndef BLDC_BEMF_HARDWARE_READY
#define BLDC_BEMF_HARDWARE_READY           (0U)
#endif
#define BLDC_BEMF_STARTUP_RAMP_MS          (6000UL)
#define BLDC_BEMF_STARTUP_TIMEOUT_MS       (10000UL)
#define BLDC_BEMF_HANDOFF_CROSSINGS        (12U)
#define BLDC_BEMF_HANDOFF_MAX_PERIOD_US    (4000UL)
#define BLDC_BEMF_MIN_PERIOD_US            (150UL)
#define BLDC_BEMF_MAX_PERIOD_US            (20000UL)
#define BLDC_BEMF_DEMAG_MAX_US             (150UL)
#define BLDC_BEMF_HYSTERESIS               (24) /* 2*float_adc - high_adc */
#define BLDC_BEMF_CONFIRM_SAMPLES          (2U)
#define BLDC_BEMF_DUTY_RAMP_MS             (1000UL)

extern volatile BLDC_Handle motor;

void BLDC_Init(void);
void BLDC_Task(void);

void BLDC_SetDuty(uint16_t duty);
void BLDC_SetDutyPercent(uint8_t percent);
/* In open-loop mode duty requests slew by at most 1% per 100ms; zero stops.
 * Sensorless mode uses its own 1s duty slew and 6% ADC sampling minimum.
 * Period requests apply only to open-loop mode and set the electrical speed
 * target, not a measured mechanical speed. */
void BLDC_SetSpeed(uint16_t duty);
void BLDC_SetDirection(Motor_Direction dir);
void BLDC_SetCommutationPeriodUs(uint32_t period_us);

void BLDC_Start(void);
void BLDC_OpenLoopStart(void);
void BLDC_SensorlessStart(void);
void BLDC_Stop(void);
void BLDC_EmergencyStop(void);
void BLDC_Commutation(uint8_t step);

uint8_t BLDC_GetStep(void);
uint16_t BLDC_GetDuty(void);
uint8_t BLDC_GetDutyPercent(void);
BLDC_RunningState BLDC_GetRunningState(void);
BLDC_Fault BLDC_GetFault(void);
uint32_t BLDC_GetMeasuredRpm(void);

/* Called only by the synchronized ADC1/ADC2 injected-conversion ISR. */
void BLDC_BEMF_Sample(uint16_t floating_adc, uint16_t high_adc, uint32_t time_us);

/* Safe bench tests. Do NOT connect a motor until the MCU-side waveforms
 * and EG2133 HIN/LIN waveforms have been verified with an oscilloscope.
 */
void BLDC_TestHighPWM(BLDC_Phase phase, uint8_t duty_percent);
void BLDC_TestLowSide(BLDC_Phase phase);
void BLDC_TestStep(uint8_t step, uint8_t duty_percent);

/* Call this from HAL_TIM_PeriodElapsedCallback(). */
void BLDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_H */
