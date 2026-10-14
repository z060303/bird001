/* Exercise the actual open-loop driver with mocked HAL timer boundaries.
 * The emulator checks the six-step schedule, not motor electromechanics. */
#include "BLDC.h"
#include "tim.h"
#include "sbus_motor_control.h"

TIM_HandleTypeDef htim1 = { .Instance = TIM1 };
TIM_HandleTypeDef htim2 = { .Instance = TIM2 };
static unsigned errors;

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000UL; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config, uint32_t *latency)
{
    config->APB1CLKDivider = RCC_HCLK_DIV4;
    *latency = 5U;
}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer)
{
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
    timer->Instance->DIER |= TIM_IT_UPDATE;
    return HAL_TIM_Base_Start(timer);
}
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{
    timer->Instance->DIER &= ~TIM_IT_UPDATE;
    timer->Instance->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}
void Error_Handler(void) { errors++; }
int printf(const char *format, ...) { (void)format; return 0; }

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static void NextTimerEvent(void)
{
    TIM2->CNT = 0U;
    BLDC_TIM_PeriodElapsedCallback(&htim2);
}

int test_main(void)
{
    TIM1->ARR = BLDC_TIM1_PERIOD;
    TIM2->ARR = 0xFFFFFFFFUL;
    BLDC_Init();
    CHECK(errors == 0U && motor.state == BLDC_STATE_STOPPED);
    CHECK(TIM2->PSC == 83U);

    BLDC_OpenLoopStart();
    CHECK(motor.state == BLDC_STATE_BOOTSTRAP_PRECHARGE);
    CHECK(TIM2->ARR + 1UL == BLDC_BOOTSTRAP_PRECHARGE_US);
    TIM2->CNT = BLDC_COMMUTATION_BLANK_US + 1UL;
    BLDC_TIM_PeriodElapsedCallback(&htim2);
    CHECK(motor.state == BLDC_STATE_ALIGNING);
    CHECK(BLDC_GetMaxTimerLatencyUs() == BLDC_COMMUTATION_BLANK_US + 1UL);
    CHECK(BLDC_GetLateTimerCount() == 1UL);
    for (unsigned i = 0U; i < (BLDC_ALIGNMENT_RAMP_US + BLDC_ALIGNMENT_HOLD_US) /
                              BLDC_ALIGNMENT_UPDATE_US; ++i) NextTimerEvent();
    CHECK(motor.state == BLDC_STATE_OPEN_LOOP_STARTING);
    CHECK(TIM2->ARR + 1UL == BLDC_COMMUTATION_BLANK_US);

    /* The blank-end callback must switch exactly one step without forcing
     * TIM1 UG, which would reset the 20 kHz PWM phase. */
    TIM1->EGR = 0U;
    TIM1->CNT = 1234U;
    NextTimerEvent();
    CHECK(motor.step == 1U && TIM1->EGR == 0U && TIM1->CNT == 1234U);
    CHECK(TIM2->ARR + 1UL == BLDC_START_PERIOD_US - BLDC_COMMUTATION_BLANK_US);

    for (unsigned i = 0U; i < 2000U && motor.state != BLDC_STATE_OPEN_LOOP_RUNNING; ++i)
    {
        uint8_t previous = motor.step;
        NextTimerEvent();
        CHECK(TIM2->ARR + 1UL == BLDC_COMMUTATION_BLANK_US);
        TIM1->EGR = 0U;
        TIM1->CNT = 1234U;
        NextTimerEvent();
        CHECK(motor.step == (uint8_t)((previous + 1U) % 6U));
        CHECK(TIM1->EGR == 0U && TIM1->CNT == 1234U);
        CHECK(TIM2->ARR + 1UL == motor.commutation_period_us - BLDC_COMMUTATION_BLANK_US);
        CHECK(motor.duty <= BLDC_TIM1_PERIOD / 5U + 1U);
    }
    CHECK(motor.state == BLDC_STATE_OPEN_LOOP_RUNNING);
    CHECK(motor.commutation_period_us == BLDC_TARGET_PERIOD_US);
    SbusMotorControl control = {0};
    (void)SbusMotor_Update(&control, 312U, 1U, 0U);
    SbusMotorCommand command = SbusMotor_Update(&control, 333U, 1U, 0U);
    BLDC_SetCommutationPeriodUs(command.period_us);
    BLDC_SetDutyPercent(command.duty_percent);
    for (unsigned i = 0U; i < 3000U && motor.commutation_period_us != command.period_us; ++i)
    {
        NextTimerEvent(); NextTimerEvent();
    }
    CHECK(motor.commutation_period_us == BLDC_START_PERIOD_US);
    command = SbusMotor_Update(&control, 1912U, 1U, 0U);
    BLDC_SetCommutationPeriodUs(command.period_us);
    for (unsigned i = 0U; i < 3000U && motor.commutation_period_us != command.period_us; ++i)
    {
        NextTimerEvent(); NextTimerEvent();
    }
    CHECK(motor.commutation_period_us == BLDC_TARGET_PERIOD_US);
    command = SbusMotor_Update(&control, 1912U, 0U, 0U);
    BLDC_SetDutyPercent(command.duty_percent);
    CHECK(!control.armed && motor.state == BLDC_STATE_STOPPED && (TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    BLDC_EmergencyStop();
    CHECK(motor.state == BLDC_STATE_STOPPED && (TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    return 0;
}
