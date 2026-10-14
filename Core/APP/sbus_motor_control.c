#include "sbus_motor_control.h"

_Static_assert(APP_SBUS_CH3_MAX > APP_SBUS_CH3_MIN + APP_SBUS_CH3_STOP_BAND + 1U,
               "CH3 running range must have positive width");
_Static_assert(APP_SBUS_MOTOR_FAST_PERIOD_US > 0U &&
               APP_SBUS_MOTOR_SLOW_PERIOD_US >= APP_SBUS_MOTOR_FAST_PERIOD_US,
               "Motor speed periods must be positive and ordered");

SbusMotorCommand SbusMotor_Update(SbusMotorControl *control, uint16_t ch3,
                                  uint8_t valid, uint32_t age_ms)
{
    SbusMotorCommand command = {0};
    if (!valid || age_ms > APP_SBUS_MOTOR_STALE_MS)
    {
        control->armed = 0U;
        return command;
    }
    if (ch3 <= APP_SBUS_CH3_MIN + APP_SBUS_CH3_STOP_BAND)
    {
        control->armed = 1U;
        return command;
    }
    if (!control->armed) return command;
    if (ch3 > APP_SBUS_CH3_MAX) ch3 = APP_SBUS_CH3_MAX;

    const uint32_t first_run = APP_SBUS_CH3_MIN + APP_SBUS_CH3_STOP_BAND + 1U;
    const uint32_t span = APP_SBUS_CH3_MAX - first_run;
    const uint32_t slow = APP_SBUS_MOTOR_SLOW_PERIOD_US;
    const uint32_t fast = APP_SBUS_MOTOR_FAST_PERIOD_US;
    /* Interpolate reciprocal period so nominal speed, rather than period, is linear. */
    uint64_t denominator = (uint64_t)fast * span + (uint64_t)(slow - fast) * (ch3 - first_run);
    command.period_us = (uint32_t)(((uint64_t)slow * fast * span + denominator / 2U) / denominator);
    command.nominal_rpm = (10000000U + BLDC_MOTOR_POLE_PAIRS * command.period_us / 2U)
                         / (BLDC_MOTOR_POLE_PAIRS * command.period_us);
    command.duty_percent = BLDC_TARGET_DUTY_PERCENT;
    return command;
}
