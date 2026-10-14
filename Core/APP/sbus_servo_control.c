#include "sbus_servo_control.h"
#include "Servo.h"

float SbusServo_TargetAngle(uint16_t ch1)
{
    if (ch1 < APP_SBUS_CH1_MIN) ch1 = APP_SBUS_CH1_MIN;
    if (ch1 > APP_SBUS_CH1_MAX) ch1 = APP_SBUS_CH1_MAX;
    float offset;
    /* Map each side of the calibrated stick midpoint to the servo midpoint. */
    if (ch1 <= APP_SBUS_CH1_MID)
        offset = -(float)(APP_SBUS_CH1_MID - ch1) / (float)(APP_SBUS_CH1_MID - APP_SBUS_CH1_MIN)
                 * (SERVO_CENTER_ANGLE - SERVO_MIN_ANGLE);
    else
        offset = (float)(ch1 - APP_SBUS_CH1_MID) / (float)(APP_SBUS_CH1_MAX - APP_SBUS_CH1_MID)
                 * (SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE);
    return SERVO_CENTER_ANGLE + APP_SBUS_SERVO_DIRECTION * offset;
}

float SbusServo_Evaluate(uint16_t ch1, uint8_t valid, uint32_t age_ms)
{
    if (!valid || age_ms > APP_SBUS_STALE_MS) return SERVO_CENTER_ANGLE;
    return SbusServo_TargetAngle(ch1);
}
