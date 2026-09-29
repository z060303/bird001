#include "roll_control.h"
#include "Servo.h"
#include <math.h>

float RollControl_TargetAngle(float roll_degrees)
{
    if (!isfinite(roll_degrees))
    {
        return SERVO_CENTER_ANGLE;
    }
    float angle = SERVO_CENTER_ANGLE + APP_ROLL_DIRECTION * APP_ROLL_GAIN * roll_degrees;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    return angle;
}

float RollControl_Evaluate(float roll_degrees, uint8_t valid, uint32_t age_ms)
{
    if (!valid || age_ms > APP_IMU_STALE_MS) return SERVO_CENTER_ANGLE;
    return RollControl_TargetAngle(roll_degrees);
}
