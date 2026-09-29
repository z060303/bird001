#ifndef APP_ROLL_CONTROL_H
#define APP_ROLL_CONTROL_H
#include <stdint.h>

/* Lower pulse/angle is LEFT, matching Servo_Left(). Reverse if mounted backwards. */
#define APP_ROLL_DIRECTION       (-1.0f)
#define APP_ROLL_GAIN            1.0f
#define APP_IMU_STALE_MS         40U

float RollControl_TargetAngle(float roll_degrees);
float RollControl_Evaluate(float roll_degrees, uint8_t valid, uint32_t age_ms);
#endif
