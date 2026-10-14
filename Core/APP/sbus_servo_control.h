#ifndef APP_SBUS_SERVO_CONTROL_H
#define APP_SBUS_SERVO_CONTROL_H
#include <stdint.h>

/* Low CH1 -> left, high CH1 -> right. Change to -1 if mounted backwards. */
#define APP_SBUS_SERVO_DIRECTION 1
#define APP_SBUS_STALE_MS 100U
#define APP_SBUS_CH1_MIN 544U
#define APP_SBUS_CH1_MID 942U
#define APP_SBUS_CH1_MAX 1340U

float SbusServo_TargetAngle(uint16_t ch1);
float SbusServo_Evaluate(uint16_t ch1, uint8_t valid, uint32_t age_ms);
#endif
