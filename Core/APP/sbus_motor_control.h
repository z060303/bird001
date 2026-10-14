#ifndef APP_SBUS_MOTOR_CONTROL_H
#define APP_SBUS_MOTOR_CONTROL_H
#include <stdint.h>
#include "BLDC.h"

#define APP_SBUS_CH3_MIN 312U
#define APP_SBUS_CH3_MAX 1912U
#define APP_SBUS_CH3_STOP_BAND 20U
#define APP_SBUS_MOTOR_STALE_MS 100U
#define APP_SBUS_MOTOR_SLOW_PERIOD_US BLDC_START_PERIOD_US
#define APP_SBUS_MOTOR_FAST_PERIOD_US BLDC_TARGET_PERIOD_US

typedef struct { uint8_t armed; } SbusMotorControl;
typedef struct
{
    uint32_t period_us;
    uint32_t nominal_rpm;
    uint8_t duty_percent;
} SbusMotorCommand;

/* Low throttle arms; invalid/stale reception disarms until low throttle returns. */
SbusMotorCommand SbusMotor_Update(SbusMotorControl *control, uint16_t ch3,
                                  uint8_t valid, uint32_t age_ms);
#endif
