#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================
 * 标准舵机 PWM 参数
 *
 * PB8 -> TIM4_CH3
 * PWM 频率 = 50Hz
 * 周期 = 20ms
 * TIM4 计数频率 = 1MHz，即 1 tick = 1us
 *
 * 舵机已确认机械范围及脉宽：
 *   0度    -> 0.5ms = 500us
 *   72.5度 -> 1.5ms = 1500us
 *   145度  -> 2.5ms = 2500us
 * ============================================================ */

#define SERVO_MIN_ANGLE        0.0f
#define SERVO_MAX_ANGLE        145.0f
#define SERVO_CENTER_ANGLE     ((SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) * 0.5f)

#define SERVO_MIN_PULSE_US     500.0f
#define SERVO_MAX_PULSE_US     2500.0f

void Servo_Init(void);
void Servo_SetPulseUs(uint32_t pulse_us);
void Servo_SetAngle(float angle);
float Servo_GetAngle(void);
void Servo_Center(void);
void Servo_Left(void);
void Servo_Right(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
