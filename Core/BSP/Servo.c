#include "Servo.h"
#include "tim.h"
#include <math.h>

/* 当前舵机角度，仅用于调试读取 */
static float servo_current_angle = SERVO_CENTER_ANGLE;

static uint32_t Servo_ClampPulseUs(uint32_t pulse_us)
{
    if (pulse_us < (uint32_t)SERVO_MIN_PULSE_US)
    {
        pulse_us = (uint32_t)SERVO_MIN_PULSE_US;
    }

    if (pulse_us > (uint32_t)SERVO_MAX_PULSE_US)
    {
        pulse_us = (uint32_t)SERVO_MAX_PULSE_US;
    }

    return pulse_us;
}

/**
 * @brief 初始化舵机 PWM。
 *
 * PB8 使用 TIM4_CH3 输出标准舵机 PWM：
 *   - PWM 频率：50Hz
 *   - 周期：20ms
 *   - 计数分辨率：1us / tick
 */
void Servo_Init(void)
{
    /* 先写入居中脉宽，再启动 PWM，避免上电后输出 0 占空比 */
    Servo_Center();
    /* Latch the preloaded 1500 us compare before the very first PWM period. */
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 直接设置舵机脉宽，单位 us。
 *
 * @param pulse_us 测试范围 500~2500us，函数内部会限幅。
 */
void Servo_SetPulseUs(uint32_t pulse_us)
{
    pulse_us = Servo_ClampPulseUs(pulse_us);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_us);
}

/**
 * @brief 设置舵机角度。
 *
 * @param angle 角度范围 0~145 度。
 */
void Servo_SetAngle(float angle)
{
    uint32_t pulse;

    if (!isfinite(angle)) angle = SERVO_CENTER_ANGLE;

    if (angle < SERVO_MIN_ANGLE)
    {
        angle = SERVO_MIN_ANGLE;
    }

    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    /* 0度=500us，72.5度=1500us，145度=2500us */
    pulse = (uint32_t)(SERVO_MIN_PULSE_US +
                       ((angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) *
                       (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + 0.5f);

    Servo_SetPulseUs(pulse);
    servo_current_angle = angle;
}

/**
 * @brief 获取当前设置的舵机角度。
 */
float Servo_GetAngle(void)
{
    return servo_current_angle;
}

/**
 * @brief 舵机回中，72.5 度。
 */
void Servo_Center(void)
{
    Servo_SetAngle(SERVO_CENTER_ANGLE);
}

/**
 * @brief 舵机转到 0 度。
 */
void Servo_Left(void)
{
    Servo_SetAngle(SERVO_MIN_ANGLE);
}

/**
 * @brief 舵机转到 145 度。
 */
void Servo_Right(void)
{
    Servo_SetAngle(SERVO_MAX_ANGLE);
}
