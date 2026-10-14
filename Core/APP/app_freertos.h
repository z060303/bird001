#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdint.h>

/* SBUS bench mode: only UART4 reception and USART6 logging are started. */
#ifndef APP_SBUS_ONLY
#define APP_SBUS_ONLY 0U
#endif

#ifndef APP_MOTOR_TASK_ENABLED
#define APP_MOTOR_TASK_ENABLED 1U
#endif

#define APP_IMU_PERIOD_MS        8U
#define APP_CONTROL_PERIOD_MS    8U
#define APP_PRESSURE_PERIOD_MS   20U
#define APP_SBUS_PERIOD_MS       5U
#define APP_SBUS_LOG_PERIOD_MS   50U
#define APP_SBUS_STATUS_PERIOD_MS 1000U
#define APP_MOTOR_PERIOD_MS      20U

/* Motor task requires APP_SBUS_ONLY=0. CH3 owns startup and stop; no autostart. */

typedef struct
{
    uint32_t imu_samples, imu_errors, imu_overruns;
    uint32_t pressure_samples, pressure_errors, pressure_overruns;
    uint32_t control_updates, control_overruns, servo_failsafe;
    uint32_t log_dropped, uart_errors;
    uint32_t sbus_updates, sbus_frames, sbus_overruns;
    uint32_t motor_updates, motor_overruns;
    uint32_t sbus_failsafe, motor_state, motor_fault;
    uint32_t motor_step, motor_period_us, motor_duty_percent;
    uint32_t motor_irq_max_latency_us, motor_irq_late_count;
    uint32_t motor_target_rpm, motor_target_period_us, motor_armed, motor_failsafe;
} App_Diagnostics;

/* Visible in debugger; monotonic counters, no printing from real-time tasks. */
extern volatile App_Diagnostics g_app_diagnostics;
void App_Start(void);
int App_LogWrite(const char *text, int length);

#endif
