#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdint.h>

#define APP_IMU_PERIOD_MS        8U
#define APP_CONTROL_PERIOD_MS    8U
#define APP_PRESSURE_PERIOD_MS   20U

typedef struct
{
    uint32_t imu_samples, imu_errors, imu_overruns;
    uint32_t pressure_samples, pressure_errors, pressure_overruns;
    uint32_t control_updates, control_overruns, servo_failsafe;
    uint32_t log_dropped, uart_errors;
} App_Diagnostics;

/* Visible in debugger; monotonic counters, no printing from real-time tasks. */
extern volatile App_Diagnostics g_app_diagnostics;
void App_Start(void);
int App_LogWrite(const char *text, int length);

#endif
