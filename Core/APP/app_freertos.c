#include "app_freertos.h"
#include "roll_control.h"
#include "sbus_servo_control.h"
#include "sbus_motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ICM42688.h"
#include "PressureSensor.h"
#include "Servo.h"
#include "SBUS.h"
#include "BLDC.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>

#if !APP_SBUS_ONLY && APP_MOTOR_TASK_ENABLED && (BLDC_CONTROL_MODE != BLDC_CONTROL_MODE_OPEN_LOOP_1637)
#error "CH3 speed control requires the existing open-loop six-step driver"
#endif

enum { PRIORITY_LOG = 1, PRIORITY_MOTOR = 1, PRIORITY_SBUS = 2, PRIORITY_PRESSURE = 2,
       PRIORITY_IMU = 3, PRIORITY_CONTROL = 4 };
enum { UART_TX_DONE = 1U, UART_TX_ERROR = 2U };
#define LOG_QUEUE_LENGTH 32U
#define UART_TIMEOUT_MS  40U
#define PRESSURE_REFERENCE_SAMPLES 50U /* 1 s at 20 ms; keep the board still. */
#define PRESSURE_REFERENCE_RANGE_PA 4.0f

typedef struct
{
    TickType_t tick;
    float roll;
    uint8_t valid;
} RollSample;

typedef struct
{
    TickType_t tick;
    uint16_t ch1;
    uint16_t ch3;
    uint8_t valid;
} SbusControlSample;

typedef struct
{
    float pressure_pa;
    float relative_height_m;
} PressureLogRecord;

typedef struct
{
    uint16_t channel[SBUS_CHANNEL_COUNT];
    uint32_t frame_count;
    uint32_t rx_byte_count, err_count;
    uint8_t digital1, digital2, frame_lost, failsafe, link_alive;
} SbusLogRecord;

typedef struct
{
    enum { LOG_PRESSURE, LOG_SBUS } type;
    union
    {
        PressureLogRecord pressure;
        SbusLogRecord sbus;
    } data;
} LogRecord;

typedef struct
{
    double sum;
    float min_pa;
    float max_pa;
    uint8_t count;
} PressureReferenceCapture;

volatile App_Diagnostics g_app_diagnostics;
static StaticQueue_t log_queue_cb;
static uint8_t log_queue_storage[LOG_QUEUE_LENGTH * sizeof(LogRecord)];
static QueueHandle_t log_queue;
static StaticTask_t sbus_cb, log_cb;
static StackType_t sbus_stack[768], log_stack[1024];
#if !APP_SBUS_ONLY
static StaticQueue_t roll_queue_cb, sbus_control_queue_cb;
static uint8_t roll_queue_storage[sizeof(RollSample)];
static uint8_t sbus_control_queue_storage[sizeof(SbusControlSample)];
static QueueHandle_t roll_queue, sbus_control_queue;
static StaticTask_t control_cb, imu_cb, pressure_cb;
static StackType_t control_stack[256], imu_stack[512], pressure_stack[512];
#if APP_MOTOR_TASK_ENABLED
static StaticTask_t motor_cb;
static StackType_t motor_stack[1024];
#endif
#endif
static TaskHandle_t log_task;

static void LogRecordEnqueue(const LogRecord *data)
{
    if (xQueueSend(log_queue, data, 0U) != pdPASS)
    {
        taskENTER_CRITICAL();
        g_app_diagnostics.log_dropped++;
        taskEXIT_CRITICAL();
    }
}

#if !APP_SBUS_ONLY
static SbusControlSample SbusControlSnapshot(const volatile SBUS_Data_t *data,
                                             TickType_t last_frame_tick, uint8_t link_alive)
{
    SbusControlSample sample = {
        .tick = last_frame_tick, .ch1 = data->channel[0], .ch3 = data->channel[2],
        .valid = link_alive && !data->failsafe && !data->frame_lost
    };
    return sample;
}

static void LogEnqueue(const PressureLogRecord *data)
{
    LogRecord record = { .type = LOG_PRESSURE, .data.pressure = *data };
    LogRecordEnqueue(&record);
}
#endif

/* Copy parsed channels here; the log task never reads mutable driver state. */
static void LogSbusSample(const volatile SBUS_Data_t *data, uint8_t new_frame,
                          uint8_t link_alive, TickType_t now,
                          TickType_t *last_log_tick, uint8_t *last_link_alive)
{
    TickType_t elapsed = now - *last_log_tick;
    uint8_t publish = link_alive
        ? new_frame && (!*last_link_alive || elapsed >= pdMS_TO_TICKS(APP_SBUS_LOG_PERIOD_MS))
        : *last_link_alive || elapsed >= pdMS_TO_TICKS(APP_SBUS_STATUS_PERIOD_MS);
    if (!publish) return;

    LogRecord record = { .type = LOG_SBUS };
    SbusLogRecord *sample = &record.data.sbus;
    sample->link_alive = link_alive;
    sample->frame_count = data->frame_count;
    sample->rx_byte_count = data->rx_byte_count;
    sample->err_count = data->err_count;
    sample->digital1 = data->digital1;
    sample->digital2 = data->digital2;
    sample->frame_lost = data->frame_lost;
    sample->failsafe = data->failsafe;
    for (uint8_t i = 0U; i < SBUS_CHANNEL_COUNT; ++i)
        sample->channel[i] = data->channel[i];
    LogRecordEnqueue(&record);
    *last_log_tick = now;
    *last_link_alive = link_alive;
}

/* Fixed release grid in normal operation; skip missed releases after a fault.
 * This prevents a failed sensor from causing a tight catch-up loop. */
static void PeriodicWait(TickType_t *release, uint32_t period_ms, volatile uint32_t *overruns)
{
    TickType_t period = pdMS_TO_TICKS(period_ms);
    if ((TickType_t)(xTaskGetTickCount() - *release) >= period)
    {
        (*overruns)++;
        *release = xTaskGetTickCount();
    }
    (void)xTaskDelayUntil(release, period);
}

#if !APP_SBUS_ONLY
static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t release = xTaskGetTickCount();
    TickType_t previous_sample = release;
    TickType_t retry_at = release;
    uint8_t ready = 0U, have_sample = 0U;
    for (;;)
    {
        RollSample sample = { .valid = 0U };
        HAL_StatusTypeDef status = HAL_ERROR;
        if (!ready && (int32_t)(xTaskGetTickCount() - retry_at) >= 0)
        {
            status = ICM42688_Init(&hi2c2);
            ready = (status == HAL_OK);
            retry_at = xTaskGetTickCount() + pdMS_TO_TICKS(1000U);
            /* Init contains power-up waits. Start the sampling grid afterwards. */
            release = xTaskGetTickCount();
            have_sample = 0U;
            if (ready)
            {
                /* Align successful startup to the common 8 ms tick grid. */
                vTaskDelay(pdMS_TO_TICKS(APP_IMU_PERIOD_MS) -
                           release % pdMS_TO_TICKS(APP_IMU_PERIOD_MS));
                release = xTaskGetTickCount();
            }
        }
        if (ready)
        {
            ICM42688_Accel_t accel;
            ICM42688_Gyro_t gyro;
            status = ICM42688_GetMotion(&accel, &gyro);
            sample.tick = xTaskGetTickCount();
            if (status == HAL_OK)
            {
                TickType_t elapsed = (TickType_t)(sample.tick - previous_sample);
                if (!have_sample || elapsed > pdMS_TO_TICKS(APP_IMU_STALE_MS))
                {
                    ICM42688_Attitude_Init();
                    elapsed = pdMS_TO_TICKS(APP_IMU_PERIOD_MS);
                }
                ICM42688_Attitude_Update(&accel, &gyro, (float)elapsed / configTICK_RATE_HZ);
                ICM42688_Attitude_t attitude;
                ICM42688_GetAttitude(&attitude);
                sample.roll = attitude.roll;
                sample.valid = isfinite(sample.roll) && isfinite(attitude.pitch) &&
                               isfinite(attitude.yaw);
                previous_sample = sample.tick;
                have_sample = sample.valid;
                if (!sample.valid) status = HAL_ERROR;
            }
        }
        sample.tick = xTaskGetTickCount();
        if (sample.valid) g_app_diagnostics.imu_samples++;
        else g_app_diagnostics.imu_errors++;
        (void)xQueueOverwrite(roll_queue, &sample);
        PeriodicWait(&release, APP_IMU_PERIOD_MS, &g_app_diagnostics.imu_overruns);
    }
}

static uint32_t SbusSampleAgeMs(const SbusControlSample *sample, TickType_t now)
{
    return (uint32_t)(((uint64_t)(TickType_t)(now - sample->tick) * 1000U) / configTICK_RATE_HZ);
}

static void ApplySbusServoSample(const SbusControlSample *sample, uint8_t have_sample,
                                 TickType_t now)
{
    uint32_t age_ms = SbusSampleAgeMs(sample, now);
    uint8_t valid = have_sample && sample->valid;
    g_app_diagnostics.servo_failsafe = !valid || age_ms > APP_SBUS_STALE_MS;
    Servo_SetAngle(SbusServo_Evaluate(sample->ch1, valid, age_ms));
}

static void ControlTask(void *argument)
{
    (void)argument;
    TickType_t release = xTaskGetTickCount();
    for (;;)
    {
        SbusControlSample sample = {0};
        uint8_t have_sample = xQueuePeek(sbus_control_queue, &sample, 0U) == pdPASS;
        ApplySbusServoSample(&sample, have_sample, xTaskGetTickCount());
        g_app_diagnostics.control_updates++;
        PeriodicWait(&release, APP_CONTROL_PERIOD_MS, &g_app_diagnostics.control_overruns);
    }
}

static PressureSensor_Status_t PreparePressureSample(const PressureSensor_Data_t *data,
                                                     PressureReferenceCapture *capture,
                                                     PressureLogRecord *record,
                                                     uint8_t *publish)
{
    *publish = 0U;
    if (!isfinite(data->Pressure_Pa) || data->Pressure_Pa <= 0.0f)
    {
        return PRESSURE_SENSOR_INVALID_PARAMETER;
    }
    if (!PressureSensor_IsReferenceValid())
    {
        /* Establish P0 from 50 periodic readings, rejecting a moving startup.
         * Double accumulation avoids losing sub-Pascal steps near 100 kPa. */
        float pressure = data->Pressure_Pa;
        if (capture->count == 0U ||
            pressure < capture->max_pa - PRESSURE_REFERENCE_RANGE_PA ||
            pressure > capture->min_pa + PRESSURE_REFERENCE_RANGE_PA)
        {
            capture->sum = pressure;
            capture->min_pa = pressure;
            capture->max_pa = pressure;
            capture->count = 1U;
            return PRESSURE_SENSOR_OK;
        }
        capture->sum += pressure;
        if (pressure < capture->min_pa) capture->min_pa = pressure;
        if (pressure > capture->max_pa) capture->max_pa = pressure;
        if (++capture->count < PRESSURE_REFERENCE_SAMPLES) return PRESSURE_SENSOR_OK;

        float reference = (float)((capture->sum - capture->min_pa - capture->max_pa) /
                                  (double)(PRESSURE_REFERENCE_SAMPLES - 2U));
        PressureSensor_Status_t status = PressureSensor_SetReferencePressure(reference);
        if (status != PRESSURE_SENSOR_OK) return status;
        record->pressure_pa = pressure;
        record->relative_height_m = 0.0f;
        *publish = 1U;
        return PRESSURE_SENSOR_OK;
    }
    record->pressure_pa = data->Pressure_Pa;
    record->relative_height_m = PressureSensor_CalculateRelativeAltitude(data->Pressure_Pa);
    *publish = 1U;
    return PRESSURE_SENSOR_OK;
}

static void PressureTask(void *argument)
{
    (void)argument;
    TickType_t release = xTaskGetTickCount(), retry_at = release;
    uint8_t ready = 0U;
    PressureReferenceCapture capture = {0};
    for (;;)
    {
        PressureSensor_Data_t data;
        PressureLogRecord record;
        uint8_t publish = 0U;
        PressureSensor_Status_t status = PRESSURE_SENSOR_ERROR;
        if (!ready && (int32_t)(xTaskGetTickCount() - retry_at) >= 0)
        {
            status = PressureSensor_Init();
            ready = (status == PRESSURE_SENSOR_OK);
            retry_at = xTaskGetTickCount() + pdMS_TO_TICKS(1000U);
            release = xTaskGetTickCount();
        }
        if (ready) status = PressureSensor_ReadPeriodic(&data);
        if (status == PRESSURE_SENSOR_OK) status = PreparePressureSample(&data, &capture, &record, &publish);
        if (status == PRESSURE_SENSOR_OK)
        {
            g_app_diagnostics.pressure_samples++;
            if (publish) LogEnqueue(&record);
        }
        else g_app_diagnostics.pressure_errors++;
        PeriodicWait(&release, APP_PRESSURE_PERIOD_MS, &g_app_diagnostics.pressure_overruns);
    }
}

#endif

/* UART4 receive runs in its ISR. This task parses complete SBUS frames and
 * lets the driver retry its receive configuration without blocking sensors. */
static void SbusTask(void *argument)
{
    (void)argument;
    SBUS_Init(&huart4);
    TickType_t release = xTaskGetTickCount();
    TickType_t last_frame_tick = release;
    TickType_t last_log_tick = release - pdMS_TO_TICKS(APP_SBUS_STATUS_PERIOD_MS);
    uint8_t last_link_alive = 0U;
    uint32_t last_frame_count = 0U;
    for (;;)
    {
        SBUS_Process();
        const volatile SBUS_Data_t *data = SBUS_GetData();
        uint32_t count = data->frame_count;
        uint8_t new_frame = count != 0U && count != last_frame_count;
        if (new_frame)
        {
            g_app_diagnostics.sbus_frames += count > last_frame_count ? count - last_frame_count : 1U;
            last_frame_tick = xTaskGetTickCount();
        }
        last_frame_count = count;
        g_app_diagnostics.sbus_updates++;
        g_app_diagnostics.sbus_failsafe = count == 0U || data->failsafe || data->frame_lost ||
            (TickType_t)(xTaskGetTickCount() - last_frame_tick) > pdMS_TO_TICKS(APP_SBUS_STALE_MS);
        TickType_t now = xTaskGetTickCount();
        uint8_t link_alive = count != 0U &&
            (TickType_t)(now - last_frame_tick) <= pdMS_TO_TICKS(APP_SBUS_STALE_MS);
#if !APP_SBUS_ONLY
        SbusControlSample control_sample = SbusControlSnapshot(data, last_frame_tick, link_alive);
        (void)xQueueOverwrite(sbus_control_queue, &control_sample);
#endif
        LogSbusSample(data, new_frame, link_alive, now, &last_log_tick, &last_link_alive);
        PeriodicWait(&release, APP_SBUS_PERIOD_MS, &g_app_diagnostics.sbus_overruns);
    }
}

#if !APP_SBUS_ONLY && APP_MOTOR_TASK_ENABLED
static void ApplySbusMotorSample(const SbusControlSample *sample, uint8_t have_sample,
                                 TickType_t now, SbusMotorControl *control)
{
    uint32_t age_ms = SbusSampleAgeMs(sample, now);
    uint8_t valid = have_sample && sample->valid && BLDC_GetRunningState() != BLDC_STATE_FAULT;
    SbusMotorCommand command = SbusMotor_Update(control, sample->ch3, valid, age_ms);
    g_app_diagnostics.motor_armed = control->armed;
    g_app_diagnostics.motor_failsafe = !valid || age_ms > APP_SBUS_MOTOR_STALE_MS;
    g_app_diagnostics.motor_target_rpm = command.nominal_rpm;
    g_app_diagnostics.motor_target_period_us = command.period_us;
    if (command.period_us == 0U)
    {
        if (BLDC_GetRunningState() != BLDC_STATE_STOPPED) BLDC_Stop();
        BLDC_SetDutyPercent(0U);
        return;
    }
    BLDC_SetCommutationPeriodUs(command.period_us);
    BLDC_SetDutyPercent(command.duty_percent);
    /* A changed throttle updates the target; it must not restart alignment/ramp. */
    if (BLDC_GetRunningState() == BLDC_STATE_STOPPED) BLDC_OpenLoopStart();
}

/* TIM2 owns commutation timing; this task consumes CH3 speed requests.
 * BLDC_Task() is a printf-only debug hook and is omitted because USART6 is
 * reserved for queued barometer and SBUS telemetry. */
static void MotorTask(void *argument)
{
    (void)argument;
    BLDC_SetDirection(MOTOR_FORWARD);
    SbusMotorControl control = {0};
    TickType_t release = xTaskGetTickCount();
    for (;;)
    {
        SbusControlSample sample = {0};
        uint8_t have_sample = xQueuePeek(sbus_control_queue, &sample, 0U) == pdPASS;
        ApplySbusMotorSample(&sample, have_sample, xTaskGetTickCount(), &control);
        g_app_diagnostics.motor_state = (uint32_t)BLDC_GetRunningState();
        g_app_diagnostics.motor_fault = (uint32_t)BLDC_GetFault();
        g_app_diagnostics.motor_step = (uint32_t)motor.step;
        g_app_diagnostics.motor_period_us = motor.commutation_period_us;
        g_app_diagnostics.motor_duty_percent = motor.duty_percent;
#if (BLDC_CONTROL_MODE == BLDC_CONTROL_MODE_OPEN_LOOP_1637)
        g_app_diagnostics.motor_irq_max_latency_us = BLDC_GetMaxTimerLatencyUs();
        g_app_diagnostics.motor_irq_late_count = BLDC_GetLateTimerCount();
#endif
        g_app_diagnostics.motor_updates++;
        PeriodicWait(&release, APP_MOTOR_PERIOD_MS, &g_app_diagnostics.motor_overruns);
    }
}
#endif

/* No floating-point printf/newlib allocation. Bound line length even after bad data. */
static void FormatFixed(char *buffer, size_t size, float value)
{
    if (!isfinite(value)) { (void)snprintf(buffer, size, "nan"); return; }
    if (value > 999999.0f) value = 999999.0f;
    if (value < -999999.0f) value = -999999.0f;
    /* Use double for decimal rounding: float loses 0.01 Pa steps near 100 kPa. */
    long scaled = (long)((double)fabsf(value) * 100.0 + 0.5);
    (void)snprintf(buffer, size, "%s%ld.%02ld", value < 0.0f ? "-" : "", scaled / 100L, scaled % 100L);
}

/* Only pressure and altitude relative to the power-on pressure are sent. */
static int FormatPressureLine(char *line, size_t size, const PressureLogRecord *data)
{
    char pressure[16], altitude[16];
    FormatFixed(pressure, sizeof(pressure), data->pressure_pa);
    FormatFixed(altitude, sizeof(altitude), data->relative_height_m);
    return snprintf(line, size, "baro:%s,%s\r\n", pressure, altitude);
}

static int FormatSbusLine(char *line, size_t size, const SbusLogRecord *data)
{
    if (!data->link_alive)
        return snprintf(line, size, "sbus:NO_SIGNAL rx=%lu err=%lu\r\n",
                        (unsigned long)data->rx_byte_count, (unsigned long)data->err_count);

    int length = snprintf(line, size, "sbus:");
    if (length < 0 || (size_t)length >= size) return -1;
    for (uint8_t i = 0U; i < SBUS_CHANNEL_COUNT; ++i)
    {
        int added = snprintf(line + length, size - (size_t)length, "CH%u=%u ",
                             (unsigned)i + 1U, (unsigned)data->channel[i]);
        if (added < 0 || (size_t)added >= size - (size_t)length) return -1;
        length += added;
    }
    int added = snprintf(line + length, size - (size_t)length,
                         "D17=%u D18=%u lost=%u fs=%u n=%lu\r\n",
                         (unsigned)data->digital1, (unsigned)data->digital2,
                         (unsigned)data->frame_lost, (unsigned)data->failsafe,
                         (unsigned long)data->frame_count);
    if (added < 0 || (size_t)added >= size - (size_t)length) return -1;
    return length + added;
}

static void UartSend(const char *buffer, size_t length)
{
    uint32_t events;
    (void)xTaskNotifyWait(0U, UINT32_MAX, &events, 0U);
    if (HAL_UART_Transmit_IT(&huart6, (const uint8_t *)buffer, (uint16_t)length) != HAL_OK ||
        xTaskNotifyWait(0U, UINT32_MAX, &events, pdMS_TO_TICKS(UART_TIMEOUT_MS)) != pdTRUE ||
        events != UART_TX_DONE)
    {
        /* Synchronous abort disables TX interrupts before this stack buffer is reused. */
        (void)HAL_UART_AbortTransmit(&huart6);
        g_app_diagnostics.uart_errors++;
    }
}

static void LogTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        LogRecord data;
        char line[256];
        int length;
        if (xQueueReceive(log_queue, &data, pdMS_TO_TICKS(100U)) != pdPASS) continue;
        length = data.type == LOG_SBUS
            ? FormatSbusLine(line, sizeof(line), &data.data.sbus)
            : FormatPressureLine(line, sizeof(line), &data.data.pressure);
        if (length > 0 && (size_t)length < sizeof(line)) UartSend(line, (size_t)length);
    }
}

/* Keep legacy printf from other modules off the telemetry UART. */
int App_LogWrite(const char *text, int length)
{
    (void)text;
    (void)length;
    return -1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART6 && log_task != NULL)
    {
        BaseType_t wake = pdFALSE;
        (void)xTaskNotifyFromISR(log_task, UART_TX_DONE, eSetBits, &wake);
        portYIELD_FROM_ISR(wake);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART6 && log_task != NULL)
    {
        BaseType_t wake = pdFALSE;
        (void)xTaskNotifyFromISR(log_task, UART_TX_ERROR, eSetBits, &wake);
        portYIELD_FROM_ISR(wake);
    }
}

void App_Start(void)
{
#if !APP_SBUS_ONLY
    Servo_Init(); /* 72.5 degrees / 1500 us before sensor startup. */
    roll_queue = xQueueCreateStatic(1U, sizeof(RollSample), roll_queue_storage, &roll_queue_cb);
    sbus_control_queue = xQueueCreateStatic(1U, sizeof(SbusControlSample),
                                          sbus_control_queue_storage, &sbus_control_queue_cb);
    configASSERT(roll_queue != NULL && sbus_control_queue != NULL);
#endif
    HAL_NVIC_SetPriority(USART6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0U);
    log_queue = xQueueCreateStatic(LOG_QUEUE_LENGTH, sizeof(LogRecord), log_queue_storage, &log_queue_cb);
    configASSERT(log_queue != NULL);
    log_task = xTaskCreateStatic(LogTask, "uart_log", 1024U, NULL, PRIORITY_LOG, log_stack, &log_cb);
    configASSERT(log_task != NULL);
#if !APP_SBUS_ONLY
    configASSERT(xTaskCreateStatic(ControlTask, "ch1_servo", 256U, NULL, PRIORITY_CONTROL,
                                   control_stack, &control_cb) != NULL);
    configASSERT(xTaskCreateStatic(ImuTask, "imu_8ms", 512U, NULL, PRIORITY_IMU, imu_stack, &imu_cb) != NULL);
    configASSERT(xTaskCreateStatic(PressureTask, "baro_20ms", 512U, NULL, PRIORITY_PRESSURE,
                                   pressure_stack, &pressure_cb) != NULL);
#endif
    configASSERT(xTaskCreateStatic(SbusTask, "sbus_5ms", 768U, NULL, PRIORITY_SBUS,
                                   sbus_stack, &sbus_cb) != NULL);
#if !APP_SBUS_ONLY && APP_MOTOR_TASK_ENABLED
    configASSERT(xTaskCreateStatic(MotorTask, "bldc_20ms", 1024U, NULL, PRIORITY_MOTOR,
                                   motor_stack, &motor_cb) != NULL);
#endif
    vTaskStartScheduler();
    App_AssertFailed(__FILE__, __LINE__);
}
