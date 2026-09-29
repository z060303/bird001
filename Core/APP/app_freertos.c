#include "app_freertos.h"
#include "roll_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ICM42688.h"
#include "PressureSensor.h"
#include "Servo.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { PRIORITY_LOG = 1, PRIORITY_PRESSURE, PRIORITY_IMU, PRIORITY_CONTROL };
enum { LOG_IMU, LOG_PRESSURE, LOG_TEXT };
enum { UART_TX_DONE = 1U, UART_TX_ERROR = 2U };
#define LOG_QUEUE_LENGTH 32U
#define LOG_TEXT_LENGTH  80U
#define UART_TIMEOUT_MS  20U

typedef struct
{
    TickType_t tick;
    float roll;
    uint8_t valid;
} RollSample;

typedef struct
{
    uint32_t tick;
    uint32_t status;
    uint8_t type;
    union
    {
        ICM42688_Attitude_t imu;
        PressureSensor_Data_t pressure;
        char text[LOG_TEXT_LENGTH + 1U];
    } value;
} LogRecord;

volatile App_Diagnostics g_app_diagnostics;
static StaticQueue_t roll_queue_cb, log_queue_cb;
static uint8_t roll_queue_storage[sizeof(RollSample)];
static uint8_t log_queue_storage[LOG_QUEUE_LENGTH * sizeof(LogRecord)];
static QueueHandle_t roll_queue, log_queue;
static StaticTask_t control_cb, imu_cb, pressure_cb, log_cb;
static StackType_t control_stack[256], imu_stack[512], pressure_stack[512], log_stack[1024];
static TaskHandle_t log_task;

static void LogEnqueue(const LogRecord *record)
{
    if (xQueueSend(log_queue, record, 0U) != pdPASS)
    {
        taskENTER_CRITICAL();
        g_app_diagnostics.log_dropped++;
        taskEXIT_CRITICAL();
    }
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

static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t release = xTaskGetTickCount();
    TickType_t previous_sample = release;
    TickType_t retry_at = release;
    uint8_t ready = 0U, have_sample = 0U;
    for (;;)
    {
        LogRecord log = { .type = LOG_IMU };
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
                ICM42688_GetAttitude(&log.value.imu);
                sample.roll = log.value.imu.roll;
                sample.valid = isfinite(sample.roll) && isfinite(log.value.imu.pitch) &&
                               isfinite(log.value.imu.yaw);
                previous_sample = sample.tick;
                have_sample = sample.valid;
                if (!sample.valid) status = HAL_ERROR;
            }
        }
        sample.tick = xTaskGetTickCount();
        log.tick = sample.tick;
        log.status = (uint32_t)status;
        if (sample.valid) g_app_diagnostics.imu_samples++;
        else g_app_diagnostics.imu_errors++;
        (void)xQueueOverwrite(roll_queue, &sample);
        LogEnqueue(&log);
        PeriodicWait(&release, APP_IMU_PERIOD_MS, &g_app_diagnostics.imu_overruns);
    }
}

static void ControlTask(void *argument)
{
    (void)argument;
    /* Nominal I2C2 burst is ~1.4 ms at 100 kHz. Release control 3 ms after IMU. */
    vTaskDelay(pdMS_TO_TICKS(3U));
    TickType_t release = xTaskGetTickCount();
    for (;;)
    {
        RollSample sample = {0};
        uint8_t valid = xQueuePeek(roll_queue, &sample, 0U) == pdPASS;
        uint32_t age = (TickType_t)(xTaskGetTickCount() - sample.tick);
        valid = valid && sample.valid && isfinite(sample.roll);
        float target = RollControl_Evaluate(sample.roll, valid, age);
        g_app_diagnostics.servo_failsafe = !valid || age > APP_IMU_STALE_MS;
        Servo_SetAngle(target);
        g_app_diagnostics.control_updates++;
        PeriodicWait(&release, APP_CONTROL_PERIOD_MS, &g_app_diagnostics.control_overruns);
    }
}

static void PressureTask(void *argument)
{
    (void)argument;
    TickType_t release = xTaskGetTickCount(), retry_at = release;
    uint8_t ready = 0U;
    for (;;)
    {
        LogRecord log = { .type = LOG_PRESSURE };
        PressureSensor_Status_t status = PRESSURE_SENSOR_ERROR;
        if (!ready && (int32_t)(xTaskGetTickCount() - retry_at) >= 0)
        {
            status = PressureSensor_Init();
            ready = (status == PRESSURE_SENSOR_OK);
            retry_at = xTaskGetTickCount() + pdMS_TO_TICKS(1000U);
            release = xTaskGetTickCount();
        }
        if (ready) status = PressureSensor_ReadPeriodic(&log.value.pressure);
        log.tick = xTaskGetTickCount();
        log.status = (uint32_t)status;
        if (status == PRESSURE_SENSOR_OK) g_app_diagnostics.pressure_samples++;
        else g_app_diagnostics.pressure_errors++;
        LogEnqueue(&log);
        PeriodicWait(&release, APP_PRESSURE_PERIOD_MS, &g_app_diagnostics.pressure_overruns);
    }
}

/* No floating-point printf/newlib allocation. Bound line length even after bad data. */
static void FormatFixed(char *buffer, size_t size, float value)
{
    if (!isfinite(value)) { (void)snprintf(buffer, size, "nan"); return; }
    if (value > 999999.0f) value = 999999.0f;
    if (value < -999999.0f) value = -999999.0f;
    long scaled = (long)(fabsf(value) * 100.0f + 0.5f);
    (void)snprintf(buffer, size, "%s%ld.%02ld", value < 0.0f ? "-" : "", scaled / 100L, scaled % 100L);
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
    static const char banner[] = "FreeRTOS: I,t_ms,roll_deg,pitch_deg,yaw_deg; P,t_ms,Pa,C,alt_m\r\n";
    UartSend(banner, sizeof(banner) - 1U);
    for (;;)
    {
        LogRecord log;
        char line[128], a[16], b[16], c[16];
        int length;
        if (xQueueReceive(log_queue, &log, pdMS_TO_TICKS(100U)) != pdPASS) continue;
        if (log.type == LOG_TEXT)
        {
            UartSend(log.value.text, strlen(log.value.text));
            continue;
        }
        if (log.status != 0U)
        {
            length = snprintf(line, sizeof(line), "%c,%lu,ERR=%lu\r\n",
                              log.type == LOG_IMU ? 'I' : 'P', (unsigned long)log.tick,
                              (unsigned long)log.status);
        }
        else
        {
            if (log.type == LOG_IMU)
            {
                FormatFixed(a, sizeof(a), log.value.imu.roll);
                FormatFixed(b, sizeof(b), log.value.imu.pitch);
                FormatFixed(c, sizeof(c), log.value.imu.yaw);
            }
            else
            {
                FormatFixed(a, sizeof(a), log.value.pressure.Pressure_Pa);
                FormatFixed(b, sizeof(b), log.value.pressure.Temperature_C);
                FormatFixed(c, sizeof(c), log.value.pressure.Altitude);
            }
            length = snprintf(line, sizeof(line), "%c,%lu,%s,%s,%s\r\n",
                              log.type == LOG_IMU ? 'I' : 'P', (unsigned long)log.tick, a, b, c);
        }
        if (length > 0 && (size_t)length < sizeof(line)) UartSend(line, (size_t)length);
    }
}

/* Best-effort compatibility path. Periodic tasks use complete typed records instead. */
int App_LogWrite(const char *text, int length)
{
    if (text == NULL || length < 0 || log_queue == NULL || __get_IPSR() != 0U) return -1;
    int written = 0;
    while (written < length)
    {
        LogRecord log = { .type = LOG_TEXT };
        size_t count = (size_t)(length - written);
        if (count > LOG_TEXT_LENGTH) count = LOG_TEXT_LENGTH;
        memcpy(log.value.text, text + written, count);
        LogEnqueue(&log);
        written += (int)count;
    }
    return written;
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
    Servo_Init(); /* 72.5 degrees / 1500 us before sensor startup. */
    HAL_NVIC_SetPriority(USART6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0U);
    roll_queue = xQueueCreateStatic(1U, sizeof(RollSample), roll_queue_storage, &roll_queue_cb);
    log_queue = xQueueCreateStatic(LOG_QUEUE_LENGTH, sizeof(LogRecord), log_queue_storage, &log_queue_cb);
    configASSERT(roll_queue != NULL && log_queue != NULL);
    log_task = xTaskCreateStatic(LogTask, "uart_log", 1024U, NULL, PRIORITY_LOG, log_stack, &log_cb);
    configASSERT(log_task != NULL);
    configASSERT(xTaskCreateStatic(ControlTask, "roll_servo", 256U, NULL, PRIORITY_CONTROL,
                                   control_stack, &control_cb) != NULL);
    configASSERT(xTaskCreateStatic(ImuTask, "imu_8ms", 512U, NULL, PRIORITY_IMU, imu_stack, &imu_cb) != NULL);
    configASSERT(xTaskCreateStatic(PressureTask, "baro_20ms", 512U, NULL, PRIORITY_PRESSURE,
                                   pressure_stack, &pressure_cb) != NULL);
    vTaskStartScheduler();
    App_AssertFailed(__FILE__, __LINE__);
}
