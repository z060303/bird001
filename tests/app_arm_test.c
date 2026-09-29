/* Execute production functions with mocked HAL/RTOS boundaries on ARM emulation.
 * Includes give access to private periodic/log helpers without a production test API. */
#include "../Core/APP/app_freertos.c"
#include "../Core/BSP/PressureSensor.c"
#include "../Core/BSP/ICM42688.c"
#include <sys/stat.h>

/* Minimal libc host boundaries: tests have no OS/file descriptors. */
void *_sbrk(ptrdiff_t increment)
{
    static char heap[4096];
    static ptrdiff_t used;
    if (increment < 0 || used + increment > (ptrdiff_t)sizeof(heap)) return (void *)-1;
    void *result = heap + used;
    used += increment;
    return result;
}
int _write(int fd, const void *data, size_t length) { (void)fd; (void)data; return (int)length; }
int _read(int fd, void *data, size_t length) { (void)fd; (void)data; (void)length; return 0; }
int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, struct stat *s) { (void)fd; s->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart6;
I2C_HandleTypeDef hi2c1, hi2c2;
static TickType_t fake_tick, delay_sum, last_wait;
static BaseType_t scheduler_state = taskSCHEDULER_RUNNING;
static uint32_t hal_delay_sum, transmit_calls, abort_calls, notify_events = UART_TX_DONE;
static BaseType_t notify_result = pdTRUE, queue_result = pdPASS;
static HAL_StatusTypeDef uart_result = HAL_OK, i2c_result = HAL_OK;
static uint32_t queue_calls, critical_depth, read_calls, recover_calls;
static uint8_t current_command, conversion_command, commands[16], command_count;
static uint8_t raw_motion[12] = {0x20,0, 0xE0,0, 0x40,0, 0x02,0x8F, 0xFD,0x71, 0,0};

TickType_t xTaskGetTickCount(void) { return fake_tick; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler_state; }
BaseType_t xTaskDelayUntil(TickType_t *release, TickType_t increment)
{
    *release += increment;
    last_wait = increment;
    fake_tick = *release;
    return pdTRUE;
}
void vTaskDelay(TickType_t ticks) { delay_sum += ticks; fake_tick += ticks; }
void HAL_Delay(uint32_t delay) { hal_delay_sum += delay; fake_tick += delay; }
uint32_t HAL_GetTick(void) { return fake_tick; }
void vPortEnterCritical(void) { critical_depth++; }
void vPortExitCritical(void) { critical_depth--; }
void App_AssertFailed(const char *file, int line) { (void)file; (void)line; for (;;) {} }
void Error_Handler(void) { for (;;) {} }

BaseType_t xQueueGenericSend(QueueHandle_t queue, const void * const item, TickType_t wait,
                             const BaseType_t position)
{
    (void)queue; (void)item; (void)position;
    queue_calls++;
    last_wait = wait;
    return queue_result;
}
BaseType_t xTaskGenericNotifyWait(UBaseType_t index, uint32_t entry, uint32_t exit_bits,
                                  uint32_t *events, TickType_t wait)
{
    (void)index; (void)entry; (void)exit_bits;
    last_wait = wait;
    *events = wait ? notify_events : 0U;
    return wait ? notify_result : pdFALSE;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size)
{
    (void)uart; (void)data; (void)size;
    transmit_calls++;
    return uart_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    (void)uart;
    abort_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)channel;
    /* Observe the initial compare and update event before PWM starts. */
    return timer->Instance->CCR3 == 1500U && timer->Instance->EGR == TIM_EGR_UG ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *i2c) { (void)i2c; recover_calls++; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i2c) { (void)i2c; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i2c, uint16_t address, uint32_t trials, uint32_t timeout)
{
    (void)i2c; (void)address; (void)trials; (void)timeout; return HAL_OK;
}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *i2c) { (void)i2c; return HAL_I2C_ERROR_AF; }
void HAL_GPIO_Init(GPIO_TypeDef *gpio, GPIO_InitTypeDef *init) { (void)gpio; (void)init; }
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState state) { (void)gpio; (void)pin; (void)state; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio, uint16_t pin) { (void)gpio; (void)pin; return GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                  uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)i2c; (void)address; (void)reg; (void)reg_size;
    read_calls++;
    if (timeout != 2U || size != sizeof(raw_motion)) return HAL_ERROR;
    memcpy(data, raw_motion, size);
    return i2c_result;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *i2c, uint16_t address,
                                        uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)i2c; (void)address; (void)size; (void)timeout;
    current_command = data[0];
    if ((current_command & 0xE0U) == 0x40U)
    {
        conversion_command = current_command;
        if (command_count < sizeof(commands)) commands[command_count++] = current_command;
    }
    return i2c_result;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *i2c, uint16_t address,
                                       uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)i2c; (void)address; (void)timeout;
    if (current_command != MS5611_CMD_ADC_READ || size != 3U) return HAL_ERROR;
    uint32_t raw = (conversion_command & 0x10U) ? 8569150U : 9085466U;
    data[0] = raw >> 16U; data[1] = raw >> 8U; data[2] = raw;
    return i2c_result;
}

int test_main(void)
{
    htim4.Instance = TIM4;
    hi2c2.Instance = I2C2;
    Servo_Init();
    CHECK(TIM4->CCR3 == 1500U && Servo_GetAngle() == 72.5f);
    CHECK(RollControl_TargetAngle(10.0f) == 62.5f);
    CHECK(RollControl_TargetAngle(-10.0f) == 82.5f);
    Servo_SetAngle(RollControl_TargetAngle(72.5f)); CHECK(TIM4->CCR3 == 500U);
    Servo_SetAngle(RollControl_TargetAngle(-72.5f)); CHECK(TIM4->CCR3 == 2500U);
    Servo_SetAngle(RollControl_TargetAngle(180.0f)); CHECK(TIM4->CCR3 == 500U);
    Servo_SetAngle(RollControl_TargetAngle(-180.0f)); CHECK(TIM4->CCR3 == 2500U);
    Servo_SetAngle(NAN); CHECK(TIM4->CCR3 == 1500U);
    Servo_SetAngle(INFINITY); CHECK(TIM4->CCR3 == 1500U);
    Servo_SetPulseUs(0U); CHECK(TIM4->CCR3 == 500U);
    Servo_SetPulseUs(9999U); CHECK(TIM4->CCR3 == 2500U);
    CHECK(RollControl_Evaluate(35.0f, 0U, 0U) == 72.5f);
    CHECK(RollControl_Evaluate(35.0f, 1U, 41U) == 72.5f);
    CHECK(RollControl_Evaluate(35.0f, 1U, 40U) == 37.5f);
    CHECK(RollControl_Evaluate(NAN, 1U, 0U) == 72.5f);

    TickType_t release = 100U;
    volatile uint32_t overruns = 0U;
    fake_tick = 102U; PeriodicWait(&release, 8U, &overruns);
    CHECK(release == 108U && overruns == 0U);
    fake_tick = 139U; PeriodicWait(&release, 8U, &overruns);
    CHECK(release == 147U && overruns == 1U && last_wait == 8U);
    release = UINT32_MAX - 3U; fake_tick = 1U;
    PeriodicWait(&release, 8U, &overruns);
    CHECK(release == 4U && overruns == 1U);
    release = 200U; fake_tick = 215U;
    PeriodicWait(&release, 20U, &overruns);
    CHECK(release == 220U && overruns == 1U);

    BSP_DelayMs(5U); CHECK(delay_sum == 6U);
    scheduler_state = taskSCHEDULER_NOT_STARTED;
    BSP_DelayMs(5U); CHECK(hal_delay_sum == 5U);
    scheduler_state = taskSCHEDULER_RUNNING;

    LogRecord record = { .type = LOG_IMU };
    queue_result = errQUEUE_FULL; LogEnqueue(&record);
    CHECK(g_app_diagnostics.log_dropped == 1U && last_wait == 0U && critical_depth == 0U);
    queue_result = pdPASS; LogEnqueue(&record);
    CHECK(g_app_diagnostics.log_dropped == 1U && queue_calls == 2U);
    UartSend("I,0,0,0,0\r\n", 11U);
    CHECK(transmit_calls == 1U && abort_calls == 0U && last_wait == 20U);
    notify_result = pdFALSE; UartSend("X", 1U);
    CHECK(abort_calls == 1U && g_app_diagnostics.uart_errors == 1U);
    notify_result = pdTRUE; notify_events = UART_TX_ERROR; UartSend("X", 1U);
    CHECK(abort_calls == 2U && g_app_diagnostics.uart_errors == 2U);
    uart_result = HAL_BUSY; UartSend("X", 1U);
    CHECK(abort_calls == 3U && g_app_diagnostics.uart_errors == 3U);
    char formatted[16];
    FormatFixed(formatted, sizeof(formatted), -0.25f); CHECK(strcmp(formatted, "-0.25") == 0);
    FormatFixed(formatted, sizeof(formatted), 101325.0f); CHECK(strcmp(formatted, "101325.00") == 0);
    FormatFixed(formatted, sizeof(formatted), NAN); CHECK(strcmp(formatted, "nan") == 0);

    /* Datasheet example: 100009 Pa, 20.07 C, tested with both conversion modes. */
    const uint16_t calibration[8] = {0U,40127U,36924U,23317U,23282U,33464U,28312U,0U};
    memcpy(g_prom, calibration, sizeof(g_prom));
    g_ms5611_address = MS5611_I2C_ADDR_76;
    PressureSensor_Data_t pressure;
    delay_sum = 0U;
    CHECK(PressureSensor_ReadPeriodic(&pressure) == PRESSURE_SENSOR_OK);
    CHECK(commands[0] == 0x46U && commands[1] == 0x56U && delay_sum == 12U);
    CHECK(pressure.D1 == 9085466U && pressure.D2 == 8569150U);
    CHECK(pressure.Pressure_Pa == 100009.0f && fabsf(pressure.Temperature_C - 20.07f) < 0.001f);
    CHECK(PressureSensor_Read(&pressure) == PRESSURE_SENSOR_OK);
    CHECK(commands[2] == 0x48U && commands[3] == 0x58U && delay_sum == 34U);
    CHECK(PressureSensor_ReadPeriodic(NULL) == PRESSURE_SENSOR_INVALID_PARAMETER);
    i2c_result = HAL_ERROR;
    CHECK(PressureSensor_ReadPeriodic(&pressure) == PRESSURE_SENSOR_I2C_ERROR);
    CHECK(recover_calls == MS5611_I2C_RETRY_COUNT - 1U);

    ICM42688_Accel_t accel;
    ICM42688_Gyro_t gyro;
    icm42688_hi2c = &hi2c2;
    i2c_result = HAL_OK;
    CHECK(ICM42688_GetMotion(&accel, &gyro) == HAL_OK);
    CHECK(accel.x == 0.5f && accel.y == -0.5f && accel.z == 1.0f);
    CHECK(gyro.x == 10.0f && gyro.y == -10.0f && gyro.z == 0.0f);
    ICM42688_Attitude_Init();
    gyro.x = 0.0f; gyro.y = 0.0f;
    for (int i = 0; i < 500; ++i) ICM42688_Attitude_Update(&accel, &gyro, 0.008f);
    ICM42688_Attitude_t result;
    ICM42688_GetAttitude(&result);
    CHECK(result.roll < -20.0f && result.roll > -30.0f);
    CHECK(RollControl_TargetAngle(result.roll) > SERVO_CENTER_ANGLE);
    uint32_t reads_before = read_calls;
    i2c_result = HAL_ERROR;
    CHECK(ICM42688_GetMotion(&accel, &gyro) == HAL_ERROR);
    CHECK(read_calls == reads_before + 1U && ICM42688_GetBusRecoverCount() == 1U);
    return 0;
}
