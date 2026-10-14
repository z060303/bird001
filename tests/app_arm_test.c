/* Execute production functions with mocked HAL/RTOS boundaries on ARM emulation.
 * Includes give access to private periodic/log helpers without a production test API. */
/* Exercise the restored full application mode. */
#define APP_SBUS_ONLY 0U
#include "../Core/APP/app_freertos.c"
#include "../Core/BSP/PressureSensor.c"
#include "../Core/BSP/ICM42688.c"
#include "../Core/BSP/SBUS.c"
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
UART_HandleTypeDef huart4;
uint32_t SystemCoreClock = 168000000U;
I2C_HandleTypeDef hi2c1, hi2c2;
static TickType_t fake_tick, delay_sum, last_wait;
static BaseType_t scheduler_state = taskSCHEDULER_RUNNING;
static uint32_t hal_delay_sum, transmit_calls, abort_calls, notify_events = UART_TX_DONE;
static BaseType_t notify_result = pdTRUE, queue_result = pdPASS;
static HAL_StatusTypeDef uart_result = HAL_OK, i2c_result = HAL_OK;
static uint32_t queue_calls, critical_depth, read_calls, recover_calls;
static LogRecord queued_log;
volatile BLDC_Handle motor;
static uint32_t requested_period, motor_starts, motor_stops;
static uint8_t requested_duty;

BLDC_RunningState BLDC_GetRunningState(void) { return motor.state; }
void BLDC_SetCommutationPeriodUs(uint32_t period) { requested_period = period; }
void BLDC_SetDutyPercent(uint8_t duty) { requested_duty = duty; }
void BLDC_OpenLoopStart(void) { motor_starts++; motor.state = BLDC_STATE_BOOTSTRAP_PRECHARGE; }
void BLDC_Stop(void) { motor_stops++; motor.state = BLDC_STATE_STOPPED; }
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
    (void)queue; (void)position;
    queue_calls++;
    last_wait = wait;
    if (queue_result == pdPASS) memcpy(&queued_log, item, sizeof(queued_log));
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
void HAL_GPIO_DeInit(GPIO_TypeDef *gpio, uint32_t pins) { (void)gpio; (void)pins; }
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subpriority)
{ (void)irq; (void)priority; (void)subpriority; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; }
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
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

static void TestSbusByte(uint8_t byte, uint32_t error)
{
    UART4->SR = USART_SR_RXNE | error;
    UART4->DR = byte;
    UART4_IRQHandler();
}

static void TestSbusGap(void)
{
    UART4->SR = USART_SR_IDLE;
    UART4->DR = 0U;
    UART4_IRQHandler();
}

static int TestSbusDriver(void)
{
    uint16_t channels[SBUS_CHANNEL_COUNT];
    uint8_t frame[SBUS_FRAME_LEN];
    uint32_t random = 42U;
    huart4.Instance = UART4;
    huart4.Init.Mode = UART_MODE_RX;
    SBUS_Init(&huart4);
    CHECK(huart4.Init.BaudRate == 100000U && huart4.Init.WordLength == UART_WORDLENGTH_9B);
    CHECK(huart4.Init.Parity == UART_PARITY_EVEN && huart4.Init.StopBits == UART_STOPBITS_2);
    memset(frame, 0, sizeof(frame)); frame[0] = SBUS_HEADER;
    for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i) TestSbusByte(frame[i], 0U);
    SBUS_Process(); CHECK(s_data.frame_count == 0U); /* Wait for first gap after startup. */
    for (unsigned n = 0U; n < 1000U; ++n)
    {
        memset(frame, 0, sizeof(frame));
        frame[0] = SBUS_HEADER;
        frame[23] = n & 15U;
        for (unsigned i = 0U; i < SBUS_CHANNEL_COUNT; ++i)
        {
            random = random * 1664525U + 1013904223U;
            channels[i] = (random >> 16U) & 2047U;
            for (unsigned bit = 0U; bit < 11U; ++bit)
                if (channels[i] & (1U << bit))
                    frame[1U + (i * 11U + bit) / 8U] |= 1U << ((i * 11U + bit) % 8U);
        }
        TestSbusGap();
        for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i) TestSbusByte(frame[i], 0U);
        SBUS_Process();
        for (unsigned i = 0U; i < SBUS_CHANNEL_COUNT; ++i)
            CHECK(SBUS_GetChannel(i) == channels[i]);
        CHECK(s_data.digital1 == (n & 1U) && s_data.digital2 == ((n >> 1U) & 1U));
        CHECK(s_data.frame_lost == ((n >> 2U) & 1U) && s_data.failsafe == ((n >> 3U) & 1U));
    }
    CHECK(s_data.frame_count == 1000U);
    uint32_t before = s_data.frame_count;
    const uint32_t errors[] = { USART_SR_PE, USART_SR_FE, USART_SR_NE, USART_SR_ORE };
    for (unsigned fault = 0U; fault < 4U; ++fault)
    {
        TestSbusGap();
        for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i)
            TestSbusByte(frame[i], i == 1U ? errors[fault] : 0U);
        SBUS_Process();
        CHECK(s_data.frame_count == before);
    }
    CHECK(s_data.pe_count == 1U && s_data.fe_count == 1U && s_data.ne_count == 1U && s_data.ore_count == 1U);
    TestSbusGap();
    for (unsigned i = 0U; i < 10U; ++i) TestSbusByte(frame[i], 0U);
    fake_tick += 20U;
    TestSbusGap();
    for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i) TestSbusByte(frame[i], 0U);
    SBUS_Process();
    CHECK(s_data.frame_count == ++before);
    for (unsigned i = 0U; i < SBUS_CHANNEL_COUNT; ++i) CHECK(s_data.channel[i] == channels[i]);
    frame[24] = 0x01U;
    TestSbusGap();
    for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i) TestSbusByte(frame[i], 0U);
    SBUS_Process(); CHECK(s_data.frame_count == before);
    const uint8_t footers[] = { 0x00U, 0x04U, 0x08U, 0x14U, 0x24U, 0x34U };
    for (unsigned n = 0U; n < sizeof(footers); ++n)
    {
        frame[24] = footers[n]; TestSbusGap();
        for (unsigned i = 0U; i < SBUS_FRAME_LEN; ++i) TestSbusByte(frame[i], 0U);
        SBUS_Process(); CHECK(s_data.frame_count == ++before);
    }
    uint8_t config = s_data.config_index;
    fake_tick += 2000U; SBUS_Process();
    CHECK(s_data.config_index == config && huart4.Init.Parity == UART_PARITY_EVEN);
    return 0;
}

static int TestCh3Control(void)
{
    SbusMotorControl control = {0};
    SbusMotorCommand request = SbusMotor_Update(&control, 1912U, 1U, 0U);
    CHECK(!control.armed && request.period_us == 0U); /* High throttle cannot start at boot. */
    request = SbusMotor_Update(&control, 312U, 1U, 0U);
    CHECK(control.armed && request.period_us == 0U);
    request = SbusMotor_Update(&control, 332U, 1U, 0U);
    CHECK(request.period_us == 0U);
    request = SbusMotor_Update(&control, 333U, 1U, 0U);
    CHECK(request.period_us == BLDC_START_PERIOD_US && request.nominal_rpm == 83U);
    CHECK(request.duty_percent == BLDC_TARGET_DUTY_PERCENT);
    uint32_t prior_period = BLDC_START_PERIOD_US, prior_rpm = 0U;
    for (unsigned raw = 333U; raw <= 2047U; ++raw)
    {
        request = SbusMotor_Update(&control, raw, 1U, 0U);
        CHECK(request.period_us <= prior_period && request.period_us >= BLDC_TARGET_PERIOD_US);
        CHECK(request.nominal_rpm >= prior_rpm && request.duty_percent == BLDC_TARGET_DUTY_PERCENT);
        prior_period = request.period_us; prior_rpm = request.nominal_rpm;
    }
    CHECK(request.period_us == BLDC_TARGET_PERIOD_US && request.nominal_rpm == 556U);
    request = SbusMotor_Update(&control, 1912U, 1U, 100U); CHECK(request.period_us != 0U);
    request = SbusMotor_Update(&control, 1912U, 1U, 101U);
    CHECK(!control.armed && request.period_us == 0U);
    request = SbusMotor_Update(&control, 1912U, 1U, 0U); CHECK(request.period_us == 0U);

    SBUS_Data_t remote = {0}; remote.channel[0] = 942U; remote.channel[2] = 1912U;
    SbusControlSample sample = SbusControlSnapshot(&remote, 0U, 1U);
    CHECK(sample.ch1 == 942U && sample.ch3 == 1912U);
    ApplySbusMotorSample(&sample, 1U, 0U, &control);
    CHECK(motor_starts == 0U && requested_duty == 0U);
    remote.channel[2] = 312U; sample = SbusControlSnapshot(&remote, 1U, 1U);
    ApplySbusMotorSample(&sample, 1U, 1U, &control);
    CHECK(control.armed && motor_starts == 0U);
    remote.channel[2] = 1912U; sample = SbusControlSnapshot(&remote, 2U, 1U);
    ApplySbusMotorSample(&sample, 1U, 2U, &control);
    CHECK(motor_starts == 1U && requested_period == 3000U && requested_duty == 16U);
    remote.channel[2] = 1112U; sample = SbusControlSnapshot(&remote, 3U, 1U);
    ApplySbusMotorSample(&sample, 1U, 3U, &control);
    CHECK(motor_starts == 1U && requested_period > 3000U && requested_period < 20000U);
    motor.state = BLDC_STATE_OPEN_LOOP_RUNNING;
    ApplySbusMotorSample(&sample, 1U, 104U, &control);
    CHECK(motor_stops == 1U && !control.armed && requested_duty == 0U);
    sample.tick = 105U; ApplySbusMotorSample(&sample, 1U, 105U, &control);
    CHECK(motor_starts == 1U); /* A recovered high throttle stays stopped. */
    sample.ch3 = 312U; ApplySbusMotorSample(&sample, 1U, 105U, &control);
    sample.ch3 = 1912U; ApplySbusMotorSample(&sample, 1U, 105U, &control);
    CHECK(motor_starts == 2U);
    remote.frame_lost = 1U; sample = SbusControlSnapshot(&remote, 106U, 1U);
    ApplySbusMotorSample(&sample, 1U, 106U, &control);
    CHECK(motor_stops == 2U && !control.armed && g_app_diagnostics.motor_failsafe);
    remote.frame_lost = 0U; remote.failsafe = 1U;
    sample = SbusControlSnapshot(&remote, 107U, 1U);
    ApplySbusMotorSample(&sample, 1U, 107U, &control); CHECK(!control.armed);
    remote.failsafe = 0U; remote.channel[2] = 312U;
    sample = SbusControlSnapshot(&remote, 108U, 1U);
    ApplySbusMotorSample(&sample, 0U, 108U, &control); CHECK(!control.armed);
    ApplySbusMotorSample(&sample, 1U, 108U, &control); CHECK(control.armed);
    sample.ch3 = 1912U; sample.tick = UINT32_MAX - 49U;
    ApplySbusMotorSample(&sample, 1U, 50U, &control); CHECK(motor_starts == 3U);
    ApplySbusMotorSample(&sample, 1U, 51U, &control); CHECK(motor_stops == 3U && !control.armed);
    sample.ch3 = 312U; sample.tick = 109U;
    ApplySbusMotorSample(&sample, 1U, 109U, &control);
    motor.state = BLDC_STATE_FAULT;
    sample.ch3 = 1912U; ApplySbusMotorSample(&sample, 1U, 109U, &control);
    CHECK(motor_stops == 4U && !control.armed && motor_starts == 3U);
    return 0;
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

    CHECK(SbusServo_TargetAngle(544U) == 0.0f);
    CHECK(SbusServo_TargetAngle(942U) == 72.5f);
    CHECK(SbusServo_TargetAngle(1340U) == 145.0f);
    CHECK(SbusServo_TargetAngle(0U) == 0.0f && SbusServo_TargetAngle(2047U) == 145.0f);
    CHECK(SbusServo_TargetAngle(743U) == 36.25f);
    CHECK(SbusServo_TargetAngle(1141U) == 108.75f);
    float previous = -1.0f;
    for (unsigned raw = 0U; raw <= 2047U; ++raw)
    {
        float angle = SbusServo_TargetAngle(raw);
        CHECK(isfinite(angle) && angle >= previous && angle >= 0.0f && angle <= 145.0f);
        previous = angle;
    }
    SBUS_Data_t remote = {0}; remote.channel[0] = 1340U;
    SbusControlSample control_sample = SbusControlSnapshot(&remote, 10U, 1U);
    ApplySbusServoSample(&control_sample, 1U, 110U);
    CHECK(TIM4->CCR3 == 2500U && !g_app_diagnostics.servo_failsafe);
    ApplySbusServoSample(&control_sample, 1U, 111U);
    CHECK(TIM4->CCR3 == 1500U && g_app_diagnostics.servo_failsafe);
    ApplySbusServoSample(&control_sample, 0U, 10U);
    CHECK(TIM4->CCR3 == 1500U && g_app_diagnostics.servo_failsafe);
    remote.failsafe = 1U; control_sample = SbusControlSnapshot(&remote, 10U, 1U);
    ApplySbusServoSample(&control_sample, 1U, 10U); CHECK(TIM4->CCR3 == 1500U);
    remote.failsafe = 0U; remote.frame_lost = 1U;
    control_sample = SbusControlSnapshot(&remote, 10U, 1U);
    ApplySbusServoSample(&control_sample, 1U, 10U); CHECK(TIM4->CCR3 == 1500U);
    remote.frame_lost = 0U; control_sample = SbusControlSnapshot(&remote, 10U, 0U);
    ApplySbusServoSample(&control_sample, 1U, 10U); CHECK(TIM4->CCR3 == 1500U);
    remote.channel[0] = 544U;
    control_sample = SbusControlSnapshot(&remote, UINT32_MAX - 49U, 1U);
    remote.channel[0] = 1340U; /* Queue snapshot survives later channel updates. */
    ApplySbusServoSample(&control_sample, 1U, 50U); CHECK(TIM4->CCR3 == 500U);
    ApplySbusServoSample(&control_sample, 1U, 51U); CHECK(TIM4->CCR3 == 1500U);
    remote.channel[0] = 942U; control_sample = SbusControlSnapshot(&remote, 100U, 1U);
    ApplySbusServoSample(&control_sample, 1U, 100U);
    CHECK(TIM4->CCR3 == 1500U && !g_app_diagnostics.servo_failsafe);

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

    PressureLogRecord record = { .pressure_pa = 101325.0f, .relative_height_m = 0.0f };
    queue_result = errQUEUE_FULL; LogEnqueue(&record);
    CHECK(g_app_diagnostics.log_dropped == 1U && last_wait == 0U && critical_depth == 0U);
    queue_result = pdPASS; LogEnqueue(&record);
    CHECK(g_app_diagnostics.log_dropped == 1U && queue_calls == 2U);
    UartSend("baro:101325.00,0.00\r\n", 22U);
    CHECK(transmit_calls == 1U && abort_calls == 0U && last_wait == UART_TIMEOUT_MS);
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

    /* Existing pressure telemetry remains unchanged alongside tagged SBUS lines. */
    char vofa_line[128];
    int vofa_length = FormatPressureLine(vofa_line, sizeof(vofa_line), &record);
    CHECK(vofa_length > 0 && strcmp(vofa_line, "baro:101325.00,0.00\r\n") == 0);
    CHECK(vofa_length == (int)strlen(vofa_line));
    record.pressure_pa = 99999.25f;
    record.relative_height_m = -12.34f;
    CHECK(FormatPressureLine(vofa_line, sizeof(vofa_line), &record) > 0);
    CHECK(strcmp(vofa_line, "baro:99999.25,-12.34\r\n") == 0);
    CHECK(App_LogWrite("IMU debug\r\n", 11) == -1 && queue_calls == 2U);

    CHECK(queued_log.type == LOG_PRESSURE && queued_log.data.pressure.pressure_pa == 101325.0f);
    SBUS_Data_t sbus = { .frame_count = UINT32_MAX, .digital1 = 1U,
                        .digital2 = 1U, .frame_lost = 1U, .failsafe = 1U };
    for (unsigned i = 0U; i < SBUS_CHANNEL_COUNT; ++i) sbus.channel[i] = 2047U;
    TickType_t last_sbus_log = 0U;
    uint8_t last_link = 0U;
    uint32_t sends = queue_calls;
    LogSbusSample(&sbus, 1U, 1U, 0U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 1U && last_link && queued_log.type == LOG_SBUS);
    sbus.channel[0] = 0U;
    CHECK(queued_log.data.sbus.channel[0] == 2047U); /* Immutable queued snapshot. */
    char sbus_line[256];
    int sbus_length = FormatSbusLine(sbus_line, sizeof(sbus_line), &queued_log.data.sbus);
    CHECK(sbus_length > 0 && sbus_length == (int)strlen(sbus_line));
    CHECK(strstr(sbus_line, "sbus:CH1=2047 CH2=2047 ") == sbus_line);
    CHECK(strstr(sbus_line, "CH16=2047 D17=1 D18=1 lost=1 fs=1 n=4294967295\r\n") != NULL);
    char tiny_line[8];
    CHECK(FormatSbusLine(tiny_line, sizeof(tiny_line), &queued_log.data.sbus) == -1);
    LogSbusSample(&sbus, 1U, 1U, 49U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 1U);
    LogSbusSample(&sbus, 1U, 1U, 50U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 2U && queued_log.data.sbus.channel[0] == 0U);
    LogSbusSample(&sbus, 0U, 1U, 100U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 2U); /* Do not repeat an old frame. */
    sbus.rx_byte_count = 250U; sbus.err_count = 2U;
    LogSbusSample(&sbus, 0U, 0U, 151U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 3U && !last_link);
    CHECK(FormatSbusLine(sbus_line, sizeof(sbus_line), &queued_log.data.sbus) > 0);
    CHECK(strcmp(sbus_line, "sbus:NO_SIGNAL rx=250 err=2\r\n") == 0);
    LogSbusSample(&sbus, 0U, 0U, 1150U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 3U);
    LogSbusSample(&sbus, 0U, 0U, 1151U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 4U);
    LogSbusSample(&sbus, 1U, 1U, 1152U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 5U && last_link);
    last_sbus_log = UINT32_MAX - 24U;
    LogSbusSample(&sbus, 1U, 1U, 25U, &last_sbus_log, &last_link);
    CHECK(queue_calls == sends + 6U); /* 50 ms across tick wrap. */
    queue_result = errQUEUE_FULL;
    LogSbusSample(&sbus, 1U, 1U, 75U, &last_sbus_log, &last_link);
    CHECK(g_app_diagnostics.log_dropped == 2U && last_wait == 0U && critical_depth == 0U);
    queue_result = pdPASS;

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
    float absolute_altitude = pressure.Altitude;
    PressureReferenceCapture capture = {0};
    uint8_t publish = 0U;
    CHECK(PressureSensor_SetReferencePressure(0.0f) == PRESSURE_SENSOR_INVALID_PARAMETER);
    CHECK(!PressureSensor_IsReferenceValid());
    CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(!publish && !PressureSensor_IsReferenceValid() && capture.count == 1U);
    for (unsigned i = 1U; i < PRESSURE_REFERENCE_SAMPLES - 1U; ++i)
    {
        pressure.Pressure_Pa = 100009.0f + (i % 2 == 0 ? 1.0f : -1.0f);
        CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
        CHECK(!publish);
    }
    pressure.Pressure_Pa = 100009.0f;
    CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(publish && capture.count == PRESSURE_REFERENCE_SAMPLES);
    CHECK(PressureSensor_IsReferenceValid() && PressureSensor_GetReferencePressure() == 100009.0f);
    CHECK(pressure.Altitude == absolute_altitude);
    CHECK(record.relative_height_m == 0.0f);
    CHECK(FormatPressureLine(vofa_line, sizeof(vofa_line), &record) > 0);
    CHECK(strcmp(vofa_line, "baro:100009.00,0.00\r\n") == 0);
    CHECK(PressureSensor_Read(&pressure) == PRESSURE_SENSOR_OK);
    CHECK(commands[2] == 0x48U && commands[3] == 0x58U && delay_sum == 34U);
    CHECK(PressureSensor_ReadPeriodic(NULL) == PRESSURE_SENSOR_INVALID_PARAMETER);
    /* A moving startup restarts calibration, so an in-motion P0 is not frozen. */
    CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(publish); /* Once captured, ordinary samples publish every 20 ms. */
    CHECK(PressureSensor_SetReferencePressure(NAN) == PRESSURE_SENSOR_INVALID_PARAMETER);
    PressureReferenceCapture moving = {0};
    g_reference_valid = 0U;
    pressure.Pressure_Pa = 100009.0f;
    CHECK(PreparePressureSample(&pressure, &moving, &record, &publish) == PRESSURE_SENSOR_OK);
    pressure.Pressure_Pa = 100018.0f;
    CHECK(PreparePressureSample(&pressure, &moving, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(!publish && moving.count == 1U && !PressureSensor_IsReferenceValid());
    CHECK(PressureSensor_SetReferencePressure(100009.0f) == PRESSURE_SENSOR_OK);
    /* Three-point median rejects an isolated 30 Pa spike. */
    g_sample_pressure_filter = 0.0f;
    PressureSensor_ResetAltitudeFilter();
    for (int i = 0; i < 4; ++i) CHECK(fabsf(PressureSensor_FilterSamplePressure(100000.0f) - 100000.0f) < 0.01f);
    CHECK(fabsf(PressureSensor_FilterSamplePressure(100030.0f) - 100000.0f) < 0.01f);
    CHECK(fabsf(PressureSensor_FilterSamplePressure(100000.0f) - 100000.0f) < 0.01f);
    for (int i = 0; i < 12; ++i) (void)PressureSensor_FilterSamplePressure(99988.0f);
    CHECK(g_sample_pressure_filter < 99990.0f);
    for (int i = 0; i < 25; ++i) (void)PressureSensor_FilterSamplePressure(100000.0f);
    CHECK(fabsf(g_sample_pressure_filter - 100000.0f) < 1.0f);
    CHECK(PressureSensor_SetReferencePressure(100000.0f) == PRESSURE_SENSOR_OK);
    for (int i = 0; i < 100; ++i)
    {
        float noisy = 100000.0f + (float)((i * 17) % 9 - 4);
        float filtered = PressureSensor_FilterSamplePressure(noisy);
        CHECK(fabsf(filtered - 100000.0f) < 2.5f);
        CHECK(fabsf(PressureSensor_CalculateRelativeAltitude(filtered)) < 0.2f);
    }
    CHECK(PressureSensor_SetReferencePressure(100009.0f) == PRESSURE_SENSOR_OK);
    pressure.Pressure_Pa = 99900.0f;
    for (int i = 0; i < 20; ++i) CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(record.relative_height_m > 0.0f);
    pressure.Pressure_Pa = 100100.0f;
    for (int i = 0; i < 20; ++i) CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(record.relative_height_m < 0.0f && PressureSensor_GetReferencePressure() == 100009.0f);
    pressure.Pressure_Pa = 100009.0f;
    for (int i = 0; i < 20; ++i) CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_OK);
    CHECK(fabsf(record.relative_height_m) < 0.1f);
    pressure.Pressure_Pa = NAN;
    CHECK(PreparePressureSample(&pressure, &capture, &record, &publish) == PRESSURE_SENSOR_INVALID_PARAMETER);
    CHECK(PressureSensor_GetReferencePressure() == 100009.0f);
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
    int ch3_result = TestCh3Control();
    return ch3_result ? ch3_result : TestSbusDriver();
}
