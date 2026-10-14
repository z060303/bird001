#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "tim.h"
#include "BLDC.h"
#include "app_freertos.h"

/* Keep fault location available in the debugger; never print from a fault hook. */
const char * volatile g_app_assert_file;
volatile int g_app_assert_line;

void App_AssertFailed(const char *file, int line)
{
    __disable_irq();
#if !APP_SBUS_ONLY && APP_MOTOR_TASK_ENABLED
    BLDC_EmergencyStop();
#endif
    g_app_assert_file = file;
    g_app_assert_line = line;
#if !APP_SBUS_ONLY
    if (htim4.Instance == TIM4) TIM4->CCR3 = 1500U;
#endif
    for (;;) { __NOP(); }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    App_AssertFailed(name, -1);
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, configSTACK_DEPTH_TYPE *depth)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = configMINIMAL_STACK_SIZE;
}
