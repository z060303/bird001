#include "bsp_delay.h"
#include "main.h"
#ifdef APP_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

void BSP_DelayMs(uint32_t milliseconds)
{
#ifdef APP_USE_FREERTOS
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        configASSERT(__get_IPSR() == 0U);
        /* One extra tick covers entry just before the next tick edge. */
        vTaskDelay(pdMS_TO_TICKS(milliseconds) + 1U);
        return;
    }
#endif
    HAL_Delay(milliseconds);
}
