#ifndef BSP_DELAY_H
#define BSP_DELAY_H
#include <stdint.h>
/* Minimum wall-time delay. Task context only; yields when FreeRTOS is running. */
void BSP_DelayMs(uint32_t milliseconds);
#endif
