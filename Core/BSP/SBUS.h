#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define SBUS_FRAME_LEN       25U
#define SBUS_HEADER          0x0FU
#define SBUS_CHANNEL_COUNT   16U
#define SBUS_RAW_MIN         172U
#define SBUS_RAW_MID         992U
#define SBUS_RAW_MAX         1811U

typedef struct
{
    uint16_t channel[SBUS_CHANNEL_COUNT];
    uint8_t  digital1;
    uint8_t  digital2;
    uint8_t  frame_lost;
    uint8_t  failsafe;
    uint8_t  frame_ready;
    uint32_t frame_count;
    uint32_t frame_err_count;
    uint32_t err_count;
    uint32_t uart_error;
    uint32_t uart_status;
    uint32_t rx_byte_count;
    uint32_t pe_count;
    uint32_t ne_count;
    uint32_t fe_count;
    uint32_t ore_count;
    uint32_t soft_edges;
    uint32_t soft_glitches;
    uint32_t soft_bad_bytes;
    uint32_t soft_run_count;
    uint32_t soft_run_sum_cycles;
    uint32_t soft_min_cycles;
    uint32_t soft_max_cycles;
    uint32_t rx_restart_count;
    uint32_t rx_restart_error;
    uint8_t  last_rx_byte;
    uint8_t  config_index;
    uint8_t  config_locked;
} SBUS_Data_t;

void SBUS_Init(UART_HandleTypeDef *huart);
void SBUS_Process(void);
const char *SBUS_GetConfigName(void);

uint8_t  SBUS_FrameReady(void);
void     SBUS_ClearFrameReady(void);
uint16_t SBUS_GetChannel(uint8_t index);
uint8_t  SBUS_Failsafe(void);
const volatile SBUS_Data_t *SBUS_GetData(void);

uint16_t SBUS_GetAmplitude(void);
float    SBUS_GetTurnAngle(void);

#ifdef __cplusplus
}
#endif

#endif
