#include "SBUS.h"
#include <string.h>

static uint8_t s_isr_buf[SBUS_FRAME_LEN];
static uint8_t s_frame[SBUS_FRAME_LEN];
static uint8_t s_rx_idx;
static uint8_t s_rx_state;
static UART_HandleTypeDef *s_huart;

typedef struct
{
    uint32_t baud_rate;
    uint32_t word_length;
    uint32_t parity;
    uint32_t stop_bits;
    const char *name;
} SBUS_Config_t;

static const SBUS_Config_t s_configs[] =
{
    /* STM32F4: 9B + parity enabled means eight payload bits plus parity. */
    {100000U, UART_WORDLENGTH_9B, UART_PARITY_EVEN, UART_STOPBITS_2, "100k 8E2"},
    {100000U, UART_WORDLENGTH_9B, UART_PARITY_NONE, UART_STOPBITS_2, "100k 9N2"},
    {100000U, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_2, "100k 8N2"},
    {100000U, UART_WORDLENGTH_8B, UART_PARITY_EVEN, UART_STOPBITS_2, "100k 7E2"},
    {100000U, UART_WORDLENGTH_8B, UART_PARITY_EVEN, UART_STOPBITS_1, "100k 7E1"},
    {200000U, UART_WORDLENGTH_9B, UART_PARITY_NONE, UART_STOPBITS_2, "200k 9N2"},
    {200000U, UART_WORDLENGTH_9B, UART_PARITY_EVEN, UART_STOPBITS_2, "200k 8E2"},
};

#define SBUS_UART_CONFIG_COUNT  (sizeof(s_configs) / sizeof(s_configs[0]))
#define SBUS_MODE_SOFT_100K_12  0U
#define SBUS_MODE_SOFT_100K_11  1U
#define SBUS_MODE_SOFT_200K_12  2U
#define SBUS_MODE_SOFT_200K_11  3U
#define SBUS_MODE_SOFT_ADAPT_12 4U
#define SBUS_MODE_SOFT_ADAPT_11 5U
#define SBUS_MODE_UART_BASE     6U
#define SBUS_MODE_COUNT         (SBUS_MODE_UART_BASE + SBUS_UART_CONFIG_COUNT)
#define SBUS_CONFIG_TEST_MS     1000U
#define SBUS_ADAPT_MEASURE_MS   500U
#define SBUS_CONFIG_MIN_FRAMES  20U
#define SBUS_SOFT_GAP_BITS      6U
#define SBUS_SOFT_HIST_MAX_US   100U

static volatile SBUS_Data_t s_data;
static uint32_t s_config_trial_start;
static uint32_t s_config_trial_frames;
static uint32_t s_last_frame_tick;

static uint32_t s_soft_bit_cycles;
static uint32_t s_soft_cycles_per_us;
static uint32_t s_soft_last_cycles;
static uint8_t s_soft_last_level;
static uint8_t s_soft_idle_level;
static uint8_t s_soft_idle_known;
static uint8_t s_soft_active;
static uint8_t s_soft_bit_count;
static uint16_t s_soft_bits;
static uint8_t s_soft_frame_bits;
static uint8_t s_soft_adaptive;
static uint8_t s_soft_refined;
static uint16_t s_soft_run_hist[SBUS_SOFT_HIST_MAX_US + 1U];
static uint32_t s_soft_run_hist_sum[SBUS_SOFT_HIST_MAX_US + 1U];

static uint32_t SBUS_HalErrorFromStatus(uint32_t status)
{
    uint32_t error = HAL_UART_ERROR_NONE;

    if ((status & USART_SR_PE) != 0U)  error |= HAL_UART_ERROR_PE;
    if ((status & USART_SR_NE) != 0U)  error |= HAL_UART_ERROR_NE;
    if ((status & USART_SR_FE) != 0U)  error |= HAL_UART_ERROR_FE;
    if ((status & USART_SR_ORE) != 0U) error |= HAL_UART_ERROR_ORE;

    return error;
}

static void SBUS_ResetFrameState(void)
{
    s_rx_state = 0U;
    s_rx_idx = 0U;
}

static uint16_t SBUS_Constrain(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void SBUS_ParseFrame(const uint8_t *buf)
{
    uint16_t channel[SBUS_CHANNEL_COUNT];

    channel[0]  = (uint16_t)((buf[1]    | buf[2]  << 8) & 0x07FF);
    channel[1]  = (uint16_t)((buf[2]>>3 | buf[3]  << 5) & 0x07FF);
    channel[2]  = (uint16_t)((buf[3]>>6 | buf[4]  << 2 | buf[5]  << 10) & 0x07FF);
    channel[3]  = (uint16_t)((buf[5]>>1 | buf[6]  << 7) & 0x07FF);
    channel[4]  = (uint16_t)((buf[6]>>4 | buf[7]  << 4) & 0x07FF);
    channel[5]  = (uint16_t)((buf[7]>>7 | buf[8]  << 1 | buf[9]  << 9) & 0x07FF);
    channel[6]  = (uint16_t)((buf[9]>>2 | buf[10] << 6) & 0x07FF);
    channel[7]  = (uint16_t)((buf[10]>>5| buf[11] << 3) & 0x07FF);
    channel[8]  = (uint16_t)((buf[12]  | buf[13] << 8) & 0x07FF);
    channel[9]  = (uint16_t)((buf[13]>>3| buf[14] << 5) & 0x07FF);
    channel[10] = (uint16_t)((buf[14]>>6| buf[15] << 2 | buf[16] << 10) & 0x07FF);
    channel[11] = (uint16_t)((buf[16]>>1| buf[17] << 7) & 0x07FF);
    channel[12] = (uint16_t)((buf[17]>>4| buf[18] << 4) & 0x07FF);
    channel[13] = (uint16_t)((buf[18]>>7| buf[19] << 1 | buf[20] << 9) & 0x07FF);
    channel[14] = (uint16_t)((buf[20]>>2| buf[21] << 6) & 0x07FF);
    channel[15] = (uint16_t)((buf[21]>>5| buf[22] << 3) & 0x07FF);

    s_data.digital1  = (buf[23] & 0x01) ? 1U : 0U;
    s_data.digital2  = (buf[23] & 0x02) ? 1U : 0U;
    s_data.frame_lost= (buf[23] & 0x04) ? 1U : 0U;
    s_data.failsafe  = (buf[23] & 0x08) ? 1U : 0U;

    for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++)
        s_data.channel[i] = channel[i];

    s_data.frame_count++;
}

static void SBUS_ConsumeByte(uint8_t b)
{
    if (s_rx_state == 0U)
    {
        if (b == SBUS_HEADER)
        {
            s_isr_buf[0] = b;
            s_rx_idx = 1U;
            s_rx_state = 1U;
        }
    }
    else
    {
        s_isr_buf[s_rx_idx++] = b;
        if (s_rx_idx >= SBUS_FRAME_LEN)
        {
            s_rx_state = 0U;
            if ((s_isr_buf[0] == SBUS_HEADER) &&
                ((s_isr_buf[24] & 0xF0U) == 0U))
            {
                uint32_t basepri = __get_BASEPRI();
                __set_BASEPRI(1U << (8U - __NVIC_PRIO_BITS));
                memcpy(s_frame, s_isr_buf, SBUS_FRAME_LEN);
                __set_BASEPRI(basepri);
                s_data.frame_ready = 1U;
            }
            else
            {
                s_data.frame_err_count++;
                s_data.err_count++;
            }
        }
    }
}

static void SBUS_SoftReset(void)
{
    s_soft_idle_known = 0U;
    s_soft_active = 0U;
    s_soft_bit_count = 0U;
    s_soft_bits = 0U;
}

static void SBUS_SoftAppend(uint8_t logic, uint32_t count)
{
    while ((count > 0U) && (s_soft_bit_count < s_soft_frame_bits))
    {
        if (logic != 0U)
            s_soft_bits |= (uint16_t)(1U << s_soft_bit_count);

        s_soft_bit_count++;
        count--;
    }
}

static void SBUS_SoftDecodeByte(void)
{
    uint8_t start_bit = (uint8_t)(s_soft_bits & 0x01U);
    uint8_t data = (uint8_t)((s_soft_bits >> 1U) & 0xFFU);
    uint8_t stop_bits = (uint8_t)((s_soft_bits >> (s_soft_frame_bits - 2U)) & 0x03U);

    if ((start_bit == 0U) && (stop_bits == 0x03U))
    {
        s_data.rx_byte_count++;
        s_data.last_rx_byte = data;
        SBUS_ConsumeByte(data);
    }
    else
    {
        s_data.soft_bad_bytes++;
        s_data.err_count++;
        SBUS_ResetFrameState();
    }
}

static void SBUS_SoftEdge(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - s_soft_last_cycles;
    uint32_t run;
    uint8_t current_level = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t start_next = 0U;

    s_data.soft_edges++;

    if (delta < (SystemCoreClock / 10000U))
    {
        uint32_t run_us = (delta + (s_soft_cycles_per_us / 2U)) / s_soft_cycles_per_us;

        s_data.soft_run_count++;
        s_data.soft_run_sum_cycles += delta;

        if ((s_data.soft_min_cycles == 0U) || (delta < s_data.soft_min_cycles))
            s_data.soft_min_cycles = delta;
        if (delta > s_data.soft_max_cycles)
            s_data.soft_max_cycles = delta;

        if ((run_us > 0U) && (run_us <= SBUS_SOFT_HIST_MAX_US))
        {
            if (s_soft_run_hist[run_us] != UINT16_MAX)
                s_soft_run_hist[run_us]++;
            s_soft_run_hist_sum[run_us] += delta;
        }
    }

    /* Ignore edges shorter than half a bit; they are usually switching noise. */
    if (delta < (s_soft_bit_cycles / 2U))
    {
        s_data.soft_glitches++;
        return;
    }

    run = (delta + (s_soft_bit_cycles / 2U)) / s_soft_bit_cycles;
    if (run == 0U)
        run = 1U;

    if (s_soft_idle_known == 0U)
    {
        if (run >= SBUS_SOFT_GAP_BITS)
        {
            s_soft_idle_level = s_soft_last_level;
            s_soft_idle_known = 1U;

            if (current_level != s_soft_idle_level)
                start_next = 1U;
        }
    }
    else if (s_soft_active != 0U)
    {
        uint32_t remaining = s_soft_frame_bits - s_soft_bit_count;
        uint8_t logic = (s_soft_last_level == s_soft_idle_level) ? 1U : 0U;

        if ((run > remaining) &&
            (logic != 0U) &&
            (remaining <= 2U))
        {
            SBUS_SoftAppend(1U, remaining);
        }
        else if (run > remaining)
        {
            s_data.soft_bad_bytes++;
            s_data.err_count++;
            SBUS_ResetFrameState();
            s_soft_active = 0U;
            s_soft_bit_count = 0U;
            s_soft_bits = 0U;
        }
        else
        {
            SBUS_SoftAppend(logic, run);
        }

        if (s_soft_bit_count >= s_soft_frame_bits)
        {
            SBUS_SoftDecodeByte();
            s_soft_active = 0U;
            s_soft_bit_count = 0U;
            s_soft_bits = 0U;

            if (current_level != s_soft_idle_level)
                start_next = 1U;
        }
    }
    else
    {
        if (run >= SBUS_SOFT_GAP_BITS)
        {
            s_soft_idle_level = s_soft_last_level;

            if (current_level != s_soft_idle_level)
                start_next = 1U;
        }
        else if ((s_soft_last_level == s_soft_idle_level) &&
                 (current_level != s_soft_idle_level))
        {
            start_next = 1U;
        }
    }

    if (start_next != 0U)
    {
        s_soft_active = 1U;
        s_soft_bit_count = 0U;
        s_soft_bits = 0U;
    }

    s_soft_last_cycles = now;
    s_soft_last_level = current_level;
}

static void SBUS_SoftInit(uint32_t baud_rate, uint8_t frame_bits, uint8_t adaptive)
{
    GPIO_InitTypeDef gpio = {0};

    if (s_huart != NULL)
    {
        __HAL_UART_DISABLE_IT(s_huart, UART_IT_RXNE | UART_IT_PE);
        __HAL_UART_DISABLE_IT(s_huart, UART_IT_ERR);
        __HAL_UART_DISABLE(s_huart);
    }

    HAL_NVIC_DisableIRQ(UART4_IRQn);
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_soft_bit_cycles = SystemCoreClock / baud_rate;
    if (s_soft_bit_cycles == 0U)
        s_soft_bit_cycles = 1U;
    s_soft_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_soft_cycles_per_us == 0U)
        s_soft_cycles_per_us = 1U;
    s_soft_frame_bits = frame_bits;
    s_soft_adaptive = adaptive;
    s_soft_refined = 0U;
    memset(s_soft_run_hist, 0, sizeof(s_soft_run_hist));
    memset(s_soft_run_hist_sum, 0, sizeof(s_soft_run_hist_sum));

    s_soft_last_cycles = DWT->CYCCNT;
    s_soft_last_level = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) ? 1U : 0U;
    SBUS_SoftReset();

    HAL_NVIC_SetPriority(EXTI1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static uint32_t SBUS_SoftEstimateBitCycles(void)
{
    uint32_t threshold = s_data.soft_run_count / 50U;
    uint32_t cycles = 0U;

    if (threshold < 2U)
        threshold = 2U;

    for (uint32_t i = 1U; i <= SBUS_SOFT_HIST_MAX_US; i++)
    {
        if (s_soft_run_hist[i] >= threshold)
        {
            cycles = s_soft_run_hist_sum[i] / s_soft_run_hist[i];
            break;
        }
    }

    if (cycles == 0U)
        cycles = s_data.soft_min_cycles;
    if (cycles == 0U)
        cycles = s_soft_bit_cycles;

    return cycles;
}

static void SBUS_ApplyConfig(uint8_t index);

void SBUS_Init(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
        return;

    s_huart = huart;
    memset((void *)&s_data, 0, sizeof(s_data));
    /* The board already inverts SBUS, so use the UART hardware first. */
    SBUS_ApplyConfig(SBUS_MODE_UART_BASE);
}

static void SBUS_ApplyConfig(uint8_t index)
{
    if ((s_huart == NULL) || (index >= SBUS_MODE_COUNT))
        return;

    SBUS_ResetFrameState();
    memset((void *)&s_data, 0, sizeof(s_data));
    s_data.config_index = index;
    s_data.config_locked = 0U;
    s_config_trial_start = HAL_GetTick();
    s_config_trial_frames = 0U;
    s_last_frame_tick = s_config_trial_start;

    if (index == SBUS_MODE_SOFT_100K_12)
    {
        SBUS_SoftInit(100000U, 12U, 0U);
    }
    else if (index == SBUS_MODE_SOFT_100K_11)
    {
        SBUS_SoftInit(100000U, 11U, 0U);
    }
    else if (index == SBUS_MODE_SOFT_200K_12)
    {
        SBUS_SoftInit(200000U, 12U, 0U);
    }
    else if (index == SBUS_MODE_SOFT_200K_11)
    {
        SBUS_SoftInit(200000U, 11U, 0U);
    }
    else if (index == SBUS_MODE_SOFT_ADAPT_12)
    {
        SBUS_SoftInit(100000U, 12U, 1U);
    }
    else if (index == SBUS_MODE_SOFT_ADAPT_11)
    {
        SBUS_SoftInit(100000U, 11U, 1U);
    }
    else
    {
        const SBUS_Config_t *config = &s_configs[index - SBUS_MODE_UART_BASE];

        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        __HAL_UART_DISABLE_IT(s_huart, UART_IT_RXNE | UART_IT_PE);
        __HAL_UART_DISABLE_IT(s_huart, UART_IT_ERR);

        /*
         * A previous software-decode trial deinitializes PA1.  Deinitialize
         * the UART handle before reinitializing so HAL_UART_MspInit restores
         * PA1 as UART4_RX instead of only rewriting UART registers.
         */
        if (HAL_UART_DeInit(s_huart) != HAL_OK)
        {
            Error_Handler();
        }

        s_huart->Init.BaudRate = config->baud_rate;
        s_huart->Init.WordLength = config->word_length;
        s_huart->Init.Parity = config->parity;
        s_huart->Init.StopBits = config->stop_bits;

        if (HAL_UART_Init(s_huart) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_UART_CLEAR_PEFLAG(s_huart);
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_PE);
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_ERR);
        HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
}

void SBUS_Process(void)
{
    uint8_t frame[SBUS_FRAME_LEN];
    uint8_t ready;
    uint32_t primask;
    uint32_t now;
    uint32_t frame_count_before = s_data.frame_count;

    /* Take an atomic snapshot because s_frame is written in the UART ISR. */
    primask = __get_PRIMASK();
    __disable_irq();
    ready = s_data.frame_ready;
    if (ready != 0U)
    {
        memcpy(frame, s_frame, SBUS_FRAME_LEN);
        s_data.frame_ready = 0U;
    }
    __set_PRIMASK(primask);

    if (ready != 0U)
        SBUS_ParseFrame(frame);

    now = HAL_GetTick();
    if (s_data.frame_count != frame_count_before)
        s_last_frame_tick = now;

    if ((s_soft_adaptive != 0U) &&
        (s_soft_refined == 0U) &&
        ((now - s_config_trial_start) >= SBUS_ADAPT_MEASURE_MS) &&
        (s_data.soft_min_cycles > 0U))
    {
        s_soft_bit_cycles = SBUS_SoftEstimateBitCycles();
        s_soft_refined = 1U;
        SBUS_SoftReset();
        s_config_trial_frames = s_data.frame_count;
        s_config_trial_start = HAL_GetTick();
        s_last_frame_tick = s_config_trial_start;
    }

    if ((s_data.config_locked != 0U) &&
        ((now - s_last_frame_tick) >= SBUS_CONFIG_TEST_MS))
    {
        uint8_t next = (uint8_t)((s_data.config_index + 1U) % SBUS_MODE_COUNT);
        SBUS_ApplyConfig(next);
    }
    else if ((s_data.config_locked == 0U) &&
             ((now - s_config_trial_start) >= SBUS_CONFIG_TEST_MS))
    {
        uint32_t frames = s_data.frame_count - s_config_trial_frames;

        if ((frames >= SBUS_CONFIG_MIN_FRAMES) &&
            (frames <= 200U) &&
            ((now - s_last_frame_tick) <= 100U))
        {
            s_data.config_locked = 1U;
        }
        else
        {
            uint8_t next = (uint8_t)((s_data.config_index + 1U) % SBUS_MODE_COUNT);
            SBUS_ApplyConfig(next);
        }
    }
}

uint8_t SBUS_FrameReady(void)       { return s_data.frame_ready; }
void SBUS_ClearFrameReady(void)     { s_data.frame_ready = 0; }
uint8_t SBUS_Failsafe(void)         { return s_data.failsafe; }
const volatile SBUS_Data_t *SBUS_GetData(void) { return &s_data; }

const char *SBUS_GetConfigName(void)
{
    switch (s_data.config_index)
    {
        case SBUS_MODE_SOFT_100K_12:
            return "SW 100k 12bit";
        case SBUS_MODE_SOFT_100K_11:
            return "SW 100k 11bit";
        case SBUS_MODE_SOFT_200K_12:
            return "SW 200k 12bit";
        case SBUS_MODE_SOFT_200K_11:
            return "SW 200k 11bit";
        case SBUS_MODE_SOFT_ADAPT_12:
            return "SW adapt 12bit";
        case SBUS_MODE_SOFT_ADAPT_11:
            return "SW adapt 11bit";
        default:
            if ((s_data.config_index >= SBUS_MODE_UART_BASE) &&
                (s_data.config_index < SBUS_MODE_COUNT))
            {
                return s_configs[s_data.config_index - SBUS_MODE_UART_BASE].name;
            }
            return "invalid";
    }
}

uint16_t SBUS_GetChannel(uint8_t index)
{
    if (index >= SBUS_CHANNEL_COUNT) return 0;
    return s_data.channel[index];
}

uint16_t SBUS_GetAmplitude(void)
{
    uint16_t raw = s_data.channel[2];
    int32_t v = 20 + (int32_t)(raw - SBUS_RAW_MIN) * (80 - 20)
              / (int32_t)(SBUS_RAW_MAX - SBUS_RAW_MIN);
    return (uint16_t)SBUS_Constrain((uint16_t)v, 20, 80);
}

float SBUS_GetTurnAngle(void)
{
    uint16_t raw = s_data.channel[0];
    return -10.0f + (float)(raw - SBUS_RAW_MIN) * 20.0f
                 / (float)(SBUS_RAW_MAX - SBUS_RAW_MIN);
}

void UART4_IRQHandler(void)
{
    uint32_t status = UART4->SR;
    uint32_t error_flags = status &
                           (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE);
    uint8_t has_data = ((status & USART_SR_RXNE) != 0U) ? 1U : 0U;
    uint16_t raw = 0U;
    uint8_t b;

    /* Reading SR followed by DR clears PE/FE/NE/ORE on STM32F4. */
    if ((has_data != 0U) || (error_flags != 0U))
        raw = (uint16_t)(UART4->DR & 0x01FFU);

    b = (uint8_t)(raw & 0x00FFU);

    if (error_flags != 0U)
    {
        s_data.uart_error = SBUS_HalErrorFromStatus(status);
        s_data.uart_status = status;
        s_data.err_count++;
        if ((status & USART_SR_PE) != 0U)  s_data.pe_count++;
        if ((status & USART_SR_NE) != 0U)  s_data.ne_count++;
        if ((status & USART_SR_FE) != 0U)  s_data.fe_count++;
        if ((status & USART_SR_ORE) != 0U) s_data.ore_count++;
    }

    if (has_data != 0U)
    {
        s_data.rx_byte_count++;
        s_data.last_rx_byte = b;

        /*
         * NE/FE/PE describe the framing/parity around the byte; the received
         * data bits are still valid. Keep them so a marginal inverter or a
         * non-standard stop-bit format does not prevent frame synchronization.
         * ORE means a byte may have been lost, so that case must resync.
         */
        if ((error_flags & USART_SR_ORE) == 0U)
        {
            SBUS_ConsumeByte(b);
        }
        else
        {
            SBUS_ResetFrameState();
        }
    }
}

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == GPIO_PIN_1) &&
        (s_data.config_index <= SBUS_MODE_SOFT_ADAPT_11))
    {
        SBUS_SoftEdge();
    }
}
