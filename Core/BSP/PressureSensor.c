#include "bsp_delay.h"
#include "PressureSensor.h"

#include "i2c.h"

#include <math.h>
#include <string.h>


/*
 * ============================================================
 * 外部I2C句柄
 * ============================================================
 */

extern I2C_HandleTypeDef hi2c1;


/*
 * ============================================================
 * MS5611内部变量
 * ============================================================
 */


/*
 * 当前I2C地址
 *
 * HAL格式：
 *
 * 0xEC -> 0x76
 * 0xEE -> 0x77
 */
static uint16_t g_ms5611_address = 0;

/* Saved before a retry/reinitialisation so application diagnostics stay useful. */
static uint32_t g_last_i2c_error = HAL_I2C_ERROR_NONE;

static PressureSensor_I2CStage_t g_last_i2c_stage =
    PRESSURE_SENSOR_I2C_STAGE_NONE;


/*
 * PROM校准参数
 *
 * PROM[1] = C1
 * PROM[2] = C2
 * PROM[3] = C3
 * PROM[4] = C4
 * PROM[5] = C5
 * PROM[6] = C6
 */
static uint16_t g_prom[8];


/*
 * ============================================================
 * 相对高度变量
 * ============================================================
 */


/*
 * 相对高度零点压力
 *
 * 单位：Pa
 */
static float g_reference_pressure = 0.0f;


/*
 * 零点是否有效
 */
static uint8_t g_reference_valid = 0;


/*
 * ============================================================
 * 压力中值滤波缓存
 * ============================================================
 */

static float g_median_buffer[
    PRESSURE_MEDIAN_SIZE
];

static uint8_t g_median_index = 0;

static uint8_t g_median_count = 0;


/*
 * ============================================================
 * 压力平均滤波
 * ============================================================
 */

static float g_average_buffer[
    PRESSURE_AVERAGE_SIZE
];

static uint8_t g_average_index = 0;

static uint8_t g_average_count = 0;


/*
 * 当前滤波压力
 */
static float g_filtered_pressure = 0.0f;


/*
 * ============================================================
 * 高度滤波
 * ============================================================
 */

static float g_filtered_altitude = 0.0f;

static uint8_t g_altitude_filter_initialized = 0;


/*
 * ============================================================
 * 内部函数声明
 * ============================================================
 */

static PressureSensor_Status_t
MS5611_WriteCommand(
    uint8_t command
);

static HAL_StatusTypeDef
MS5611_I2C_Recover(void);

static HAL_StatusTypeDef
MS5611_I2C_SelectAddress(void);

static HAL_StatusTypeDef
MS5611_I2C_Transmit(
    uint8_t *data,
    uint16_t size
);

static HAL_StatusTypeDef
MS5611_I2C_Receive(
    uint8_t *data,
    uint16_t size
);


static PressureSensor_Status_t
MS5611_ReadPROM(
    uint8_t index,
    uint16_t *value
);


static PressureSensor_Status_t
MS5611_ReadADC(
    uint32_t *adc
);


static PressureSensor_Status_t
MS5611_Convert(
    uint8_t command,
    uint32_t *adc
);


static uint8_t
MS5611_CRC4(
    uint16_t *prom
);


static float
PressureSensor_MedianFilter(
    float value
);


static float
PressureSensor_AverageFilter(
    float value
);


static float
PressureSensor_PressureToRelativeAltitude(
    float pressure_pa
);


/*
 * Retry a failed transfer after reinitialising I2C1.  A short electrical
 * disturbance during an ADC conversion must not invalidate the whole
 * altitude-reference sampling sequence.
 */
static HAL_StatusTypeDef
MS5611_I2C_SelectAddress(void)
{
    static const uint16_t addresses[] =
    {
        MS5611_I2C_ADDR_76,
        MS5611_I2C_ADDR_77
    };

    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t i = 0U;
         i < (sizeof(addresses) / sizeof(addresses[0]));
         i++)
    {
        status = HAL_I2C_IsDeviceReady(
            &hi2c1,
            addresses[i],
            2U,
            MS5611_I2C_TIMEOUT_MS
        );

        if (status == HAL_OK)
        {
            g_ms5611_address = addresses[i];
            g_last_i2c_error = HAL_I2C_ERROR_NONE;
            return HAL_OK;
        }

        g_last_i2c_error = HAL_I2C_GetError(&hi2c1);
    }

    return status;
}


static HAL_StatusTypeDef
MS5611_I2C_Recover(void)
{
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    BSP_DelayMs(2);

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        g_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        return HAL_ERROR;
    }

    BSP_DelayMs(2);

    return MS5611_I2C_SelectAddress();
}


static HAL_StatusTypeDef
MS5611_I2C_Transmit(
    uint8_t *data,
    uint16_t size
)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U;
         attempt < MS5611_I2C_RETRY_COUNT;
         attempt++)
    {
        status = HAL_I2C_Master_Transmit(
            &hi2c1,
            g_ms5611_address,
            data,
            size,
            MS5611_I2C_TIMEOUT_MS
        );

        if (status == HAL_OK)
        {
            g_last_i2c_error = HAL_I2C_ERROR_NONE;
            return HAL_OK;
        }

        g_last_i2c_error = HAL_I2C_GetError(&hi2c1);

        if ((attempt + 1U) < MS5611_I2C_RETRY_COUNT)
        {
            (void)MS5611_I2C_Recover();
        }
    }

    return status;
}


static HAL_StatusTypeDef
MS5611_I2C_Receive(
    uint8_t *data,
    uint16_t size
)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t attempt = 0U;
         attempt < MS5611_I2C_RETRY_COUNT;
         attempt++)
    {
        status = HAL_I2C_Master_Receive(
            &hi2c1,
            g_ms5611_address,
            data,
            size,
            MS5611_I2C_TIMEOUT_MS
        );

        if (status == HAL_OK)
        {
            g_last_i2c_error = HAL_I2C_ERROR_NONE;
            return HAL_OK;
        }

        g_last_i2c_error = HAL_I2C_GetError(&hi2c1);

        if ((attempt + 1U) < MS5611_I2C_RETRY_COUNT)
        {
            (void)MS5611_I2C_Recover();
        }
    }

    return status;
}


/*
 * ============================================================
 * 写MS5611命令
 * ============================================================
 */

static PressureSensor_Status_t
MS5611_WriteCommand(
    uint8_t command
)
{
    HAL_StatusTypeDef status;

    if (command == MS5611_CMD_RESET)
    {
        g_last_i2c_stage = PRESSURE_SENSOR_I2C_STAGE_RESET;
    }


    status = MS5611_I2C_Transmit(&command, 1U);


    if (status != HAL_OK)
    {
        return PRESSURE_SENSOR_I2C_ERROR;
    }


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 读取PROM
 * ============================================================
 */

static PressureSensor_Status_t
MS5611_ReadPROM(
    uint8_t index,
    uint16_t *value
)
{
    uint8_t command;

    uint8_t buffer[2];

    HAL_StatusTypeDef status;


    if (value == NULL)
    {
        return PRESSURE_SENSOR_INVALID_PARAMETER;
    }


    /*
     * PROM地址：
     *
     * 0xA0
     * 0xA2
     * 0xA4
     * ...
     * 0xAE
     */
    command =
        MS5611_CMD_PROM_BASE
        +
        index * 2U;

    g_last_i2c_stage = PRESSURE_SENSOR_I2C_STAGE_PROM_COMMAND;


    /*
     * 发送PROM地址
     */
    status = MS5611_I2C_Transmit(&command, 1U);


    if (status != HAL_OK)
    {
        return PRESSURE_SENSOR_I2C_ERROR;
    }


    /*
     * 读取16bit PROM
     */
    g_last_i2c_stage = PRESSURE_SENSOR_I2C_STAGE_PROM_DATA;

    status = MS5611_I2C_Receive(buffer, 2U);


    if (status != HAL_OK)
    {
        return PRESSURE_SENSOR_I2C_ERROR;
    }


    *value =
        ((uint16_t)buffer[0] << 8)
        |
        buffer[1];


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 读取ADC
 * ============================================================
 */

static PressureSensor_Status_t
MS5611_ReadADC(
    uint32_t *adc
)
{
    uint8_t command =
        MS5611_CMD_ADC_READ;

    uint8_t buffer[3];

    HAL_StatusTypeDef status;


    if (adc == NULL)
    {
        return PRESSURE_SENSOR_INVALID_PARAMETER;
    }


    /*
     * ADC Read
     */
    status = MS5611_I2C_Transmit(&command, 1U);


    if (status != HAL_OK)
    {
        return PRESSURE_SENSOR_I2C_ERROR;
    }


    /*
     * MS5611 ADC = 24bit
     */
    status = MS5611_I2C_Receive(buffer, 3U);


    if (status != HAL_OK)
    {
        return PRESSURE_SENSOR_I2C_ERROR;
    }


    *adc =
        ((uint32_t)buffer[0] << 16)
        |
        ((uint32_t)buffer[1] << 8)
        |
        buffer[2];


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 启动D1/D2转换并读取
 * ============================================================
 */

static PressureSensor_Status_t
MS5611_Convert(
    uint8_t command,
    uint32_t *adc
)
{
    PressureSensor_Status_t status;


    /*
     * 启动转换
     */
    g_last_i2c_stage =
        ((command & 0x10U) == 0U)
        ? PRESSURE_SENSOR_I2C_STAGE_D1_COMMAND
        : PRESSURE_SENSOR_I2C_STAGE_D2_COMMAND;

    status =
        MS5611_WriteCommand(
            command
        );


    if (status != PRESSURE_SENSOR_OK)
    {
        return status;
    }


    /* OSR4096 max 9.04 ms; OSR2048 max 4.54 ms. BSP delay yields
     * and includes tick-phase margin when running in a FreeRTOS task. */
    BSP_DelayMs((command & 0x0FU) == 0x06U ? 5U : 10U);

    /*
     * 读取ADC
     */
    g_last_i2c_stage =
        ((command & 0x10U) == 0U)
        ? PRESSURE_SENSOR_I2C_STAGE_D1_ADC
        : PRESSURE_SENSOR_I2C_STAGE_D2_ADC;

    return MS5611_ReadADC(adc);
}


/*
 * ============================================================
 * CRC4
 *
 * 按数据手册PROM CRC算法实现
 * ============================================================
 */

static uint8_t
MS5611_CRC4(
    uint16_t *prom
)
{
    uint16_t n_rem = 0;

    uint16_t crc_read;

    uint8_t n_bit;


    /*
     * 保存原始CRC
     */
    crc_read =
        prom[7] & 0x000F;


    /*
     * 清除CRC位
     */
    prom[7] &= 0xFFF0;


    /*
     * CRC计算
     */
    for (uint8_t cnt = 0;
         cnt < 16;
         cnt++)
    {
        if ((cnt & 1U) != 0)
        {
            n_rem ^=
                prom[cnt >> 1]
                & 0x00FF;
        }
        else
        {
            n_rem ^=
                prom[cnt >> 1]
                >> 8;
        }


        for (n_bit = 8;
             n_bit > 0;
             n_bit--)
        {
            if ((n_rem & 0x8000U) != 0)
            {
                n_rem =
                    (n_rem << 1)
                    ^ 0x3000U;
            }
            else
            {
                n_rem <<= 1;
            }
        }
    }


    /*
     * 恢复CRC
     */
    prom[7] =
        (prom[7] & 0xFFF0)
        |
        crc_read;


    return
        (uint8_t)(
            (n_rem >> 12)
            & 0x000F
        );
}


/*
 * ============================================================
 * 初始化MS5611
 * ============================================================
 */

PressureSensor_Status_t
PressureSensor_Init(void)
{
    HAL_StatusTypeDef hal_status;

    PressureSensor_Status_t status;


    g_last_i2c_error = HAL_I2C_ERROR_NONE;
    g_last_i2c_stage = PRESSURE_SENSOR_I2C_STAGE_NONE;

    /*
     * ========================================================
     * 1. 检查0x76
     * ========================================================
     */

    g_ms5611_address =
        MS5611_I2C_ADDR_76;


    hal_status =
        HAL_I2C_IsDeviceReady(
            &hi2c1,
            MS5611_I2C_ADDR_76,
            3,
            100
        );


    /*
     * ========================================================
     * 2. 如果0x76不存在，检查0x77
     * ========================================================
     */

    if (hal_status != HAL_OK)
    {
        g_ms5611_address =
            MS5611_I2C_ADDR_77;


        hal_status =
            HAL_I2C_IsDeviceReady(
                &hi2c1,
                MS5611_I2C_ADDR_77,
                3,
                100
            );


        if (hal_status != HAL_OK)
        {
            g_last_i2c_error = HAL_I2C_GetError(&hi2c1);
            g_ms5611_address = 0;

            return PRESSURE_SENSOR_I2C_ERROR;
        }
    }


    /*
     * ========================================================
     * 3. Reset
     * ========================================================
     */

    status =
        MS5611_WriteCommand(
            MS5611_CMD_RESET
        );


    if (status != PRESSURE_SENSOR_OK)
    {
        return status;
    }


    /*
     * 数据手册：
     *
     * Reset后等待至少2.8ms
     *
     * 这里等待5ms。
     */
    BSP_DelayMs(5);


    /*
     * ========================================================
     * 4. 读取PROM
     * ========================================================
     */

    for (uint8_t i = 0;
         i < 8;
         i++)
    {
        status =
            MS5611_ReadPROM(
                i,
                &g_prom[i]
            );


        if (status != PRESSURE_SENSOR_OK)
        {
            return PRESSURE_SENSOR_PROM_ERROR;
        }
    }


    /*
     * ========================================================
     * 5. CRC校验
     * ========================================================
     */

    uint8_t crc_calculated =
        MS5611_CRC4(g_prom);


    uint8_t crc_stored =
        g_prom[7] & 0x0F;


    if (crc_calculated != crc_stored)
    {
        return PRESSURE_SENSOR_CRC_ERROR;
    }


    /*
     * ========================================================
     * 6. 检查C1~C6
     * ========================================================
     */

    for (uint8_t i = 1;
         i <= 6;
         i++)
    {
        if (g_prom[i] == 0 ||
            g_prom[i] == 0xFFFF)
        {
            return PRESSURE_SENSOR_PROM_ERROR;
        }
    }


    /*
     * 清除滤波器
     */
    PressureSensor_ResetAltitudeFilter();


    /*
     * 没有零点
     */
    g_reference_pressure = 0.0f;

    g_reference_valid = 0;


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 读取MS5611
 * ============================================================
 */

static PressureSensor_Status_t PressureSensor_ReadWithOSR(
    PressureSensor_Data_t *data, uint8_t d1_command, uint8_t d2_command);

PressureSensor_Status_t PressureSensor_Read(PressureSensor_Data_t *data)
{
    return PressureSensor_ReadWithOSR(data, MS5611_CMD_D1_4096, MS5611_CMD_D2_4096);
}

PressureSensor_Status_t PressureSensor_ReadPeriodic(PressureSensor_Data_t *data)
{
    return PressureSensor_ReadWithOSR(data, MS5611_CMD_D1_2048, MS5611_CMD_D2_2048);
}

static PressureSensor_Status_t PressureSensor_ReadWithOSR(
    PressureSensor_Data_t *data, uint8_t d1_command, uint8_t d2_command)
{
    uint32_t D1;

    uint32_t D2;


    int64_t dT;

    int64_t OFF;

    int64_t SENS;


    int64_t T2;

    int64_t OFF2;

    int64_t SENS2;


    int32_t TEMP;

    int32_t pressure;


    PressureSensor_Status_t status;


    if (data == NULL)
    {
        return PRESSURE_SENSOR_INVALID_PARAMETER;
    }


    if (g_ms5611_address == 0)
    {
        return PRESSURE_SENSOR_ERROR;
    }


    /*
     * ========================================================
     * 读取D1
     * ========================================================
     */

    status =
        MS5611_Convert(
            d1_command,
            &D1
        );


    if (status != PRESSURE_SENSOR_OK)
    {
        return status;
    }


    /*
     * ========================================================
     * 读取D2
     * ========================================================
     */

    status =
        MS5611_Convert(
            d2_command,
            &D2
        );


    if (status != PRESSURE_SENSOR_OK)
    {
        return status;
    }


    /*
     * 保存原始数据
     */
    data->D1 = D1;

    data->D2 = D2;


    /*
     * ========================================================
     * dT
     *
     * dT = D2 - C5 * 2^8
     * ========================================================
     */

    dT =
        (int64_t)D2
        -
        (
            (int64_t)g_prom[5]
            << 8
        );


    /*
     * ========================================================
     * TEMP
     *
     * TEMP = 2000 + dT*C6/2^23
     *
     * 单位：0.01°C
     * ========================================================
     */

    TEMP =
        2000
        +
        (
            dT *
            (int64_t)g_prom[6]
        )
        /
        8388608LL;


    /*
     * ========================================================
     * OFF
     *
     * OFF =
     * C2*2^16
     * +
     * C4*dT/2^7
     * ========================================================
     */

    OFF =
        (
            (int64_t)g_prom[2]
            << 16
        )
        +
        (
            (int64_t)g_prom[4]
            * dT
        )
        /
        128LL;


    /*
     * ========================================================
     * SENS
     *
     * SENS =
     * C1*2^15
     * +
     * C3*dT/2^8
     * ========================================================
     */

    SENS =
        (
            (int64_t)g_prom[1]
            << 15
        )
        +
        (
            (int64_t)g_prom[3]
            * dT
        )
        /
        256LL;


    /*
     * ========================================================
     * 二阶温度补偿
     *
     * 完全按照数据手册Figure 3。
     * ========================================================
     */

    T2 = 0;

    OFF2 = 0;

    SENS2 = 0;


    /*
     * TEMP < 20°C
     */
    if (TEMP < 2000)
    {
        /*
         * T2 =
         * dT^2 / 2^31
         */
        T2 =
            (
                dT * dT
            )
            /
            2147483648LL;


        /*
         * OFF2 =
         * 5*(TEMP-2000)^2 / 2
         */
        OFF2 =
            5LL
            *
            (
                (int64_t)TEMP - 2000LL
            )
            *
            (
                (int64_t)TEMP - 2000LL
            )
            /
            2LL;


        /*
         * SENS2 =
         * 5*(TEMP-2000)^2 / 4
         */
        SENS2 =
            5LL
            *
            (
                (int64_t)TEMP - 2000LL
            )
            *
            (
                (int64_t)TEMP - 2000LL
            )
            /
            4LL;


        /*
         * TEMP < -15°C
         */
        if (TEMP < -1500)
        {
            /*
             * OFF2 +=
             * 7*(TEMP+1500)^2
             */
            OFF2 +=
                7LL
                *
                (
                    (int64_t)TEMP + 1500LL
                )
                *
                (
                    (int64_t)TEMP + 1500LL
                );


            /*
             * SENS2 +=
             * 11*(TEMP+1500)^2 / 2
             */
            SENS2 +=
                11LL
                *
                (
                    (int64_t)TEMP + 1500LL
                )
                *
                (
                    (int64_t)TEMP + 1500LL
                )
                /
                2LL;
        }
    }


    /*
     * 应用二阶补偿
     */
    TEMP -= (int32_t)T2;

    OFF -= OFF2;

    SENS -= SENS2;


    /*
     * ========================================================
     * 计算压力
     *
     * P =
     * (D1*SENS/2^21 - OFF)/2^15
     *
     * 结果单位：
     * 0.01mbar
     * ========================================================
     */

    pressure =
        (
            (
                (int64_t)D1
                *
                SENS
            )
            /
            2097152LL
            -
            OFF
        )
        /
        32768LL;


    /*
     * ========================================================
     * 温度
     * ========================================================
     */

    data->Temperature_C =
        (float)TEMP / 100.0f;


    /*
     * ========================================================
     * 压力
     *
     * pressure单位：
     *
     * 0.01mbar
     *
     * 因为：
     *
     * 1mbar = 100Pa
     *
     * 所以：
     *
     * pressure * 0.01 * 100
     *
     * = pressure Pa
     *
     * ========================================================
     */

    data->Pressure_mbar =
        (float)pressure / 100.0f;


    data->Pressure_Pa =
        (float)pressure;

    static float pressure_filter=0;

    if(pressure_filter==0)
    {
        pressure_filter=data->Pressure_Pa;
    }
    else
    {
        pressure_filter =
            pressure_filter*0.9f
            +
            data->Pressure_Pa*0.1f;
    }


    data->Pressure_Pa =
    pressure_filter;

    /*
     * ========================================================
     * 绝对高度
     *
     * 标准大气压：
     *
     * 101325Pa
     * ========================================================
     */

    if (data->Pressure_Pa > 0.0f)
    {
        data->Altitude =
            44330.0f
            *
            (
                1.0f
                -
                powf(
                    data->Pressure_Pa
                    /
                    STANDARD_PRESSURE_PA,
                    0.1902949f
                )
            );
    }
    else
    {
        data->Altitude = 0.0f;
    }


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 5点中值滤波
 * ============================================================
 */

static float PressureSensor_MedianFilter(float value)
{

    float temp[PRESSURE_MEDIAN_SIZE];


    g_median_buffer[g_median_index]=value;


    g_median_index++;

    if(g_median_index>=PRESSURE_MEDIAN_SIZE)
        g_median_index=0;


    if(g_median_count<PRESSURE_MEDIAN_SIZE)
        g_median_count++;



    memcpy(
        temp,
        g_median_buffer,
        sizeof(temp)
    );


    uint8_t size=g_median_count;


    for(uint8_t i=0;i<size-1;i++)
    {
        for(uint8_t j=0;j<size-1-i;j++)
        {
            if(temp[j]>temp[j+1])
            {
                float t=temp[j];

                temp[j]=temp[j+1];

                temp[j+1]=t;
            }
        }
    }


    /*
     * 数据不足时
     * 返回平均中心
     */

    return temp[size/2];

}


/*
 * ============================================================
 * 压力滑动平均
 * ============================================================
 */

static float
PressureSensor_AverageFilter(
    float value
)
{
    float sum = 0.0f;


    /*
     * 加入缓存
     */
    g_average_buffer[
        g_average_index
    ] = value;


    g_average_index++;


    if (g_average_index >=
        PRESSURE_AVERAGE_SIZE)
    {
        g_average_index = 0;
    }


    if (g_average_count <
        PRESSURE_AVERAGE_SIZE)
    {
        g_average_count++;
    }


    /*
     * 求和
     */
    for (uint8_t i = 0;
         i < g_average_count;
         i++)
    {
        sum +=
            g_average_buffer[i];
    }


    /*
     * 求平均
     */
    return
        sum /
        (float)g_average_count;
}


/*
 * ============================================================
 * 压力 → 相对高度
 * ============================================================
 */

static float
PressureSensor_PressureToRelativeAltitude(
    float pressure_pa
)
{
    float ratio;


    if (!g_reference_valid)
    {
        return 0.0f;
    }


    if (pressure_pa <= 0.0f)
    {
        return 0.0f;
    }


    if (g_reference_pressure <= 0.0f)
    {
        return 0.0f;
    }


    /*
     * 当前压力 / 零点压力
     */
    ratio =
        pressure_pa
        /
        g_reference_pressure;


    /*
     * 相对高度：
     *
     * h =
     *
     * 44330 *
     * [1-(P/P0)^0.1902949]
     *
     * 这里的P0不是101325Pa，
     *
     * 而是启动时实际测量得到的压力。
     */
    return
        44330.0f
        *
        (
            1.0f
            -
            powf(
                ratio,
                0.1902949f
            )
        );
}


/*
 * ============================================================
 * 设置相对高度零点
 * ============================================================
 */

PressureSensor_Status_t
PressureSensor_SetAltitudeReference(void)
{
    PressureSensor_Data_t data;


    float pressure;


    float sum = 0.0f;


    float min_pressure =
        1000000000.0f;


    float max_pressure =
        -1000000000.0f;


    /*
     * 清空旧滤波器
     */
    PressureSensor_ResetAltitudeFilter();


    /*
     * ========================================================
     * 连续采样
     * ========================================================
     */

    uint16_t valid_samples = 0U;

    PressureSensor_Status_t last_error =
        PRESSURE_SENSOR_I2C_ERROR;

    for (uint16_t attempt = 0U;
         (attempt < REFERENCE_MAX_ATTEMPTS) &&
         (valid_samples < REFERENCE_SAMPLE_COUNT);
         attempt++)
    {
        PressureSensor_Status_t status;


        status =
            PressureSensor_Read(
                &data
            );


        if (status != PRESSURE_SENSOR_OK)
        {
            last_error = status;
            BSP_DelayMs(REFERENCE_RETRY_DELAY_MS);
            continue;
        }


        pressure =
            data.Pressure_Pa;


        /*
         * 累加
         */
        sum += pressure;


        /*
         * 最小值
         */
        if (pressure <
            min_pressure)
        {
            min_pressure =
                pressure;
        }


        /*
         * 最大值
         */
        if (pressure >
            max_pressure)
        {
            max_pressure =
                pressure;
        }

        valid_samples++;
    }


    if (valid_samples < REFERENCE_SAMPLE_COUNT)
    {
        g_reference_valid = 0;

        return last_error;
    }


    /*
     * 去掉一个最大值
     */
    sum -= max_pressure;


    /*
     * 去掉一个最小值
     */
    sum -= min_pressure;


    /*
     * 求零点压力
     */
    g_reference_pressure =
        sum
        /
        (float)(
            REFERENCE_SAMPLE_COUNT - 2
        );


    /*
     * 标记有效
     */
    g_reference_valid = 1;


    /*
     * 清除滤波器
     *
     * 注意：
     *
     * 不清除reference_pressure
     */
    PressureSensor_ResetAltitudeFilter();


    /*
     * 让高度从0开始
     */
    g_filtered_altitude = 0.0f;


    g_altitude_filter_initialized = 1;


    return PRESSURE_SENSOR_OK;
}


/*
 * ============================================================
 * 根据压力计算相对高度
 * ============================================================
 */

float
PressureSensor_CalculateRelativeAltitude(
    float pressure_pa
)
{
    float median_pressure;

    float filtered_pressure;

    float raw_altitude;

    float alpha;

    float difference;


    if (!g_reference_valid)
    {
        return 0.0f;
    }


    /*
     * ========================================================
     * 第1级：
     *
     * 5点中值滤波
     *
     * 主要去除突然的尖峰。
     * ========================================================
     */

    median_pressure =
        PressureSensor_MedianFilter(
            pressure_pa
        );


    /*
     * ========================================================
     * 第2级：
     *
     * 滑动平均
     * ========================================================
     */

    filtered_pressure =
        PressureSensor_AverageFilter(
            median_pressure
        );


    /*
     * 保存滤波后的压力
     */
    g_filtered_pressure =
        filtered_pressure;


    /*
     * ========================================================
     * 第3级：
     *
     * 压力 → 相对高度
     * ========================================================
     */

    raw_altitude =
        PressureSensor_PressureToRelativeAltitude(
            filtered_pressure
        );


    /*
     * ========================================================
     * 第4级：
     *
     * 自适应低通
     *
     * 高度变化明显：
     *
     *      alpha = 0.25
     *
     * 静止：
     *
     *      alpha = 0.08
     *
     * ========================================================
     */

    difference =
        raw_altitude
        -
        g_filtered_altitude;


    if (fabsf(difference)
        >
        ALTITUDE_CHANGE_THRESHOLD)
    {
        /*
         * 有明显运动
         *
         * 提高响应速度
         */
        alpha =
            ALTITUDE_ALPHA_FAST;
    }
    else
    {
        /*
         * 基本静止
         *
         * 提高稳定性
         */
        alpha =
            ALTITUDE_ALPHA_SLOW;
    }


    /*
     * IIR低通
     */
    g_filtered_altitude =
        g_filtered_altitude
        +
        alpha
        *
        difference;


    /*
     * ========================================================
     * 第5级：
     *
     * 小死区
     * ========================================================
     */

    if (fabsf(g_filtered_altitude)
        <
        ALTITUDE_DEADBAND)
    {
        g_filtered_altitude =
            0.0f;
    }


    return g_filtered_altitude;
}


/*
 * ============================================================
 * 直接获取相对高度
 * ============================================================
 */

float
PressureSensor_GetRelativeAltitude(void)
{
    PressureSensor_Data_t data;


    /*
     * 读取传感器
     */
    if (PressureSensor_Read(&data)
        != PRESSURE_SENSOR_OK)
    {
        /*
         * 读取失败：
         *
         * 保持上一次高度。
         */
        return g_filtered_altitude;
    }


    /*
     * 计算相对高度
     */
    return
        PressureSensor_CalculateRelativeAltitude(
            data.Pressure_Pa
        );
}


/*
 * ============================================================
 * 获取基准压力
 * ============================================================
 */

float
PressureSensor_GetReferencePressure(void)
{
    return g_reference_pressure;
}


/*
 * ============================================================
 * 判断零点是否有效
 * ============================================================
 */

uint8_t
PressureSensor_IsReferenceValid(void)
{
    return g_reference_valid;
}


/*
 * ============================================================
 * 获取滤波后的压力
 * ============================================================
 */

float
PressureSensor_GetFilteredPressure(void)
{
    return g_filtered_pressure;
}


/*
 * ============================================================
 * 重置滤波器
 * ============================================================
 */

void
PressureSensor_ResetAltitudeFilter(void)
{
    /*
     * 清除中值滤波
     */
    memset(
        g_median_buffer,
        0,
        sizeof(g_median_buffer)
    );


    g_median_index = 0;

    g_median_count = 0;


    /*
     * 清除平均滤波
     */
    memset(
        g_average_buffer,
        0,
        sizeof(g_average_buffer)
    );


    g_average_index = 0;

    g_average_count = 0;


    /*
     * 清除滤波压力
     */
    g_filtered_pressure = 0.0f;


    /*
     * 清除高度
     */
    g_filtered_altitude = 0.0f;


    g_altitude_filter_initialized = 0;
}


uint32_t
PressureSensor_GetLastI2cError(void)
{
    return g_last_i2c_error;
}


PressureSensor_I2CStage_t
PressureSensor_GetLastI2cStage(void)
{
    return g_last_i2c_stage;
}
