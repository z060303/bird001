#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * ============================================================
 * MS5611-01BA03 气压传感器驱动
 *
 * MCU:
 *      STM32F405
 *
 * I2C:
 *      I2C1
 *
 * 引脚:
 *      PB6 -> SCL
 *      PB7 -> SDA
 *
 * 功能:
 *      1. MS5611初始化
 *      2. PROM校准参数读取
 *      3. CRC4校验
 *      4. 温度读取
 *      5. 压力读取
 *      6. 二阶温度补偿
 *      7. 绝对高度计算
 *      8. 相对高度计算
 *      9. 相对高度零点校准
 *      10. 中值滤波
 *      11. 滑动平均
 *      12. 自适应低通滤波
 *
 * ============================================================
 */


/* ============================================================
 * MS5611 I2C地址
 *
 * 数据手册:
 *
 * 111011Cx
 *
 * CSB = 0:
 *      0x76
 *
 * CSB = 1:
 *      0x77
 *
 * STM32 HAL使用左移后的8bit地址。
 * ============================================================
 */

#define MS5611_I2C_ADDR_76      (0x76U << 1)
#define MS5611_I2C_ADDR_77      (0x77U << 1)

/* I2C transfer and recovery settings. */
#define MS5611_I2C_TIMEOUT_MS   3U
#define MS5611_I2C_RETRY_COUNT  3U


/* ============================================================
 * MS5611命令
 * ============================================================
 */

/* Reset */
#define MS5611_CMD_RESET        0x1EU

/* D1 Pressure - OSR 4096 */
#define MS5611_CMD_D1_4096      0x48U

/* D2 Temperature - OSR 4096 */
#define MS5611_CMD_D2_4096      0x58U

/* OSR2048: maximum conversion time 4.54 ms, used by the 20 ms task. */
#define MS5611_CMD_D1_2048      0x46U
#define MS5611_CMD_D2_2048      0x56U

/* ADC Read */
#define MS5611_CMD_ADC_READ     0x00U

/* PROM */
#define MS5611_CMD_PROM_BASE    0xA0U


/* ============================================================
 * 滤波参数
 * ============================================================
 */

/*
 * 压力中值滤波使用3个点。
 *
 * 3点中值滤波非常适合去除突然出现的异常压力值。
 */
#define PRESSURE_MEDIAN_SIZE        3U


/*
 * 压力平均滤波窗口。
 *
 * 数值越大：
 *      越稳定
 *      响应越慢
 *
 * 这里使用5。
 */
#define PRESSURE_AVERAGE_SIZE       2U


/*
 * 设置相对高度零点时采样40次。
 *
 * 会去除最大值和最小值，
 * 再进行平均。
 */
#define REFERENCE_SAMPLE_COUNT      40U

/* Allow transient I2C failures without aborting the whole reference capture. */
#define REFERENCE_MAX_ATTEMPTS      (REFERENCE_SAMPLE_COUNT * 2U)
#define REFERENCE_RETRY_DELAY_MS    5U


/*
 * 高度快速变化时的滤波系数。
 *
 * 越大响应越快。
 */
#define ALTITUDE_ALPHA_FAST         0.5f


/*
 * 高度稳定时的滤波系数。
 *
 * 越小越稳定。
 */
#define ALTITUDE_ALPHA_SLOW         0.05f


/*
 * 判断是否发生明显高度变化。
 *
 * 3cm。
 */
#define ALTITUDE_CHANGE_THRESHOLD   0.03f


/*
 * 高度死区。
 *
 * 小于2cm直接认为是0。
 */
#define ALTITUDE_DEADBAND           0.02f


/*
 * 标准大气压。
 *
 * 这里只用于绝对高度。
 *
 * 相对高度使用启动时测得的P0，
 * 因此不会强制使用101325Pa。
 */
#define STANDARD_PRESSURE_PA        101325.0f


/* ============================================================
 * 状态码
 * ============================================================
 */

typedef enum
{
    PRESSURE_SENSOR_OK = 0,

    PRESSURE_SENSOR_ERROR,

    PRESSURE_SENSOR_I2C_ERROR,

    PRESSURE_SENSOR_PROM_ERROR,

    PRESSURE_SENSOR_CRC_ERROR,

    PRESSURE_SENSOR_INVALID_PARAMETER

} PressureSensor_Status_t;

typedef enum
{
    PRESSURE_SENSOR_I2C_STAGE_NONE = 0,
    PRESSURE_SENSOR_I2C_STAGE_RESET,
    PRESSURE_SENSOR_I2C_STAGE_PROM_COMMAND,
    PRESSURE_SENSOR_I2C_STAGE_PROM_DATA,
    PRESSURE_SENSOR_I2C_STAGE_D1_COMMAND,
    PRESSURE_SENSOR_I2C_STAGE_D1_ADC,
    PRESSURE_SENSOR_I2C_STAGE_D2_COMMAND,
    PRESSURE_SENSOR_I2C_STAGE_D2_ADC
} PressureSensor_I2CStage_t;


/* ============================================================
 * 数据结构
 * ============================================================
 */

typedef struct
{
    /*
     * 压力
     *
     * 单位：Pa
     */
    float Pressure_Pa;


    /*
     * 压力
     *
     * 单位：mbar
     */
    float Pressure_mbar;


    /*
     * 温度
     *
     * 单位：°C
     */
    float Temperature_C;


    /*
     * 根据标准大气压计算的绝对高度
     *
     * 单位：m
     */
    float Altitude;


    /*
     * 原始D1
     */
    uint32_t D1;


    /*
     * 原始D2
     */
    uint32_t D2;

} PressureSensor_Data_t;


/* ============================================================
 * 初始化
 * ============================================================
 */

/**
 * @brief 初始化MS5611
 *
 * @return:
 *      PRESSURE_SENSOR_OK
 *      PRESSURE_SENSOR_I2C_ERROR
 *      PRESSURE_SENSOR_PROM_ERROR
 *      PRESSURE_SENSOR_CRC_ERROR
 */
PressureSensor_Status_t PressureSensor_Init(void);


/* ============================================================
 * 读取压力和温度
 * ============================================================
 */

/**
 * @brief 读取MS5611压力和温度
 *
 * @param data 输出数据
 *
 * @return 状态
 */
/* D1/D2 at OSR2048; same compensation and units as PressureSensor_Read(). */
PressureSensor_Status_t PressureSensor_ReadPeriodic(PressureSensor_Data_t *data);

PressureSensor_Status_t PressureSensor_Read(
    PressureSensor_Data_t *data
);


/* ============================================================
 * 相对高度零点
 * ============================================================
 */

/**
 * @brief 设置当前高度为0m
 *
 * 函数会自动采样多次，
 * 去除最大值和最小值后求平均。
 *
 * 建议：
 *
 *      板子静止
 *      调用该函数
 *      等待完成
 *
 * @return 状态
 */
PressureSensor_Status_t
PressureSensor_SetAltitudeReference(void);


/* ============================================================
 * 相对高度
 * ============================================================
 */

/**
 * @brief 根据压力计算相对高度
 *
 * @param pressure_pa 当前压力，Pa
 *
 * @return 相对高度，m
 */
float PressureSensor_CalculateRelativeAltitude(
    float pressure_pa
);


/**
 * @brief 读取传感器并直接返回相对高度
 *
 * @return 相对高度，m
 */
float PressureSensor_GetRelativeAltitude(void);


/* ============================================================
 * 基准压力
 * ============================================================
 */

/**
 * @brief 获取当前相对高度零点压力
 *
 * @return P0，单位Pa
 */
float PressureSensor_GetReferencePressure(void);


/**
 * @brief 判断相对高度零点是否有效
 *
 * @return:
 *      1 = 有效
 *      0 = 无效
 */
uint8_t PressureSensor_IsReferenceValid(void);

/**
 * @brief 获取最近一次失败的 HAL I2C 错误位掩码。
 */
uint32_t PressureSensor_GetLastI2cError(void);

/**
 * @brief 获取最近一次 I2C 失败阶段，见 PressureSensor_I2CStage_t。
 */
PressureSensor_I2CStage_t PressureSensor_GetLastI2cStage(void);


/* ============================================================
 * 滤波器
 * ============================================================
 */

/**
 * @brief 清空压力和高度滤波器
 *
 * 注意：
 *      不会清除相对高度基准压力。
 */
void PressureSensor_ResetAltitudeFilter(void);


/* ============================================================
 * 获取当前滤波后的压力
 * ============================================================
 */

/**
 * @brief 获取当前经过滤波的压力
 *
 * @return Pa
 */
float PressureSensor_GetFilteredPressure(void);


#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_SENSOR_H */
