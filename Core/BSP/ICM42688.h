#ifndef __ICM42688_H
#define __ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/*
 * ============================================================
 * ICM-42688-P
 * STM32F405 + I2C2
 *
 * I2C2:
 *      PB10 -> SCL
 *      PB11 -> SDA
 *
 * ICM42688 I2C地址：
 *      AD0 = 0 -> 0x68
 *      AD0 = 1 -> 0x69
 *
 * STM32 HAL 使用 8-bit 地址：
 *      0x68 << 1 = 0xD0
 *      0x69 << 1 = 0xD2
 * ============================================================
 */

/* ==================== I2C地址 ==================== */

#define ICM42688_I2C_ADDR_68      (0x68U << 1)
#define ICM42688_I2C_ADDR_69      (0x69U << 1)

#define ICM42688_I2C_TIMEOUT_MS   2U

/*
 * 实测板载 ICM-42688-P 的 I2C 地址为 0x68（AD0/AP_AD0 接地）。
 * 若硬件将 AD0/AP_AD0 接高，则改为 0x69。
 */
#define ICM42688_DEFAULT_ADDR     ICM42688_I2C_ADDR_68


/* ==================== ICM42688寄存器 ==================== */

#define ICM42688_REG_DEVICE_CONFIG       0x11U

#define ICM42688_REG_ACCEL_DATA_X1       0x1FU
#define ICM42688_REG_ACCEL_DATA_X0       0x20U
#define ICM42688_REG_ACCEL_DATA_Y1       0x21U
#define ICM42688_REG_ACCEL_DATA_Y0       0x22U
#define ICM42688_REG_ACCEL_DATA_Z1       0x23U
#define ICM42688_REG_ACCEL_DATA_Z0       0x24U

#define ICM42688_REG_GYRO_DATA_X1        0x25U
#define ICM42688_REG_GYRO_DATA_X0        0x26U
#define ICM42688_REG_GYRO_DATA_Y1        0x27U
#define ICM42688_REG_GYRO_DATA_Y0        0x28U
#define ICM42688_REG_GYRO_DATA_Z1        0x29U
#define ICM42688_REG_GYRO_DATA_Z0        0x2AU

#define ICM42688_REG_PWR_MGMT0           0x4EU
#define ICM42688_REG_GYRO_CONFIG0        0x4FU
#define ICM42688_REG_ACCEL_CONFIG0       0x50U

#define ICM42688_REG_WHO_AM_I            0x75U
#define ICM42688_REG_REG_BANK_SEL        0x76U


/* ==================== 芯片参数 ==================== */

#define ICM42688_WHO_AM_I_VALUE          0x47U

/*
 * Gyro:
 *      FS_SEL = 2
 *      ±500 dps
 *
 * ODR:
 *      0x06 = 1 kHz
 */
#define ICM42688_GYRO_FS_SEL             2U
#define ICM42688_GYRO_ODR                0x06U

/*
 * Accel:
 *      FS_SEL = 3
 *      ±2 g
 *
 * ODR:
 *      0x06 = 1 kHz
 */
#define ICM42688_ACCEL_FS_SEL            3U
#define ICM42688_ACCEL_ODR               0x06U

/*
 * Gyro + Accel Low Noise Mode
 */
#define ICM42688_PWR_MGMT0_LN_MODE       0x0FU


/* ==================== 数据结构 ==================== */

typedef struct
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    float x;
    float y;
    float z;

} ICM42688_Gyro_t;


typedef struct
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    float x;
    float y;
    float z;

} ICM42688_Accel_t;


typedef struct
{
    float roll;
    float pitch;
    float yaw;

} ICM42688_Attitude_t;


/* ==================== Kalman ==================== */

typedef struct
{
    float angle;
    float bias;

    float P[2][2];

    float Q_angle;
    float Q_bias;
    float R_measure;

} ICM42688_Kalman_t;


/* ==================== 初始化状态 ==================== */

typedef enum
{
    ICM42688_INIT_NONE = 0,

    ICM42688_INIT_I2C_READY,

    ICM42688_INIT_PROBE_69,

    ICM42688_INIT_PROBE_68,

    ICM42688_INIT_WHO_AM_I,

    ICM42688_INIT_RESET,

    ICM42688_INIT_GYRO_CONFIG,

    ICM42688_INIT_ACCEL_CONFIG,

    ICM42688_INIT_POWER_MODE,

    ICM42688_INIT_READY,

    ICM42688_INIT_ERROR

} ICM42688_InitStage_t;


/* ==================== API ==================== */

/*
 * 初始化 ICM42688
 *
 * 参数：
 *      hi2c = &hi2c2
 */
HAL_StatusTypeDef ICM42688_Init(I2C_HandleTypeDef *hi2c);


/* ==================== 寄存器操作 ==================== */

HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg,
                                    uint8_t data);

HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg,
                                   uint8_t *data);

HAL_StatusTypeDef ICM42688_ReadRegs(uint8_t reg,
                                    uint8_t *data,
                                    uint16_t len);


/* ==================== 基础信息 ==================== */

uint8_t ICM42688_ReadWhoAmI(void);

uint8_t ICM42688_GetI2cAddress(void);

uint32_t ICM42688_GetLastI2cError(void);

HAL_StatusTypeDef ICM42688_GetLastHalStatus(void);

ICM42688_InitStage_t ICM42688_GetLastInitStage(void);

const char *ICM42688_GetLastInitStageName(void);


/* ==================== 传感器数据 ==================== */

HAL_StatusTypeDef ICM42688_GetAccel(
    ICM42688_Accel_t *accel
);

HAL_StatusTypeDef ICM42688_GetGyro(
    ICM42688_Gyro_t *gyro
);

HAL_StatusTypeDef ICM42688_GetMotion(
    ICM42688_Accel_t *accel,
    ICM42688_Gyro_t *gyro
);


/* ==================== 姿态解算 ==================== */

void ICM42688_Attitude_Init(void);

void ICM42688_Attitude_Update(
    const ICM42688_Accel_t *accel,
    const ICM42688_Gyro_t *gyro,
    float dt
);

void ICM42688_GetAttitude(
    ICM42688_Attitude_t *attitude
);

void ICM42688_GetRollPitchYaw(
    float *roll,
    float *pitch,
    float *yaw
);


/* ==================== 一体化更新 ==================== */

HAL_StatusTypeDef ICM42688_Update(float dt);


/* ==================== 调试接口 ==================== */

uint8_t ICM42688_GetSclLevel(void);

uint8_t ICM42688_GetSdaLevel(void);

uint32_t ICM42688_GetI2cSr1(void);

uint32_t ICM42688_GetI2cSr2(void);

uint32_t ICM42688_GetI2cCr1(void);


/* ==================== 总线诊断接口 ==================== */

/*
 * 获取自检/总线恢复期间统计到的 SCL、SDA 翻转次数。
 *
 * 说明：
 *      开机初始化时会用 GPIO 软件产生 9 个 SCL 脉冲自检，
 *      正常时 SCL 翻转次数 >= 9（每个脉冲 2 次边沿）。
 *
 *      如果串口打印的 SCL 一直是 1（空闲高电平），
 *      请以这两个计数作为 SCL 线是否真实翻转的依据。
 */
uint32_t ICM42688_GetSclToggleCount(void);

uint32_t ICM42688_GetSdaToggleCount(void);

/*
 * 获取 I2C 总线自动恢复次数。
 *
 * 每次 I2C 传输失败后会执行：
 *      9 个 SCL 脉冲释放总线 + 重新初始化 I2C2 外设，
 *      避免外设 BUSY 卡死导致 SCL 不再产生时钟。
 */
uint32_t ICM42688_GetBusRecoverCount(void);

/*
 * 获取 SCL 线自检结果：
 *      1 = SCL 能正常高低翻转
 *      0 = 自检未通过（SCL 硬件/接线问题）
 */
uint8_t ICM42688_GetBusSelfTestResult(void);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H */
