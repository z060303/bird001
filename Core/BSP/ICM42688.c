#include "bsp_delay.h"
/**
 * @file    ICM42688.c
 * @brief   ICM-42688-P I2C2 驱动 + 姿态解算
 *
 * STM32F405
 * I2C2
 * PB10 -> SCL
 * PB11 -> SDA
 */

#include "ICM42688.h"
#include <math.h>
#include <stdio.h>


/* ============================================================
 * 私有宏
 * ============================================================ */

#define PI_F                    3.14159265358979323846f
#define RAD_TO_DEG              57.29577951308232f


/* ============================================================
 * 全局/静态变量
 * ============================================================ */

static I2C_HandleTypeDef *icm42688_hi2c = NULL;

/*
 * 当前 I2C 地址
 *
 * HAL 使用 8-bit 地址：
 *
 * 0x68 -> 0xD0
 * 0x69 -> 0xD2
 */
static uint16_t icm42688_i2c_addr =
    ICM42688_DEFAULT_ADDR;


/* 最近一次 HAL 状态 */
static HAL_StatusTypeDef icm42688_last_hal_status =
    HAL_OK;


/* 最近一次 I2C 错误 */
static uint32_t icm42688_last_i2c_error =
    HAL_I2C_ERROR_NONE;


/* 初始化阶段 */
static ICM42688_InitStage_t icm42688_last_init_stage =
    ICM42688_INIT_NONE;


/* 总线电平 */
static uint8_t icm42688_scl_level = 0U;
static uint8_t icm42688_sda_level = 0U;


/* 总线诊断统计 */
static uint32_t icm42688_scl_toggle_count = 0U;
static uint32_t icm42688_sda_toggle_count = 0U;
static uint32_t icm42688_bus_recover_count = 0U;
static uint8_t  icm42688_bus_self_test_ok = 0U;


/* 灵敏度 */

static float gyro_sensitivity =
    65.5f;

static float accel_sensitivity =
    16384.0f;


/* 姿态 */
static ICM42688_Attitude_t attitude;


/* Kalman */
static ICM42688_Kalman_t kalman_roll;
static ICM42688_Kalman_t kalman_pitch;


/* ============================================================
 * 私有函数
 * ============================================================ */

/**
 * @brief 读取当前 SCL / SDA 电平
 */
static void ICM42688_SampleBusLines(void)
{
    /*
     * 注意：
     * PB10/PB11 即使处于 AF_OD 模式，
     * 也可以通过 GPIO 输入寄存器读取实际线电平。
     */

    icm42688_scl_level =
        (HAL_GPIO_ReadPin(
            GPIOB,
            GPIO_PIN_10
        ) == GPIO_PIN_SET)
        ? 1U : 0U;


    icm42688_sda_level =
        (HAL_GPIO_ReadPin(
            GPIOB,
            GPIO_PIN_11
        ) == GPIO_PIN_SET)
        ? 1U : 0U;
}


/**
 * @brief 更新错误信息
 */
static void ICM42688_RecordError(
    HAL_StatusTypeDef status
)
{
    icm42688_last_hal_status = status;

    if (icm42688_hi2c != NULL)
    {
        icm42688_last_i2c_error =
            HAL_I2C_GetError(icm42688_hi2c);
    }

    ICM42688_SampleBusLines();
}


/**
 * @brief 用 GPIO 软件产生 9 个 SCL 脉冲，并统计 SCL/SDA 翻转次数
 *
 * 用途：
 *      1. 验证 SCL 线确实能产生高低电平翻转（排除硬件/接线故障）
 *      2. 标准 I2C 总线释放：让拉死 SDA 的从机释放总线
 *
 * 注意：
 *      仅在总线空闲时调用（初始化阶段 / 传输失败后），
 *      空闲总线上没有 START 条件，9 个脉冲不会干扰从机。
 */
static void ICM42688_PulseScl9(void)
{
    GPIO_InitTypeDef gpio = {0};
    volatile uint32_t d;
    uint32_t prev_scl;
    uint32_t prev_sda;
    uint32_t scl;
    uint32_t sda;
    uint8_t i;


    /* 清零翻转统计 */
    icm42688_scl_toggle_count = 0U;
    icm42688_sda_toggle_count = 0U;


    /*
     * 1. 关闭 I2C2 外设，
     *    避免与 GPIO 手动操作冲突
     */
    __HAL_I2C_DISABLE(icm42688_hi2c);


    /*
     * 2. SCL(PB10) 临时配置为推挽输出，先拉高
     */
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);


    /*
     * 3. SDA(PB11) 临时配置为输入（带内部上拉），
     *    用于观察总线状态
     */
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);


    prev_scl = 1U;

    prev_sda =
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
        ? 1U : 0U;


    /*
     * 4. 产生 9 个 SCL 脉冲，
     *    每个边沿都采样一次并统计翻转
     */
    for (i = 0U; i < 9U; i++)
    {
        /* SCL 低电平 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        for (d = 0U; d < 100U; d++) { ; }

        scl =
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
            ? 1U : 0U;

        if (scl != prev_scl)
        {
            icm42688_scl_toggle_count++;
            prev_scl = scl;
        }

        sda =
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
            ? 1U : 0U;

        if (sda != prev_sda)
        {
            icm42688_sda_toggle_count++;
            prev_sda = sda;
        }

        /* SCL 高电平 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        for (d = 0U; d < 100U; d++) { ; }

        scl =
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
            ? 1U : 0U;

        if (scl != prev_scl)
        {
            icm42688_scl_toggle_count++;
            prev_scl = scl;
        }

        sda =
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
            ? 1U : 0U;

        if (sda != prev_sda)
        {
            icm42688_sda_toggle_count++;
            prev_sda = sda;
        }
    }


    /*
     * 5. 恢复 SCL/SDA 为 I2C2 的 AF_OD 配置
     */
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);


    /*
     * 6. 重新使能 I2C2 外设
     */
    __HAL_I2C_ENABLE(icm42688_hi2c);


    /*
     * 7. 自检判定：
     *    9 个脉冲应至少统计到 9 次 SCL 翻转
     *    （每个脉冲 2 次边沿，正常为 18 次）
     */
    icm42688_bus_self_test_ok =
        (icm42688_scl_toggle_count >= 9U) ? 1U : 0U;
}


/**
 * @brief I2C 总线恢复
 *
 * 处理场景：
 *      从机把 SDA 拉死（总线死锁）
 *      I2C 外设 BUSY 标志卡住，后续传输不再产生 SCL 时钟
 *
 * 步骤：
 *      1. 9 个 SCL 脉冲释放总线（顺带统计 SCL/SDA 翻转）
 *      2. HAL_I2C_DeInit / Init 重置外设，清除错误状态
 *
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef ICM42688_I2cRecover(void)
{
    HAL_StatusTypeDef status;


    /* 9 个 SCL 脉冲，释放被拉死的总线 */
    ICM42688_PulseScl9();


    /* 重置 I2C2 外设（清除 BUSY 等错误标志） */
    status = HAL_I2C_DeInit(icm42688_hi2c);

    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_I2C_Init(icm42688_hi2c);

    if (status == HAL_OK)
    {
        icm42688_bus_recover_count++;
    }

    return status;
}


/**
 * @brief 更新量程对应的灵敏度
 */
static void ICM42688_UpdateSensitivity(void)
{
    /*
     * Gyro：
     *
     * FS_SEL
     * 0 -> ±2000 dps -> 16.4 LSB/dps
     * 1 -> ±1000 dps -> 32.8
     * 2 -> ±500  dps -> 65.5
     * 3 -> ±250  dps -> 131.0
     */

    switch (ICM42688_GYRO_FS_SEL)
    {
        case 0:
            gyro_sensitivity = 16.4f;
            break;

        case 1:
            gyro_sensitivity = 32.8f;
            break;

        case 2:
            gyro_sensitivity = 65.5f;
            break;

        case 3:
            gyro_sensitivity = 131.0f;
            break;

        default:
            gyro_sensitivity = 65.5f;
            break;
    }


    /*
     * Accel：
     *
     * FS_SEL
     * 0 -> ±16g -> 2048 LSB/g
     * 1 -> ±8g  -> 4096
     * 2 -> ±4g  -> 8192
     * 3 -> ±2g  -> 16384
     */

    switch (ICM42688_ACCEL_FS_SEL)
    {
        case 0:
            accel_sensitivity = 2048.0f;
            break;

        case 1:
            accel_sensitivity = 4096.0f;
            break;

        case 2:
            accel_sensitivity = 8192.0f;
            break;

        case 3:
            accel_sensitivity = 16384.0f;
            break;

        default:
            accel_sensitivity = 16384.0f;
            break;
    }
}


/**
 * @brief 检查 ICM42688 地址
 */
static HAL_StatusTypeDef ICM42688_SelectAddress(void)
{
    HAL_StatusTypeDef status;


    /*
     * 先测试 0x69
     */
    icm42688_last_init_stage =
        ICM42688_INIT_PROBE_69;

    status = HAL_I2C_IsDeviceReady(
        icm42688_hi2c,
        ICM42688_I2C_ADDR_69,
        2,
        ICM42688_I2C_TIMEOUT_MS
    );


    if (status == HAL_OK)
    {
        icm42688_i2c_addr =
            ICM42688_I2C_ADDR_69;

        icm42688_last_hal_status =
            HAL_OK;

        icm42688_last_i2c_error =
            HAL_I2C_ERROR_NONE;

        return HAL_OK;
    }


    /*
     * 0x69失败：
     * 先恢复总线（9 个 SCL 脉冲 + 重置外设），
     * 避免 BUSY 卡死导致后续不再产生 SCL 时钟，再尝试 0x68。
     */
    ICM42688_I2cRecover();


    /*
     * 0x69失败，测试0x68
     */
    icm42688_last_init_stage =
        ICM42688_INIT_PROBE_68;

    status = HAL_I2C_IsDeviceReady(
        icm42688_hi2c,
        ICM42688_I2C_ADDR_68,
        2,
        ICM42688_I2C_TIMEOUT_MS
    );


    if (status == HAL_OK)
    {
        icm42688_i2c_addr =
            ICM42688_I2C_ADDR_68;

        icm42688_last_hal_status =
            HAL_OK;

        icm42688_last_i2c_error =
            HAL_I2C_ERROR_NONE;

        return HAL_OK;
    }


    /*
     * 两个地址都失败：
     * 恢复总线，避免外设卡在错误状态
     */
    ICM42688_I2cRecover();

    ICM42688_RecordError(status);

    icm42688_last_init_stage =
        ICM42688_INIT_ERROR;

    return status;
}


/* ============================================================
 * I2C寄存器操作
 * ============================================================ */

/**
 * @brief 写一个寄存器
 */
HAL_StatusTypeDef ICM42688_WriteReg(
    uint8_t reg,
    uint8_t data
)
{
    HAL_StatusTypeDef status;


    if (icm42688_hi2c == NULL)
    {
        return HAL_ERROR;
    }


    status = HAL_I2C_Mem_Write(
        icm42688_hi2c,

        icm42688_i2c_addr,

        reg,

        I2C_MEMADD_SIZE_8BIT,

        &data,

        1,

        ICM42688_I2C_TIMEOUT_MS
    );


    if (status != HAL_OK)
    {
        /*
         * 第一次失败：记录错误，恢复总线，重试一次。
         *
         * 传输失败后 I2C 外设可能卡在 BUSY 状态，
         * 导致后续不再产生 SCL 时钟（SCL 恒高）。
         * 恢复 + 重试可以自愈，让串口继续读到数据。
         */
        ICM42688_RecordError(status);

        ICM42688_I2cRecover();

        status = HAL_I2C_Mem_Write(
            icm42688_hi2c,

            icm42688_i2c_addr,

            reg,

            I2C_MEMADD_SIZE_8BIT,

            &data,

            1,

            ICM42688_I2C_TIMEOUT_MS
        );

        if (status != HAL_OK)
        {
            ICM42688_RecordError(status);
            return status;
        }
    }


    icm42688_last_hal_status =
        HAL_OK;

    icm42688_last_i2c_error =
        HAL_I2C_ERROR_NONE;

    return HAL_OK;
}


/**
 * @brief 读一个寄存器
 */
HAL_StatusTypeDef ICM42688_ReadReg(
    uint8_t reg,
    uint8_t *data
)
{
    HAL_StatusTypeDef status;


    if ((icm42688_hi2c == NULL) ||
        (data == NULL))
    {
        return HAL_ERROR;
    }


    status = HAL_I2C_Mem_Read(
        icm42688_hi2c,

        icm42688_i2c_addr,

        reg,

        I2C_MEMADD_SIZE_8BIT,

        data,

        1,

        ICM42688_I2C_TIMEOUT_MS
    );


    if (status != HAL_OK)
    {
        /*
         * 第一次失败：记录错误，恢复总线，重试一次。
         */
        ICM42688_RecordError(status);

        ICM42688_I2cRecover();

        status = HAL_I2C_Mem_Read(
            icm42688_hi2c,

            icm42688_i2c_addr,

            reg,

            I2C_MEMADD_SIZE_8BIT,

            data,

            1,

            ICM42688_I2C_TIMEOUT_MS
        );

        if (status != HAL_OK)
        {
            ICM42688_RecordError(status);
            return status;
        }
    }


    icm42688_last_hal_status =
        HAL_OK;

    icm42688_last_i2c_error =
        HAL_I2C_ERROR_NONE;

    return HAL_OK;
}


/**
 * @brief 连续读取多个寄存器
 */
HAL_StatusTypeDef ICM42688_ReadRegs(
    uint8_t reg,
    uint8_t *data,
    uint16_t len
)
{
    HAL_StatusTypeDef status;


    if ((icm42688_hi2c == NULL) ||
        (data == NULL) ||
        (len == 0U))
    {
        return HAL_ERROR;
    }


    status = HAL_I2C_Mem_Read(
        icm42688_hi2c,

        icm42688_i2c_addr,

        reg,

        I2C_MEMADD_SIZE_8BIT,

        data,

        len,

        ICM42688_I2C_TIMEOUT_MS
    );


    if (status != HAL_OK)
    {
        /* Preserve this failure. Recover once and retry on the next 8 ms release. */
        ICM42688_RecordError(status);
        (void)ICM42688_I2cRecover();
        return status;
    }


    icm42688_last_hal_status =
        HAL_OK;

    icm42688_last_i2c_error =
        HAL_I2C_ERROR_NONE;

    return HAL_OK;
}


/* ============================================================
 * 初始化
 * ============================================================ */

/**
 * @brief 初始化 ICM42688
 */
HAL_StatusTypeDef ICM42688_Init(
    I2C_HandleTypeDef *hi2c
)
{
    uint8_t who_am_i;

    uint8_t gyro_config;

    uint8_t accel_config;

    HAL_StatusTypeDef status;


    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }


    /*
     * 保存 I2C 句柄
     */
    icm42688_hi2c = hi2c;


    /*
     * 必须使用 I2C2
     */
    if (hi2c->Instance != I2C2)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * 读取总线电平
     */
    ICM42688_SampleBusLines();


    /*
     * 等待 I2C2 初始化完成
     */
    BSP_DelayMs(10);


    icm42688_last_init_stage =
        ICM42688_INIT_I2C_READY;


    /*
     * ===========================
     * 总线自检：
     * 用 GPIO 软件产生 9 个 SCL 脉冲，
     * 统计翻转次数并打印，证明 SCL 线能正常高低翻转。
     * ===========================
     */

    /*
     * ===========================
     * 第一步：检测 ICM42688 地址
     * ===========================
     */

    status = ICM42688_SelectAddress();

    if (status != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return status;
    }


    /*
     * ===========================
     * 第二步：选择 Bank 0
     * ===========================
     */

    if (ICM42688_WriteReg(
            ICM42688_REG_REG_BANK_SEL,
            0x00U
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * ===========================
     * 第三步：读取 WHO_AM_I
     * ===========================
     */

    icm42688_last_init_stage =
        ICM42688_INIT_WHO_AM_I;


    who_am_i =
        ICM42688_ReadWhoAmI();


    if (who_am_i != ICM42688_WHO_AM_I_VALUE)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * ===========================
     * 第四步：软复位
     * ===========================
     */

    icm42688_last_init_stage =
        ICM42688_INIT_RESET;


    if (ICM42688_WriteReg(
            ICM42688_REG_DEVICE_CONFIG,
            0x01U
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * 芯片复位需要一点时间
     */
    BSP_DelayMs(10);


    /*
     * 复位后重新确认地址
     */
    status = ICM42688_SelectAddress();

    if (status != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return status;
    }


    /*
     * 重新选择 Bank 0
     */
    if (ICM42688_WriteReg(
            ICM42688_REG_REG_BANK_SEL,
            0x00U
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * ===========================
     * 更新灵敏度
     * ===========================
     */

    ICM42688_UpdateSensitivity();


    /*
     * ===========================
     * Gyro 配置
     * ===========================
     *
     * FS_SEL << 5
     * ODR
     */

    gyro_config =
        (uint8_t)(
            (ICM42688_GYRO_FS_SEL << 5U) |
            (ICM42688_GYRO_ODR & 0x0FU)
        );


    icm42688_last_init_stage =
        ICM42688_INIT_GYRO_CONFIG;


    if (ICM42688_WriteReg(
            ICM42688_REG_GYRO_CONFIG0,
            gyro_config
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * ===========================
     * Accel 配置
     * ===========================
     */

    accel_config =
        (uint8_t)(
            (ICM42688_ACCEL_FS_SEL << 5U) |
            (ICM42688_ACCEL_ODR & 0x0FU)
        );


    icm42688_last_init_stage =
        ICM42688_INIT_ACCEL_CONFIG;


    if (ICM42688_WriteReg(
            ICM42688_REG_ACCEL_CONFIG0,
            accel_config
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * ===========================
     * 开启 Gyro + Accel
     * Low Noise Mode
     * ===========================
     */

    icm42688_last_init_stage =
        ICM42688_INIT_POWER_MODE;


    if (ICM42688_WriteReg(
            ICM42688_REG_PWR_MGMT0,
            ICM42688_PWR_MGMT0_LN_MODE
        ) != HAL_OK)
    {
        icm42688_last_init_stage =
            ICM42688_INIT_ERROR;

        return HAL_ERROR;
    }


    /*
     * 等待传感器稳定
     */
    BSP_DelayMs(50);


    /*
     * 初始化姿态解算
     */
    ICM42688_Attitude_Init();


    /*
     * 初始化完成
     */
    icm42688_last_init_stage =
        ICM42688_INIT_READY;


    return HAL_OK;
}


/* ============================================================
 * WHO_AM_I
 * ============================================================ */

uint8_t ICM42688_ReadWhoAmI(void)
{
    uint8_t value = 0U;


    if (ICM42688_ReadReg(
            ICM42688_REG_WHO_AM_I,
            &value
        ) != HAL_OK)
    {
        return 0U;
    }


    return value;
}


/* ============================================================
 * 加速度读取
 * ============================================================ */

HAL_StatusTypeDef ICM42688_GetAccel(
    ICM42688_Accel_t *accel
)
{
    uint8_t data[6];


    if (accel == NULL)
    {
        return HAL_ERROR;
    }


    if (ICM42688_ReadRegs(
            ICM42688_REG_ACCEL_DATA_X1,
            data,
            6
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }


    /*
     * X
     */
    accel->raw_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            data[1]
        );


    /*
     * Y
     */
    accel->raw_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            data[3]
        );


    /*
     * Z
     */
    accel->raw_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            data[5]
        );


    /*
     * 转换为 g
     */
    accel->x =
        (float)accel->raw_x /
        accel_sensitivity;


    accel->y =
        (float)accel->raw_y /
        accel_sensitivity;


    accel->z =
        (float)accel->raw_z /
        accel_sensitivity;


    return HAL_OK;
}


/* ============================================================
 * 陀螺仪读取
 * ============================================================ */

HAL_StatusTypeDef ICM42688_GetGyro(
    ICM42688_Gyro_t *gyro
)
{
    uint8_t data[6];


    if (gyro == NULL)
    {
        return HAL_ERROR;
    }


    if (ICM42688_ReadRegs(
            ICM42688_REG_GYRO_DATA_X1,
            data,
            6
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }


    /*
     * X
     */
    gyro->raw_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            data[1]
        );


    /*
     * Y
     */
    gyro->raw_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            data[3]
        );


    /*
     * Z
     */
    gyro->raw_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            data[5]
        );


    /*
     * 转换为 dps
     */
    gyro->x =
        (float)gyro->raw_x /
        gyro_sensitivity;


    gyro->y =
        (float)gyro->raw_y /
        gyro_sensitivity;


    gyro->z =
        (float)gyro->raw_z /
        gyro_sensitivity;


    return HAL_OK;
}


/* ============================================================
 * 同时读取 Accel + Gyro
 * ============================================================ */

HAL_StatusTypeDef ICM42688_GetMotion(
    ICM42688_Accel_t *accel,
    ICM42688_Gyro_t *gyro
)
{
    uint8_t data[12];


    if ((accel == NULL) ||
        (gyro == NULL))
    {
        return HAL_ERROR;
    }


    /*
     * 一次读取：
     *
     * 0x1F ~ 0x2A
     *
     * 共 12 字节
     */

    if (ICM42688_ReadRegs(
            ICM42688_REG_ACCEL_DATA_X1,
            data,
            12
        ) != HAL_OK)
    {
        return HAL_ERROR;
    }


    /*
     * Accel X
     */
    accel->raw_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            data[1]
        );


    /*
     * Accel Y
     */
    accel->raw_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            data[3]
        );


    /*
     * Accel Z
     */
    accel->raw_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            data[5]
        );


    /*
     * Gyro X
     */
    gyro->raw_x =
        (int16_t)(
            ((uint16_t)data[6] << 8U) |
            data[7]
        );


    /*
     * Gyro Y
     */
    gyro->raw_y =
        (int16_t)(
            ((uint16_t)data[8] << 8U) |
            data[9]
        );


    /*
     * Gyro Z
     */
    gyro->raw_z =
        (int16_t)(
            ((uint16_t)data[10] << 8U) |
            data[11]
        );


    /*
     * 转换
     */

    accel->x =
        (float)accel->raw_x /
        accel_sensitivity;

    accel->y =
        (float)accel->raw_y /
        accel_sensitivity;

    accel->z =
        (float)accel->raw_z /
        accel_sensitivity;


    gyro->x =
        (float)gyro->raw_x /
        gyro_sensitivity;

    gyro->y =
        (float)gyro->raw_y /
        gyro_sensitivity;

    gyro->z =
        (float)gyro->raw_z /
        gyro_sensitivity;


    return HAL_OK;
}


/* ============================================================
 * Kalman
 * ============================================================ */

static void Kalman_Init(
    ICM42688_Kalman_t *k
)
{
    if (k == NULL)
    {
        return;
    }


    k->angle = 0.0f;

    k->bias = 0.0f;


    k->P[0][0] = 0.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 0.0f;


    k->Q_angle =
        0.001f;

    k->Q_bias =
        0.003f;

    k->R_measure =
        0.03f;
}


/**
 * @brief Kalman更新
 */
static float Kalman_Update(
    ICM42688_Kalman_t *k,
    float measured_angle,
    float gyro_rate,
    float dt
)
{
    float rate;

    float S;

    float K0;

    float K1;

    float y;

    float P00_temp;

    float P01_temp;


    if (k == NULL)
    {
        return 0.0f;
    }


    /*
     * 去除陀螺仪 bias
     */
    rate =
        gyro_rate -
        k->bias;


    /*
     * 预测
     */
    k->angle +=
        dt * rate;


    /*
     * 协方差预测
     */
    k->P[0][0] +=
        dt *
        (
            dt * k->P[1][1]
            - k->P[0][1]
            - k->P[1][0]
            + k->Q_angle
        );


    k->P[0][1] -=
        dt * k->P[1][1];


    k->P[1][0] -=
        dt * k->P[1][1];


    k->P[1][1] +=
        k->Q_bias * dt;


    /*
     * 创新
     */
    y =
        measured_angle -
        k->angle;


    /*
     * 创新协方差
     */
    S =
        k->P[0][0] +
        k->R_measure;


    if (S <= 0.0f)
    {
        return k->angle;
    }


    /*
     * Kalman增益
     */
    K0 =
        k->P[0][0] / S;


    K1 =
        k->P[1][0] / S;


    /*
     * 更新
     */
    k->angle +=
        K0 * y;


    k->bias +=
        K1 * y;


    /*
     * 更新协方差
     */
    P00_temp =
        k->P[0][0];

    P01_temp =
        k->P[0][1];


    k->P[0][0] -=
        K0 * P00_temp;

    k->P[0][1] -=
        K0 * P01_temp;

    k->P[1][0] -=
        K1 * P00_temp;

    k->P[1][1] -=
        K1 * P01_temp;


    return k->angle;
}


/* ============================================================
 * 姿态初始化
 * ============================================================ */

void ICM42688_Attitude_Init(void)
{
    Kalman_Init(
        &kalman_roll
    );

    Kalman_Init(
        &kalman_pitch
    );


    attitude.roll =
        0.0f;

    attitude.pitch =
        0.0f;

    attitude.yaw =
        0.0f;
}


/* ============================================================
 * 姿态更新
 * ============================================================ */

void ICM42688_Attitude_Update(
    const ICM42688_Accel_t *accel,
    const ICM42688_Gyro_t *gyro,
    float dt
)
{
    float accel_roll;

    float accel_pitch;


    if ((accel == NULL) ||
        (gyro == NULL))
    {
        return;
    }


    /*
     * 主循环含 100 ms 阻塞延时，实测周期约 0.1~0.2 s，
     * 上限放宽到 0.5 s，避免每次姿态更新都直接返回。
     */
    if ((dt <= 0.0f) ||
        (dt > 0.5f))
    {
        return;
    }


    /*
     * 当前默认安装方向：
     *
     * X → 前
     * Y → 右
     * Z → 上
     */


    /*
     * Roll
     */
    accel_roll =
        atan2f(
            accel->y,
            accel->z
        ) * RAD_TO_DEG;


    /*
     * Pitch
     */
    accel_pitch =
        atan2f(
            -accel->x,

            sqrtf(
                accel->y * accel->y +
                accel->z * accel->z
            )
        ) * RAD_TO_DEG;


    /*
     * Kalman
     */

    attitude.roll =
        Kalman_Update(
            &kalman_roll,
            accel_roll,
            gyro->x,
            dt
        );


    attitude.pitch =
        Kalman_Update(
            &kalman_pitch,
            accel_pitch,
            gyro->y,
            dt
        );


    /*
     * 没有磁力计：
     *
     * Yaw只能通过陀螺仪积分
     *
     * 会随着时间产生漂移。
     */

    attitude.yaw +=
        gyro->z * dt;


    /*
     * 限制到 -180 ~ +180
     */

    while (attitude.yaw > 180.0f)
    {
        attitude.yaw -=
            360.0f;
    }


    while (attitude.yaw < -180.0f)
    {
        attitude.yaw +=
            360.0f;
    }
}


/* ============================================================
 * 获取姿态
 * ============================================================ */

void ICM42688_GetAttitude(
    ICM42688_Attitude_t *result
)
{
    if (result == NULL)
    {
        return;
    }


    *result =
        attitude;
}


/**
 * @brief 获取 Roll / Pitch / Yaw
 */
void ICM42688_GetRollPitchYaw(
    float *roll,
    float *pitch,
    float *yaw
)
{
    if (roll != NULL)
    {
        *roll =
            attitude.roll;
    }


    if (pitch != NULL)
    {
        *pitch =
            attitude.pitch;
    }


    if (yaw != NULL)
    {
        *yaw =
            attitude.yaw;
    }
}


/* ============================================================
 * 一体化更新
 * ============================================================ */

HAL_StatusTypeDef ICM42688_Update(
    float dt
)
{
    ICM42688_Accel_t accel;

    ICM42688_Gyro_t gyro;

    static uint32_t last_err_print_ms = 0U;

    uint32_t now_ms;


    now_ms = HAL_GetTick();


    if (ICM42688_GetMotion(
            &accel,
            &gyro
        ) != HAL_OK)
    {
        /*
         * 失败诊断（节流：每 1 秒打印一次），
         * 用于定位 I2C 超时/无应答的物理层原因。
         */
        if ((now_ms - last_err_print_ms) >= 1000U)
        {
            last_err_print_ms = now_ms;

            printf(
                "ICM diag: ERR=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX "
                "SCL=%u SDA=%u ADDR=0x%02X RECOVER=%lu\r\n",
                (unsigned long)icm42688_last_i2c_error,
                (unsigned long)ICM42688_GetI2cSr1(),
                (unsigned long)ICM42688_GetI2cSr2(),
                (unsigned long)ICM42688_GetI2cCr1(),
                (unsigned int)icm42688_scl_level,
                (unsigned int)icm42688_sda_level,
                (unsigned int)ICM42688_GetI2cAddress(),
                (unsigned long)icm42688_bus_recover_count
            );
        }

        return HAL_ERROR;
    }


    ICM42688_Attitude_Update(
        &accel,
        &gyro,
        dt
    );


    return HAL_OK;
}


/* ============================================================
 * 调试信息
 * ============================================================ */

uint8_t ICM42688_GetI2cAddress(void)
{
    /*
     * 返回7-bit地址
     *
     * 0xD2 -> 0x69
     * 0xD0 -> 0x68
     */

    return (uint8_t)(
        icm42688_i2c_addr >> 1U
    );
}


uint32_t ICM42688_GetLastI2cError(void)
{
    return icm42688_last_i2c_error;
}


HAL_StatusTypeDef ICM42688_GetLastHalStatus(void)
{
    return icm42688_last_hal_status;
}


ICM42688_InitStage_t
ICM42688_GetLastInitStage(void)
{
    return icm42688_last_init_stage;
}


const char *
ICM42688_GetLastInitStageName(void)
{
    switch (icm42688_last_init_stage)
    {
        case ICM42688_INIT_NONE:
            return "NONE";

        case ICM42688_INIT_I2C_READY:
            return "I2C_READY";

        case ICM42688_INIT_PROBE_69:
            return "PROBE_0x69";

        case ICM42688_INIT_PROBE_68:
            return "PROBE_0x68";

        case ICM42688_INIT_WHO_AM_I:
            return "WHO_AM_I";

        case ICM42688_INIT_RESET:
            return "RESET";

        case ICM42688_INIT_GYRO_CONFIG:
            return "GYRO_CONFIG";

        case ICM42688_INIT_ACCEL_CONFIG:
            return "ACCEL_CONFIG";

        case ICM42688_INIT_POWER_MODE:
            return "POWER_MODE";

        case ICM42688_INIT_READY:
            return "READY";

        case ICM42688_INIT_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * SCL / SDA
 * ============================================================ */

uint8_t ICM42688_GetSclLevel(void)
{
    ICM42688_SampleBusLines();

    return icm42688_scl_level;
}


uint8_t ICM42688_GetSdaLevel(void)
{
    ICM42688_SampleBusLines();

    return icm42688_sda_level;
}


/* ============================================================
 * I2C寄存器调试
 * ============================================================ */

uint32_t ICM42688_GetI2cSr1(void)
{
    if (icm42688_hi2c == NULL)
    {
        return 0U;
    }


    return icm42688_hi2c->Instance->SR1;
}


uint32_t ICM42688_GetI2cSr2(void)
{
    if (icm42688_hi2c == NULL)
    {
        return 0U;
    }


    return icm42688_hi2c->Instance->SR2;
}


uint32_t ICM42688_GetI2cCr1(void)
{
    if (icm42688_hi2c == NULL)
    {
        return 0U;
    }


    return icm42688_hi2c->Instance->CR1;
}


/* ============================================================
 * 总线诊断
 * ============================================================ */

uint32_t ICM42688_GetSclToggleCount(void)
{
    return icm42688_scl_toggle_count;
}


uint32_t ICM42688_GetSdaToggleCount(void)
{
    return icm42688_sda_toggle_count;
}


uint32_t ICM42688_GetBusRecoverCount(void)
{
    return icm42688_bus_recover_count;
}


uint8_t ICM42688_GetBusSelfTestResult(void)
{
    return icm42688_bus_self_test_ok;
}
