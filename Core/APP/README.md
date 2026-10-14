# FreeRTOS 传感器与横滚舵机应用

入口为 `main.c -> App_Start()`。主循环由 FreeRTOS 调度器接管。
内核使用官方 FreeRTOS-Kernel V11.1.0、GCC ARM_CM4F 端口，保存任务浮点上下文。
系统 tick 为 1 ms。任务、队列及空闲任务均静态分配，不使用 FreeRTOS heap。

| 任务 | 周期 | 优先级（数值越大越高） | 栈大小 | 职责 |
| --- | --- | --- | --- | --- |
| roll_servo | 8 ms | 4 | 1024 B | 获取最新横滚角、检查有效期、更新 TIM4_CH3 |
| imu_8ms | 8 ms | 3 | 2048 B | I2C2 读取加速度/角速度、姿态解算、发布横滚角与打印记录 |
| baro_20ms | 20 ms | 2 | 2048 B | I2C1 读取 MS5611 D1/D2、温度补偿、发布打印记录 |
| uart_log | 队列触发 | 1 | 4096 B | 格式化记录，通过 USART6 中断发送 |

周期指传感器初始化成功后的任务释放周期。使用 `xTaskDelayUntil()`，避免工作耗时累积造成周期漂移。
IMU 正常启动后对齐 8 ms tick 网格，控制任务相位偏移约 3 ms，通常读取本周期完成的样本。
发生超时时记录 overrun，跳过过期的释放点，不进行连续补跑；故障情况下不保证采集周期。
气压计与 IMU 分别独占 I2C1 和 I2C2，控制任务独占舵机，日志任务独占串口发送。

## 舵机映射

- PB8 / TIM4_CH3，原有 PWM 保持 50 Hz（20 ms 一帧），1 tick = 1 μs。
- 舵机范围 0～145°，脉宽 500～2500 μs。启动任务前先设置 72.5° / 1500 μs，并装载比较寄存器再启动 PWM。
- 默认目标角度：`clamp(72.5 - roll, 0, 145)`，比例为 1° 横滚角对应 1° 舵机偏转。
- 沿用 BSP 原有约定：较小角度/脉宽为左，较大为右。正横滚向左，负横滚向右，0° 回中。
- 例如 +30° → 42.5° / 1086 μs；-30° → 102.5° / 1914 μs；超过 ±72.5° 后限幅。
- 改比例和机械方向：编辑 `roll_control.h` 的 `APP_ROLL_GAIN` 和 `APP_ROLL_DIRECTION`。
- 任务每 8 ms 更新目标，PWM 比较值在硬件的下一个 20 ms 帧边界生效；125 Hz 控制不等于 125 Hz PWM。
- 无样本、最近一次读取失败、角度非有限值或样本超过 40 ms 时回中。

## 气压计周期接口

新增 `PressureSensor_ReadPeriodic()`：D1/D2 都采用 OSR2048，命令 0x46 / 0x56；每次转换等待至少 5 ms。
在 1 kHz FreeRTOS tick 下，等待增加一个 tick 的相位余量并主动让出 CPU，两次等待合计约 10～12 ms，
余下时间用于 I2C 通信、补偿计算和抢占。采样值仍使用现有 PROM 系数和二阶温度补偿。
原 `PressureSensor_Read()` 保留 OSR4096（0x48 / 0x58）接口供其他用途，不能用于保证本应用的 20 ms 周期。
OSR2048 比 OSR4096 的采样噪声略大；本应用优先满足 20 ms 采样预算。

## 串口与并发

USART6：PC6 TX / PC7 RX，115200、8N1。启动时打印字段说明：

```text
I,t_ms,roll_deg,pitch_deg,yaw_deg
P,t_ms,Pa,C,alt_m
```

示例（仅说明格式）：

```text
I,1234,12.34,-1.25,0.03
P,1240,101325.00,25.30,0.00
I,1242,ERR=1
```

`t_ms` 为解析完成时的 RTOS tick 毫秒值；高度为标准气压下的绝对高度，不是开机相对高度。
错误码为各驱动状态枚举。正确记录每次采样都会入队，发送有少量队列延迟。
格式化只在日志任务进行，使用整数实现两位小数输出；数据行长度有上限，175 行/秒可容纳在 115200 波特率内。

- 横滚角使用长度 1 的覆盖队列，控制任务只取最新记录，不等待 IMU，不积压旧角度。
- 打印队列长度 32，生产者零等待。串口异常或额外日志过多时，队列满丢弃新日志并计数，不阻塞控制。
- UART 使用 `HAL_UART_Transmit_IT()`；完成/错误中断通知日志任务，等待最多 20 ms，超时同步终止发送。
- USART6 中断优先级为 5，满足 `configMAX_SYSCALL_INTERRUPT_PRIORITY`。ISR 不打印、不进行阻塞等待。
- 没有互斥锁或嵌套锁，没有任务间无限等待；等待硬件转换时调用 `BSP_DelayMs()` 让出 CPU。
- ICM 多寄存器读取使用 2 ms HAL 超时，失败恢复总线后在下一周期重试，不立即重复长读取。
- MS5611 传输使用 3 ms HAL 超时，保留最多 3 次重试/总线恢复；恢复或设备启动可能超过正常周期。
- STM32 HAL 对 BUSY 标志存在内部 25 ms 等待，所以不能把 2/3 ms 参数当成故障时的整个函数执行上限。
  此时最高优先级控制任务仍可抢占传感器任务，并按样本有效期回中。
- 初始化失败的传感器每秒尝试重新初始化，另一传感器及控制任务继续运行。
- legacy `printf` 的字符通过 `App_LogWrite()` 尽力入队；不要在实时任务或 ISR 中频繁调用 printf，
  周期采样使用已有的结构化记录队列。

## 调试与维护

在调试器查看 `g_app_diagnostics`：样本数、读取错误、周期超时、控制更新数、回中状态、日志丢弃和串口错误。
可用 `uxTaskGetStackHighWaterMark()` 检查任务栈余量。开启栈溢出检查和 FreeRTOS 断言；
异常时保存 `g_app_assert_file` / `g_app_assert_line`，设置中点脉宽后停机，不在异常路径调用串口。

主要文件：

- `app_freertos.c/.h`：任务、队列、串口发送及诊断。
- `roll_control.c/.h`：控制映射、方向、比例和样本超时。
- `rtos_hooks.c`：静态空闲任务与异常钩子。
- `Core/Inc/FreeRTOSConfig.h`：内核配置。
- `Core/BSP/bsp_delay.c/.h`：兼容裸机/FreeRTOS 的最短等待。
- `Core/Src/stm32f4xx_it.c`：SysTick 同时维护 HAL 和 FreeRTOS；SVC/PendSV 由内核端口提供。

FreeRTOS 通过顶层 CMake 集成，而不是由 CubeMX 生成。
重新生成 CubeMX 工程后需检查 SVC/PendSV 不能恢复成空实现，SysTick 必须保留 FreeRTOS 分发；
`main.c` 的 App_Start 位于 USER CODE 区，USART6 的中断优先级也已同步进 `.ioc`。
本公开仓库未包含开发过程中的本地备份目录。

## 验证方法

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
python tests/run_app_arm_test.py
```

发布副本的构建与测试结果见 [构建与验证](../../docs/构建与验证.md)。ARM 测试需要 Python 的 `unicorn` 和 `pyelftools`，测试真实 APP/BSP 函数，
只替换 HAL/RTOS 边界：覆盖中点启动、方向/限幅、无效和过期数据回中、周期回绕/超时、
队列满、UART 错误/超时、MS5611 数据手册补偿算例/转换命令、IMU 有符号解析/姿态和读取恢复。
测试不模拟真实任务上下文切换、电气时序或舵机机械方向。本次未烧录，上板后需核对日志间隔、overrun 计数和实际左右方向。

设计依据：[FreeRTOS 周期任务 API](https://www.freertos.org/Documentation/02-Kernel/04-API-references/02-Task-control/03-xTaskDelayUntil)、
[TE MS5611 数据手册](https://my.avnet.com/wcm/connect/83d782cc-0b60-4045-a710-a8a2daf90b20/TE-Connectivity-MS5611-Series-EN-Datasheet.pdf?CACHE=NONE&CVID=oMx6Zo2&ContentCache=NONE&MOD=AJPERES)。


## 2026-10-14 修改记录

以下记录本次 SBUS 接收、串口打印和 CH1/CH3 遥控控制改动；上述既有说明保留为历史内容，当前行为以本节为准。

### 当前任务与启动配置

- `APP_SBUS_ONLY=0`、`APP_MOTOR_TASK_ENABLED=1`：启动 `ch1_servo`（8 ms）、`imu_8ms`（8 ms）、`baro_20ms`（20 ms）、`sbus_5ms`（5 ms）、`bldc_20ms`（20 ms）和 `uart_log` 六个任务。
- 电机初始化在调度器启动前完成，占空比设为 0，等待 CH3 指令，不再上电自动运行。
- `APP_MOTOR_TASK_ENABLED=0` 可单独暂停电机任务；`APP_SBUS_ONLY=1` 可只运行 SBUS 接收和串口打印。
- CH1/CH3 及原始帧时间戳共用长度 1 的覆盖队列，舵机、电机任务使用 `xQueuePeek()` 各自读取，不消耗对方的样本。IMU 姿态解算继续运行，舵机角度由 CH1 决定。

### SBUS 接收与串口打印

- 接收：PA1 / UART4_RX / AF8；信号已硬件反相，固定 100000 波特率、8 位数据、偶校验、2 停止位。STM32 HAL 的 `UART_WORDLENGTH_9B + UART_PARITY_EVEN` 对应线上 8E2。
- 丢弃 PE/FE/NE/ORE 错误帧，等待帧间空闲重新同步；IDLE 间隔清除残帧，防止与下一帧拼接。启动时先等待帧间空闲。
- 帧尾明确接受 0x00/0x04/0x08/0x14/0x24/0x34，支持常见 SBUS/SBUS2 帧尾；断流时保持标准 UART4 配置。
- 打印：PC6 TX / PC7 RX 的 USART6，115200、8N1。每隔至少 50 ms 输出一次最新通道和状态；无信号或超过 100 ms 没有新帧时输出 `sbus:NO_SIGNAL rx=... err=...`，持续无信号每秒输出一次。
- 示例：

```text
sbus:CH1=942 CH2=992 CH3=312 CH4=992 CH5=992 CH6=992 CH7=992 CH8=992 CH9=992 CH10=992 CH11=992 CH12=992 CH13=992 CH14=992 CH15=992 CH16=992 D17=0 D18=0 lost=0 fs=0 n=123
```

`CH1`～`CH16` 为原始 11 位通道值；`D17/D18` 为数字通道，`lost/fs` 为丢帧/失控标志，`n` 为解析帧数。原有 `baro:` 输出保留，IMU 不入串口日志队列。打印记录复制入队，由日志任务统一格式化和中断发送，发送等待上限从 20 ms 增至 40 ms。

接线：PC6 接 USB 转串口模块 RX，PC7 接模块 TX（只查看输出时可不接），两端 GND 共地。串口助手用文本模式查看；VOFA+ FireWater 气压显示需只选择 `baro:` 行。

### CH1 舵机控制

| CH1 | 舵机角度 | PWM 脉宽 |
| --- | --- | --- |
| 544 | 0°，左端 | 500 μs |
| 942 | 72.5°，居中 | 1500 μs |
| 1340 | 145°，右端 | 2500 μs |

参数位于 `sbus_servo_control.h`，当前中位取输入范围中点 942。输入先限制在 544～1340，角度跟随拨杆位置；超出范围不会超出舵机行程。机械方向相反时可将 `APP_SBUS_SERVO_DIRECTION` 改为 -1。保留 PB8 / TIM4_CH3、50 Hz 舵机 PWM。

没有 SBUS 样本、接收机 failsafe/frame_lost 置位，或距最近有效帧超过 100 ms 时回中；有效帧恢复后重新跟随 CH1。反复发布旧样本不会刷新帧时间戳。

### CH3 无刷电机调速

参数位于 `sbus_motor_control.h`，CH3 范围为 312～1912。通过 `BLDC_SetCommutationPeriodUs()` 调速，使用原有开环六步正转，不使用只改变占空比的旧 `BLDC_SetSpeed()` 接口。

| CH3 | 电机行为 | 目标换相周期 | 理论转速设定（现有 6 对极配置） |
| --- | --- | --- | --- |
| 312～332 | 停机，低端 20 点死区 | 无 | 0 |
| 333 | 最低运行档 | 20000 μs | 约 83 rpm |
| 1112 | 中间油门 | 约 5269 μs | 约 316 rpm |
| 1912 及以上 | 原有最高运行档 | 3000 μs | 约 556 rpm |

有效运行区按理论转速线性插值后反算换相周期；保留 16% 运行占空比、250 rpm/s 加速、350 rpm/s 减速和预充电/定位/起转流程。运行时改变 CH3 只更新目标，不反复重启定位。当前没有实际速度反馈，上述为开环设定值，实际转速尚未上板验证。

上电先回最低油门，再推高启动。无有效样本、failsafe/frame_lost、驱动故障或帧龄超过 100 ms 时，20 ms 电机任务停止输出，并撤销启动许可；接收恢复后需再次回最低油门。新增调试字段 `motor_target_rpm`、`motor_target_period_us`、`motor_armed`、`motor_failsafe`。

### 验证

- Debug、Release 编译通过；固件启动检查确认创建全部六个任务，舵机启动脉宽为 1500 μs。
- `tests/run_app_arm_test.py` 覆盖 CH1 全范围/端点/中点/限幅、失联/标志回中、时间戳回绕、1000 组真实 SBUS 解码、UART 错帧与帧间隔恢复、SBUS2 帧尾、打印节流/队列快照/缓冲边界，以及 CH3 映射/低油门启动许可/失联停机/恢复和不重复启动。
- `tests/run_bldc_openloop_arm_test.py` 覆盖实际开环驱动的 CH3 加减速请求、换相与占空比归零停机。
- 测试使用 ARM 模拟及 HAL/RTOS 边界替代，不模拟电机机械响应或真实任务切换；本次未烧录。
