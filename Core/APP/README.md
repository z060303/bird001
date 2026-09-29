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
