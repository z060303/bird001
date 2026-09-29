# Vendored FreeRTOS kernel

Upstream: https://github.com/FreeRTOS/FreeRTOS-Kernel
Tag: V11.1.0
Commit: dbf70559b27d39c1fdb68dfb9a32140b6a6777a0
License: MIT (see LICENSE.md).

Unmodified subset: tasks.c, queue.c, list.c, include/, portable/GCC/ARM_CM4F/.
The project uses static allocation only; no heap implementation, software timers,
CMSIS-RTOS wrapper or additional ports are compiled.
Project configuration is in Core/Inc/FreeRTOSConfig.h.
