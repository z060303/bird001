cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# ============================================================
# Compiler settings
# ============================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# ============================================================
# Build type
# ============================================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()


# ============================================================
# Project name
# ============================================================

set(CMAKE_PROJECT_NAME bird001)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# ============================================================
# Project
# ============================================================

project(${CMAKE_PROJECT_NAME})

message("Build type: " ${CMAKE_BUILD_TYPE})


# ============================================================
# STM32CubeProgrammer
# ============================================================

find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI
        HINTS
        "$ENV{STM32CUBECLT_ROOT}/STM32CubeProgrammer/bin"
        "D:/app/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin"
)

if(NOT STM32_PROGRAMMER_CLI)
    message(WARNING
            "STM32_Programmer_CLI was not found; "
            "the flash target will be unavailable"
    )
endif()


# ============================================================
# Enable C / ASM
# ============================================================

enable_language(C ASM)


# ============================================================
# Create executable
#
# 注意：
# 这里不要再放 .h 文件
# ============================================================

add_executable(${CMAKE_PROJECT_NAME}
        Core/BSP/BLDC.c
        Core/BSP/BLDC_OpenLoop1637.c
        Core/BSP/BLDC.h
        Core/BSP/BLDC_BEMF.c
        Core/BSP/BLDC_BEMF.h
        Core/BSP/Servo.c
        Core/BSP/Servo.h
)


# ============================================================
# Add STM32CubeMX generated sources
# ============================================================

add_subdirectory(cmake/stm32cubemx)


# ============================================================
# BSP source files
#
# Core/BSP 下所有 .c 自动加入工程
# ============================================================

file(GLOB BSP_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP/*.c"
)


# ============================================================
# Link directories
# ============================================================

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

        # Add user defined library search paths

)


# ============================================================
# Add user sources
# ============================================================

target_sources(${CMAKE_PROJECT_NAME} PRIVATE

        # BSP
        ${BSP_SOURCES}

)


# ============================================================
# Include directories
#
# Core/BSP 加入之后：
#
# #include "PressureSensor.h"
#
# 就可以直接找到：
#
# Core/BSP/PressureSensor.h
# ============================================================

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP

)


# ============================================================
# Project symbols
# ============================================================

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

        # Add user defined symbols

)


# ============================================================
# Remove wrong libob.a dependency
# ============================================================

list(REMOVE_ITEM
        CMAKE_C_IMPLICIT_LINK_LIBRARIES
        ob
)

# ============================================================
# Enable printf float support
# ============================================================

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)


# ============================================================
# Link libraries
# ============================================================

target_link_libraries(${CMAKE_PROJECT_NAME}

        stm32cubemx

)


# ============================================================
# Flash target
# ============================================================

if(STM32_PROGRAMMER_CLI)

    add_custom_target(flash

            COMMAND "${STM32_PROGRAMMER_CLI}"

            -c port=SWD mode=UR freq=100

            -d "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"

            -v

            -rst

            DEPENDS ${CMAKE_PROJECT_NAME}

            USES_TERMINAL

            COMMENT
            "Programming ${CMAKE_PROJECT_NAME} through ST-LINK"

    )

endif()

# FreeRTOS V11.1.0, Cortex-M4F (hardware floating-point context preserved).
set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS-Kernel")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
    Core/APP/app_freertos.c
    Core/APP/roll_control.c
    Core/APP/rtos_hooks.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/APP
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_USE_FREERTOS=1)
