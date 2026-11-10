cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dx_assignment5_freertos)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
option(ARM_FOUR_AXIS_BENCH "Four-axis arm bench: no yaw or chassis control" ON)
option(DM_ID_CONFIG_MODE "Standalone CAN1 DM ID configuration; no arm control" OFF)
option(CHASSIS_SUSPENDED_BENCH "Explicit suspended chassis pulse testing only" OFF)
option(YAW_SUSPENDED_BENCH "Explicit limited Yaw tuning in the existing owner" OFF)
option(VEHICLE_SAFETY_ENABLED "Real IMU, posture interlocks and conservative power stopping" ON)
if(YAW_SUSPENDED_BENCH AND (ARM_FOUR_AXIS_BENCH OR CHASSIS_SUSPENDED_BENCH OR DM_ID_CONFIG_MODE))
    message(FATAL_ERROR "Yaw bench requires the five-axis owner and excludes other test modes")
endif()
if(CHASSIS_SUSPENDED_BENCH AND (NOT ARM_FOUR_AXIS_BENCH OR DM_ID_CONFIG_MODE))
    message(FATAL_ERROR "Chassis bench requires the disabled four-axis owner and excludes ID configuration")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Explicit source ownership prevents archived/template drivers from silently
# rejoining the firmware when files are added to the repository.
set(BSP_SOURCES
    1Bsp/CAN_bsp.cpp
    1Bsp/dr16_bsp.c
    1Bsp/bsp_dwt.cpp
    1Bsp/bsp_imu.cpp
)
set(CONTROL_SOURCES
    2User_Driver/Arm/arm_control.cpp
    2User_Driver/Arm/arm_config.cpp
    2User_Driver/Arm/dm_id_config.cpp
    2User_Driver/Remote/operator_control.cpp
    2User_Driver/Remote/operator_config.cpp
    2User_Driver/Chassis/chassis_motor.cpp
    2User_Driver/Chassis/chassis_control.cpp
    2User_Driver/Chassis/chassis_bench.cpp
    2User_Driver/Power/power_meter.cpp
    2User_Driver/Power/chassis_power.cpp
    2User_Driver/Imu/attitude_estimator.cpp
    2User_Driver/Vehicle/vehicle_state.cpp
)
set(TASK_SOURCES
    3Task/arm_task.cpp
    3Task/robot_control.cpp
    3Task/yaw_service.cpp
    3Task/chassis_task.cpp
    3Task/task_ins.cpp
    3Task/vehicle_state_task.cpp
    3Task/task_protect.cpp
    3Task/dm_id_config_task.cpp
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${BSP_SOURCES}
        ${CONTROL_SOURCES}
        ${TASK_SOURCES}
        Core/Src/usart.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        1Bsp
        2User_Driver
        3Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_FOUR_AXIS_BENCH=$<BOOL:${ARM_FOUR_AXIS_BENCH}>
    DM_ID_CONFIG_MODE=$<BOOL:${DM_ID_CONFIG_MODE}>
    CHASSIS_SUSPENDED_BENCH=$<BOOL:${CHASSIS_SUSPENDED_BENCH}>
    YAW_SUSPENDED_BENCH=$<BOOL:${YAW_SUSPENDED_BENCH}>
    VEHICLE_SAFETY_ENABLED=$<BOOL:${VEHICLE_SAFETY_ENABLED}>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(DM_ID_CONFIG_MODE)
    set(FIRMWARE_NAME dm_id_config)
elseif(CHASSIS_SUSPENDED_BENCH)
    set(FIRMWARE_NAME chassis_suspended)
elseif(YAW_SUSPENDED_BENCH)
    set(FIRMWARE_NAME yaw_suspended)
else()
    set(FIRMWARE_NAME engineer_arm)
endif()
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.hex
)
