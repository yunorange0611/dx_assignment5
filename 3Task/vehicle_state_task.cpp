#include "vehicle_state_task.h"
#include "../2User_Driver/Vehicle/vehicle_state.h"
#include "arm_task.h"
#include "chassis_task.h"
#include "task_ins.h"
#include "robot_control.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
extern "C" { VehicleHealth g_vehicle_health{}; }
extern "C" int Vehicle_GetHealth(VehicleHealth *out) {
    if(!out) return 0;
    taskENTER_CRITICAL(); *out=g_vehicle_health; taskEXIT_CRITICAL(); return out->ready;
}
extern "C" void vehicle_state_task(void *) {
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !CHASSIS_SUSPENDED_BENCH && !YAW_SUSPENDED_BENCH && !DM_ID_CONFIG_MODE
    uint32_t tick=osKernelGetTickCount();
    for(;;) {
        ArmStatus arm{}; Arm_GetStatus(&arm);
        ChassisStatus chassis{}; Chassis_GetStatus(&chassis);
        InsStatus imu{}; INS_GetStatus(&imu);
        const uint32_t now=HAL_GetTick();
        // The same accepted Yaw runtime range is used by the mode/core owner.
        YawLimitStatus yaw{}; Yaw_GetLimitStatus(&yaw);
        const auto state=engineer::evaluate_vehicle(now,engineer::kVehicleConfig,chassis,arm,imu,yaw.active_half_range_deg);
        Robot_SetInterlocks(&state.locks);
        taskENTER_CRITICAL(); g_vehicle_health={now,state.reasons,1,uint8_t(state.healthy)}; taskEXIT_CRITICAL();
        const uint32_t current=osKernelGetTickCount();
        tick=int32_t(current-tick)>0 ? current+20 : tick+20;
        osDelayUntil(tick);
    }
#else
    for(;;) osDelay(100);
#endif
}
