#pragma once
#include "arm_task.h"
#include "chassis_task.h"
#include "task_ins.h"
#include "robot_control.h"
#include <cstdint>
namespace engineer {
struct VehicleConfig {
    bool posture_confirmed=false;
    float drive_pitch_deg=15,drive_roll_deg=15,drive_yaw_deg=20;
    float drive_yaw_speed_deg_s=3;
    uint32_t status_timeout_ms=60;
};
inline constexpr VehicleConfig kVehicleConfig{};
enum VehicleFault : uint32_t {
    VehicleNoFault=0, VehicleConfiguration=1, VehicleChassisStale=2,
    VehicleArmStale=4, VehicleImuInvalid=8, VehicleTipped=16,
    VehicleWheelFault=32, VehicleEnergyInhibited=64, VehicleArmFault=128
};
struct VehicleState {
    uint32_t updated_ms=0, reasons=0;
    bool healthy=false;
    RobotInterlocks locks{};
};
VehicleState evaluate_vehicle(uint32_t now,const VehicleConfig &cfg,const ChassisStatus &chassis,
                              const ArmStatus &arm,const InsStatus &imu,float yaw_half_range_deg);
}
