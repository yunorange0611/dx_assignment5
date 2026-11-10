#include "robot_control.h"
#include "robot_control_internal.h"
#include "task_protect.h"
#include "operator_control.h"
#include "dr16_bsp.h"
#include "vehicle_state_task.h"
#include "vehicle_state.h"
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
namespace {
DR16Status receiver{};
VehicleHealth vehicle_health{1234,0,1,1};
bool vehicle_available=true;
unsigned vehicle_reads=0;
}
extern "C" int Vehicle_GetHealth(VehicleHealth *out) {
    ++vehicle_reads;
    if(!out || !vehicle_available) return 0;
    *out=vehicle_health; return out->ready;
}
extern "C" int DR16_GetStatus(DR16Status *out) {
    if(!out) return 0;
    *out=receiver; return out->valid;
}
int test_robot_control() {
    static_assert(ROBOT_CONTROL_API_VERSION==0x0203u);
    static_assert(unsigned(RobotMode::Standby)==ROBOT_STANDBY);
    static_assert(unsigned(RobotMode::Drive)==ROBOT_DRIVE);
    static_assert(unsigned(DriveGear::Fast)==ROBOT_GEAR_FAST);
#if YAW_SUSPENDED_BENCH
    // Dedicated yaw build keeps all chassis commands disabled; default false
    // confirmations prevent tuning requests from enabling even with valid RX.
    receiver={}; receiver.valid=1; receiver.consecutive_frames=3; receiver.received_ms=1234;
    receiver.frame.switches[kOperatorConfig.left_switch_index]=DR16_SWITCH_UP;
    receiver.frame.switches[kOperatorConfig.right_switch_index]=DR16_SWITCH_MID;
    ArmControl arm(kArmConfig); RobotControlStatus s{};
    Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
    CHECK(!s.active && s.emergency_stop && s.stop_chassis && s.longitudinal_brake);
    CHECK(s.longitudinal_speed_request==0 && s.turn_rate_request==0 && !s.fast_requested);
    CHECK(s.protection_current_reasons & PROTECT_CONFIG);
    CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    const uint8_t feedback[8]={3,232,0,0,0,0,25,0};
    CHECK(arm.receive(1,arm.feedback_id(Yaw),feedback,8,1234));
    g_robot_debug_command=ROBOT_DEBUG_CAPTURE_ZERO; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().joint[Yaw].calibrated);
    for(unsigned j=1;j<kJointCount;++j) CHECK(!arm.status().joint[j].calibrated);
    g_yaw_debug_request.mode=1; g_yaw_debug_request.speed_deg_s=2;
    g_yaw_debug_request.duration_ms=100; g_yaw_debug_request.suspended_confirm=0x59415753;
    g_yaw_debug_request.request_seq=1; Robot_Execute(arm,1234,false);
    CHECK(g_yaw_debug_status.consumed_seq==1 && !g_yaw_debug_status.active);
    CHECK(arm.status().enabled_mask==0);
    const auto epoch=s.stop_epoch; Robot_StopControl(); Robot_Execute(arm,1234,false);
    CHECK(Robot_GetControlStatus(&s) && s.stop_epoch!=epoch);
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
    CHECK(!g_yaw_debug_status.active && arm.status().enabled_mask==0);
    Frame frames[kJointCount]{}; const auto count=arm.step(1234,frames);
    unsigned yaw_frames=0;
    for(unsigned i=0;i<count;++i) {
        CHECK(frames[i].bus==1);
        if(frames[i].id==0x2fe) { ++yaw_frames; for(uint8_t b:frames[i].data) CHECK(b==0); }
    }
    CHECK(yaw_frames==1);
    CHECK(vehicle_reads==0); // Isolated existing-owner yaw test does not depend on full-vehicle sensors.
    return 0;
#else
    RobotControlStatus s{};
    CHECK(!Robot_GetControlStatus(nullptr)); CHECK(!Robot_GetControlStatus(&s));
    CHECK(!Robot_SetInterlocks(nullptr));
    RobotInterlocks locks{};
    locks.chassis_stopped=1; locks.arm_allows_drive=1; locks.arm_allows_fast=1;
    locks.drive_yaw_allowed=1; locks.drive_yaw_min_deg=-20; locks.drive_yaw_max_deg=20;
    locks.drive_yaw_max_speed_deg_s=2;
    CHECK(Robot_SetInterlocks(&locks));
    receiver.valid=1; receiver.consecutive_frames=3; receiver.received_ms=1234;
    receiver.frame.switches[0]=DR16_SWITCH_UP; receiver.frame.switches[1]=DR16_SWITCH_MID;
    CHECK(Robot_RequestControl());
    ArmControl arm(kArmConfig); Robot_Execute(arm,1234,false);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.updated_ms==1234 && s.ready && s.remote_online);
    CHECK(s.joint_mask==kArmConfig.joint_mask && g_robot_status.joint_mask==s.joint_mask);
    CHECK(!s.active && s.protection_disabled && s.emergency_stop && s.stop_chassis);
    CHECK(s.protection_current_reasons & PROTECT_CONFIG);
    CHECK(!(s.protection_current_reasons & PROTECT_REMOTE));
    CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    CHECK(!(s.protection_current_reasons & (PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED | PROTECT_VEHICLE_STATE | PROTECT_POWER)));
    CHECK(s.longitudinal_brake && s.reset_chassis_motion && !s.yaw_follow);
    CHECK(!s.fast_requested && s.drive_gear==ROBOT_GEAR_SLOW);
    CHECK(s.longitudinal_speed_request==0 && s.turn_rate_request==0 && arm.status().enabled_mask==0);
    for(unsigned bad=0;bad<8;++bad) {
        auto invalid=locks;
        if(bad==0) invalid.chassis_stopped=2;
        if(bad==1) invalid.arm_allows_drive=2;
        if(bad==2) invalid.arm_allows_fast=2;
        if(bad==3) invalid.drive_yaw_allowed=2;
        if(bad==4) invalid.drive_yaw_min_deg=std::numeric_limits<float>::quiet_NaN();
        if(bad==5) invalid.drive_yaw_max_deg=100;
        if(bad==6) invalid.drive_yaw_max_speed_deg_s=0;
        if(bad==7) invalid.drive_yaw_min_deg=21;
        CHECK(!Robot_SetInterlocks(&invalid));
    }
    Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
    if(kArmConfig.joint_mask==kFourAxisJoints) CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    else CHECK(s.protection_current_reasons & PROTECT_INTERLOCK); // 整车配置无效输入必须立刻撤掉旧许可。
    locks.drive_yaw_allowed=0; locks.drive_yaw_min_deg=0; locks.drive_yaw_max_deg=0;
    locks.drive_yaw_max_speed_deg_s=0; CHECK(Robot_SetInterlocks(&locks));
    Robot_StopControl(); Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
    CHECK(!s.active && s.emergency_stop && s.turn_rate_request==0);
    Status fault{}; fault.faults=FeedbackTimeout; Robot_ReportArmState(fault);
    CHECK(Robot_GetControlStatus(&s));
    CHECK((s.protection_latched_reasons & PROTECT_ARM_FAULT) && s.protection_disabled);
    CHECK(s.stop_chassis && s.emergency_stop && s.reset_chassis_motion);
    // Stable debugger command symbol consumes each command once and preserves default disable.
    for(unsigned j=0;j<kJointCount;++j) if(kArmConfig.joint_mask & (1u<<j)) {
        if(j==BigPitch || j==SmallPitch) {
            const uint8_t d[8]={uint8_t(0x10+j),0x80,0,0x80,0x08,0,25,25};
            CHECK(arm.receive(1,arm.feedback_id(j),d,8,1234));
        } else {
            const uint8_t d[8]={3,232,0,0,0,0,25,0};
            CHECK(arm.receive(1,arm.feedback_id(j),d,8,1234));
        }
    }
    g_robot_debug_command=ROBOT_DEBUG_CAPTURE_ZERO;
    Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().last_result==Result::Ok);
    CHECK(arm.status().enabled_mask==0);
    for(unsigned j=0;j<kJointCount;++j)
        CHECK(arm.status().joint[j].calibrated==bool(kArmConfig.joint_mask & (1u<<j)));
    arm.transport_fault(); CHECK(arm.status().faults & TransportError);
    g_robot_debug_command=ROBOT_DEBUG_CLEAR_FAULTS; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().last_result==Result::Ok);
    CHECK(arm.status().faults==0 && arm.status().enabled_mask==0);
    g_robot_debug_command=ROBOT_DEBUG_REQUEST_CONTROL; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().enabled_mask==0);
    g_robot_debug_command=ROBOT_DEBUG_STOP; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE); CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH
    {
        const uint32_t vehicle_mask=PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED |
            PROTECT_VEHICLE_STATE | PROTECT_POWER;
        const uint32_t source[]={VehicleConfiguration,VehicleChassisStale,VehicleArmStale,
            VehicleImuInvalid,VehicleTipped,VehicleWheelFault,VehicleEnergyInhibited,VehicleArmFault};
        const uint32_t expected[]={PROTECT_VEHICLE_STATE,PROTECT_CHASSIS,PROTECT_VEHICLE_STATE,
            PROTECT_IMU,PROTECT_TIPPED,PROTECT_CHASSIS,PROTECT_POWER,PROTECT_VEHICLE_STATE};
        for(unsigned n=0;n<8;++n) {
            vehicle_health={1234,source[n],1,0};
            Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
            CHECK((s.protection_current_reasons & vehicle_mask)==expected[n]);
            CHECK((s.protection_latched_reasons & expected[n]) && s.protection_disabled);
            CHECK(s.emergency_stop && s.stop_chassis && !s.active && arm.status().enabled_mask==0);
        }
        vehicle_health={1234,VehicleWheelFault | VehicleImuInvalid | VehicleTipped | VehicleEnergyInhibited,1,0};
        Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
        CHECK((s.protection_current_reasons & vehicle_mask)==(PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED | PROTECT_POWER));
        for(unsigned bad=0;bad<7;++bad) {
            vehicle_health={1234,0,1,1}; vehicle_available=true;
            if(bad==0) vehicle_health.ready=0;
            if(bad==1) vehicle_available=false;
            if(bad==2) vehicle_health.updated_ms=1173; // 61 ms: stale publication
            if(bad==3) vehicle_health.healthy=2;
            if(bad==4) vehicle_health.healthy=0; // Unknown unhealthy state cannot be treated healthy.
            if(bad==5) vehicle_health.reasons=1u<<31;
            if(bad==6) vehicle_health.updated_ms=1235;
            Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
            CHECK((s.protection_current_reasons & vehicle_mask)==PROTECT_VEHICLE_STATE);
            CHECK(s.protection_disabled && s.emergency_stop);
        }
        vehicle_available=true; vehicle_health={1174,0,1,1};
        Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
        CHECK(!(s.protection_current_reasons & vehicle_mask)); // Exact timeout remains fresh.
        CHECK(s.protection_disabled && !s.active); // Healthy source alone cannot restart.
        CHECK(s.protection_latched_reasons & PROTECT_VEHICLE_STATE);
        CHECK(vehicle_reads>0);
    }
#else
    CHECK(vehicle_reads==0); // Teammate's four-axis bench remains independent.
#endif
    return 0;
#endif
}
