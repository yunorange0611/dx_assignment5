#include "robot_control.h"
#include "robot_control_internal.h"
#include "operator_control.h"
#include "yaw_config.h"
#include "yaw_runtime.h"
#include "task_protect.h"
#include "dr16_bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <cmath>
extern "C" {
RobotControlStatus g_robot_status{};
volatile uint32_t g_robot_debug_command=ROBOT_DEBUG_IDLE;
volatile YawLimitRequest g_yaw_limit_request{};
YawLimitStatus g_yaw_limit_status{};
volatile YawDebugRequest g_yaw_debug_request{};
YawDebugStatus g_yaw_debug_status{};
}
namespace {
engineer::Interlocks shared_interlocks;
bool arm_requested=false,stop_requested=false;
uint32_t stop_epoch=0;
void publish_protection(RobotControlStatus &s) {
    const auto p=Task_Protect_GetStatus();
    s.protection_disabled=p.disabled;
    s.protection_current_reasons=p.current_reasons;
    s.protection_latched_reasons=p.latched_reasons;
    s.protection_last_trip_ms=p.last_trip_ms;
}
}
extern "C" int Robot_SetInterlocks(const RobotInterlocks *v) {
    const auto &yaw=engineer::kArmConfig.joint[engineer::Yaw];
    YawLimitStatus limits{};
    taskENTER_CRITICAL(); limits=g_yaw_limit_status; taskEXIT_CRITICAL();
    const float min=limits.ready ? limits.min_deg : yaw.min_deg;
    const float max=limits.ready ? limits.max_deg : yaw.max_deg;
    const float speed=limits.ready ? limits.max_speed_deg_s : yaw.max_speed_deg_s;
    const bool valid=v && v->chassis_stopped<=1 && v->arm_allows_drive<=1 &&
        v->arm_allows_fast<=1 && v->drive_yaw_allowed<=1 &&
        (!v->drive_yaw_allowed ||
         (std::isfinite(v->drive_yaw_min_deg) && std::isfinite(v->drive_yaw_max_deg) &&
          std::isfinite(v->drive_yaw_max_speed_deg_s) && v->drive_yaw_min_deg<v->drive_yaw_max_deg &&
          v->drive_yaw_min_deg>=min && v->drive_yaw_max_deg<=max &&
          v->drive_yaw_max_speed_deg_s>0 && v->drive_yaw_max_speed_deg_s<=speed));
    engineer::Interlocks next;
    if(valid) {
        next.valid=true; next.received_ms=HAL_GetTick(); next.chassis_stopped=v->chassis_stopped;
        next.arm_allows_drive=v->arm_allows_drive; next.arm_allows_fast=v->arm_allows_fast;
        next.drive_yaw_allowed=v->drive_yaw_allowed;
        if(v->drive_yaw_allowed) {
            next.drive_yaw_min_deg=v->drive_yaw_min_deg; next.drive_yaw_max_deg=v->drive_yaw_max_deg;
            next.drive_yaw_max_speed_deg_s=v->drive_yaw_max_speed_deg_s;
        }
    }
    taskENTER_CRITICAL(); shared_interlocks=next; taskEXIT_CRITICAL(); return valid;
}
extern "C" int Robot_RequestControl() {
    taskENTER_CRITICAL(); arm_requested=true; taskEXIT_CRITICAL(); return 1;
}
extern "C" int Yaw_GetLimitStatus(YawLimitStatus *out) {
    if(!out) return 0;
    taskENTER_CRITICAL(); *out=g_yaw_limit_status; taskEXIT_CRITICAL();
    return out->ready;
}
extern "C" void Robot_StopControl() {
    taskENTER_CRITICAL(); stop_requested=true; ++stop_epoch; taskEXIT_CRITICAL();
}
extern "C" int Robot_GetControlStatus(RobotControlStatus *v) {
    if(!v) return 0;
    taskENTER_CRITICAL(); *v=g_robot_status; taskEXIT_CRITICAL(); return v->ready;
}
void Robot_Execute(engineer::ArmControl &arm,uint32_t now,bool force_stop) {
    using namespace engineer;
    static ArmOperator controller(kArmConfig,kOperatorConfig);
    static bool initialized=false;
    static uint32_t previous_ms=0;
    if(!initialized) Task_Protect_Init();
    Interlocks locks; bool arm_request,stop_request; uint32_t debug_command;
    taskENTER_CRITICAL();
    locks=shared_interlocks; arm_request=arm_requested; stop_request=stop_requested;
    arm_requested=false; stop_requested=false;
    debug_command=g_robot_debug_command; g_robot_debug_command=ROBOT_DEBUG_IDLE;
    taskEXIT_CRITICAL();
    DR16Status rx{}; DR16_GetStatus(&rx);
    now=HAL_GetTick();
    // 仍在唯一控制任务内执行；关控优先，不让校零或开控绕过现有保护。
    if (debug_command==ROBOT_DEBUG_STOP || force_stop) {
        taskENTER_CRITICAL(); ++stop_epoch; taskEXIT_CRITICAL();
        stop_request=true;
    }
    if (!force_stop && !stop_request) {
        switch (debug_command) {
        case ROBOT_DEBUG_CAPTURE_ZERO:
#if YAW_SUSPENDED_BENCH
            arm.capture_zero(1u<<Yaw,now);
#else
            arm.capture_zero(kArmConfig.joint_mask,now);
#endif
            stop_request=true; break;
        case ROBOT_DEBUG_CLEAR_FAULTS: arm.clear_faults(); stop_request=true; break;
        case ROBOT_DEBUG_REQUEST_CONTROL: arm_request=true; break;
        case ROBOT_DEBUG_IDLE: break;
        default: stop_request=true; break;
        }
    }
    RemoteInput input{rx.frame,rx.received_ms,rx.valid && rx.consecutive_frames>=3};
    const uint32_t elapsed=initialized ? uint32_t(now-previous_ms) : kArmConfig.control_period_ms;
    previous_ms=now; initialized=true;
    const bool remote_ok=input.valid && uint32_t(now-input.received_ms)<=kOperatorConfig.remote_timeout_ms;
    const bool bench=kArmConfig.joint_mask==kFourAxisJoints;
    // Write requests are consumed only here, in the existing CAN1 owner. The
    // operator, interlock validation snapshot and inner loop see one version.
    YawLimitRequest limit_request{};
    taskENTER_CRITICAL();
    limit_request.half_range_deg=g_yaw_limit_request.half_range_deg;
    limit_request.request_seq=g_yaw_limit_request.request_seq;
    taskEXIT_CRITICAL();
    if(limit_request.request_seq!=g_yaw_limit_status.consumed_seq) {
        Result applied=Result::InvalidArgument;
        if(bench) applied=Result::InvalidArgument;
        else if(!kYawSafetyConfirmed) applied=Result::ConfigUnconfirmed;
        else if(std::isfinite(limit_request.half_range_deg) && limit_request.half_range_deg>kYawBrakingMarginDeg &&
                limit_request.half_range_deg<=kYawAbsoluteHalfRangeDeg) {
            YawRuntimeLimits proposed{-limit_request.half_range_deg,limit_request.half_range_deg,
                kArmConfig.joint[Yaw].max_speed_deg_s,kYawBrakingAccelDegS2,kYawBrakingMarginDeg,
                std::fmin(kYawMaxTargetLeadDeg,limit_request.half_range_deg),limit_request.request_seq};
            applied=arm.set_yaw_limits(proposed,now);
            if(applied==Result::Ok) {
                controller.set_yaw_limits(proposed);
                const bool incompatible=locks.drive_yaw_allowed &&
                    (locks.drive_yaw_min_deg<proposed.min_deg || locks.drive_yaw_max_deg>proposed.max_deg ||
                     locks.drive_yaw_max_speed_deg_s>proposed.max_speed_deg_s);
                taskENTER_CRITICAL();
                g_yaw_limit_status.active_half_range_deg=limit_request.half_range_deg;
                g_yaw_limit_status.min_deg=proposed.min_deg; g_yaw_limit_status.max_deg=proposed.max_deg;
                g_yaw_limit_status.max_speed_deg_s=proposed.max_speed_deg_s;
                g_yaw_limit_status.active_seq=proposed.version; g_yaw_limit_status.ready=1;
                if(incompatible) shared_interlocks.valid=false;
                taskEXIT_CRITICAL();
                if(incompatible) locks.valid=false;
            }
        }
        taskENTER_CRITICAL();
        g_yaw_limit_status.consumed_seq=limit_request.request_seq;
        g_yaw_limit_status.result=uint32_t(applied);
        taskEXIT_CRITICAL();
    }
#if YAW_SUSPENDED_BENCH
    {
    // Explicit isolated test build: the existing ArmControl owns and emits the
    // pulse. Neither a second motor object nor a second CAN publisher exists.
    static YawBenchGate yaw_gate;
    bool neutral=remote_ok;
    for(int channel:rx.frame.channel) neutral &= std::abs(channel)<=30;
    neutral &= rx.frame.switches[kOperatorConfig.left_switch_index]==DR16_SWITCH_UP &&
               rx.frame.switches[kOperatorConfig.right_switch_index]==DR16_SWITCH_MID;
    const auto &ys=arm.status().joint[Yaw];
    const bool yaw_fresh=ys.received && uint32_t(now-ys.last_feedback_ms)<=kArmConfig.dji_unwrap_gap_ms && ys.calibrated;
    const bool config_ok=kArmConfig.joint_mask==kAllJoints && arm.status().configuration_valid &&
        kArmConfig.hardware_confirmed && kYawSafetyConfirmed && kOperatorConfig.mapping_confirmed &&
        arm.yaw_limits().version!=0;
    const bool timing_ok=elapsed>0 && elapsed<=kArmConfig.control_period_ms+2;
    const TaskProtectInput p{uint8_t(config_ok),uint8_t(neutral),1,uint8_t(force_stop || stop_request),
        uint8_t(timing_ok),uint8_t(arm.status().faults!=0)};
    Task_Protect_Tick(&p,now);
    YawBenchRequest request{};
    taskENTER_CRITICAL();
    request.mode=g_yaw_debug_request.mode; request.speed_deg_s=g_yaw_debug_request.speed_deg_s;
    request.duration_ms=g_yaw_debug_request.duration_ms; request.suspended_confirm=g_yaw_debug_request.suspended_confirm;
    request.request_seq=g_yaw_debug_request.request_seq;
    const uint32_t current_stop_epoch=stop_epoch;
    taskEXIT_CRITICAL();
    YawBenchHealth h;
    h.ready=config_ok && neutral && timing_ok && yaw_fresh && ys.temperature_c<kArmConfig.joint[Yaw].max_temperature_c &&
        (arm.status().enabled_mask & ~(1u<<Yaw))==0;
    h.enabled=ys.enabled; h.faulted=arm.status().faults!=0;
    h.stationary=std::fabs(ys.velocity_deg_s)<=kArmConfig.joint[Yaw].stationary_speed_deg_s;
    h.stop=force_stop || stop_request; h.stop_epoch=current_stop_epoch;
    auto pulse=yaw_gate.step(now,request,h);
    if(pulse.active) {
        if((!ys.enabled && (!Task_Protect_TryArm() || arm.enable(1u<<Yaw,now)!=Result::Ok)) ||
           arm.set_effort_limit(Yaw,std::fmin(150.0f,kArmConfig.joint[Yaw].max_effort))!=Result::Ok ||
           arm.set_yaw_test_speed(pulse.speed_deg_s,now)!=Result::Ok) {
            arm.disable(); Task_Protect_Trip(PROTECT_ENABLE_REJECTED,now);
            h.stop=true; pulse=yaw_gate.step(now,request,h);
        }
    } else arm.disable();
    RobotControlStatus next{}; next.ready=1; next.updated_ms=now; next.joint_mask=kArmConfig.joint_mask;
    next.mode=ROBOT_SAFE; next.emergency_stop=1; next.stop_chassis=1; next.longitudinal_brake=1;
    next.reset_chassis_motion=1; next.wait_neutral=1; next.stop_epoch=current_stop_epoch;
    next.remote_online=remote_ok; next.remote_age_ms=now-input.received_ms;
    publish_protection(next);
    taskENTER_CRITICAL();
    g_robot_status=next;
    g_yaw_debug_status={pulse.consumed_seq,pulse.result,pulse.until_ms,pulse.speed_deg_s,uint8_t(pulse.active)};
    taskEXIT_CRITICAL();
    return;
    }
#endif
    const bool interlock_ok=bench || (locks.valid && uint32_t(now-locks.received_ms)<=kOperatorConfig.interlock_timeout_ms);
    const TaskProtectInput protection_input{
        uint8_t(arm.status().configuration_valid && kArmConfig.hardware_confirmed &&
                kArmConfig.pitch.mapping_confirmed && kOperatorConfig.mapping_confirmed &&
                (bench || (kYawSafetyConfirmed && arm.yaw_limits().version!=0))),
        uint8_t(remote_ok),uint8_t(interlock_ok),uint8_t(force_stop || stop_request),
        uint8_t(elapsed>0 && elapsed<=kArmConfig.control_period_ms+2),uint8_t(arm.status().faults!=0)};
    Task_Protect_Tick(&protection_input,now);
    auto result=controller.step(input,locks,arm.status(),now,arm_request,
                                Task_Protect_GetStatus().current_reasons!=0);
    if (bench && result.mode==RobotMode::Safe && result.block==OperatorBlock::NeedArm &&
        Task_Protect_GetStatus().current_reasons==0)
        Task_Protect_Trip(PROTECT_STOP_REQUEST,now);
    auto reject=[&]() {
        result.motion_epoch=controller.enable_result(false);
        result.wait_neutral=true; result.active=false; result.disable_arm=true;
        result.emergency_stop=true; result.stop_chassis=true; result.longitudinal_brake=true;
        result.reset_chassis_motion=true; result.fast_requested=false; result.drive_gear=DriveGear::Slow;
        result.yaw_follow=false; result.longitudinal_speed_request=0; result.turn_rate_request=0;
        result.block=OperatorBlock::ArmUnavailable; arm.disable();
    };
    if(!bench) {
        const auto &l=arm.yaw_limits();
        const bool drive_envelope=result.mode==RobotMode::Drive && locks.drive_yaw_allowed;
        const Result envelope=arm.set_yaw_envelope(drive_envelope ? locks.drive_yaw_min_deg : l.min_deg,
            drive_envelope ? locks.drive_yaw_max_deg : l.max_deg,
            drive_envelope ? locks.drive_yaw_max_speed_deg_s : l.max_speed_deg_s);
        if(envelope!=Result::Ok && result.active) {
            // Preserve the previously safe envelope, request normal parking and
            // retain the existing joint hold policy until the posture is fixed.
            result.stop_chassis=true; result.longitudinal_brake=true;
            result.reset_chassis_motion=true; result.longitudinal_speed_request=0; result.turn_rate_request=0;
            result.drive_yaw_limited=true; result.block=OperatorBlock::DriveYawDenied;
            result.target_deg[Yaw]=arm.status().joint[Yaw].target_deg;
        }
    }
    if(result.request_enable) {
        if(!Task_Protect_TryArm() || arm.enable(kArmConfig.joint_mask,now)!=Result::Ok) {
            Task_Protect_Trip(PROTECT_ENABLE_REJECTED,now); reject();
        } else controller.enable_result(true);
    }
    if(Task_Protect_GetStatus().disabled && result.active) reject();
    if(result.disable_arm) arm.disable();
    else if(result.active) {
        if(arm.set_effort_limit(Gripper,result.gripper_effort_limit)!=Result::Ok ||
           arm.set_targets(kArmConfig.joint_mask,result.target_deg,now)!=Result::Ok) {
            Task_Protect_Trip(PROTECT_ARM_FAULT,now); reject();
        }
    }
    RobotControlStatus next{};
    next.updated_ms=now;
    taskENTER_CRITICAL(); next.stop_epoch=stop_epoch; taskEXIT_CRITICAL();
    next.joint_mask=kArmConfig.joint_mask;
    next.ready=1; next.mode=uint8_t(result.mode); next.block=uint8_t(result.block);
    next.active=result.active; next.wait_neutral=result.wait_neutral;
    next.remote_online=remote_ok;
    next.remote_age_ms=now-input.received_ms;
    next.interlock_age_ms=bench ? 0 : now-locks.received_ms; // 台架不使用此输入，非伪造停车反馈
    next.emergency_stop=result.emergency_stop; next.stop_chassis=result.stop_chassis;
    next.longitudinal_brake=result.longitudinal_brake; next.reset_chassis_motion=result.reset_chassis_motion;
    next.motion_epoch=result.motion_epoch; next.yaw_follow=0;
    next.fast_requested=result.fast_requested; next.drive_gear=uint8_t(result.drive_gear);
    next.drive_yaw_limited=result.drive_yaw_limited;
    next.longitudinal_speed_request=result.longitudinal_speed_request;
    next.turn_rate_request=result.turn_rate_request; next.yaw_relative_deg=result.yaw_relative_deg;
    next.gripper_effort_limit=result.gripper_effort_limit;
    for(unsigned i=0;i<kJointCount;++i) next.target_deg[i]=result.target_deg[i];
    publish_protection(next);
    taskENTER_CRITICAL(); g_robot_status=next; taskEXIT_CRITICAL();
}
void Robot_ReportArmState(const engineer::Status &s) {
    // arm.step中的反馈/驱动/轨迹故障发生在Robot_Execute之后，必须同周期覆盖共享状态。
    taskENTER_CRITICAL();
    if(s.faults || (g_robot_status.active && s.enabled_mask!=engineer::kArmConfig.joint_mask)) {
        Task_Protect_Trip(PROTECT_ARM_FAULT,HAL_GetTick());
        g_robot_status.active=0; g_robot_status.emergency_stop=1; g_robot_status.stop_chassis=1;
        g_robot_status.longitudinal_brake=1; g_robot_status.reset_chassis_motion=1;
        g_robot_status.fast_requested=0; g_robot_status.drive_gear=ROBOT_GEAR_SLOW;
        g_robot_status.yaw_follow=0; g_robot_status.longitudinal_speed_request=0; g_robot_status.turn_rate_request=0;
        g_robot_status.block=uint8_t(engineer::OperatorBlock::ArmUnavailable);
        publish_protection(g_robot_status);
    }
    taskEXIT_CRITICAL();
}
