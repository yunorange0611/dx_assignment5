#include "yaw_service.h"
#include "Arm/arm_control.h"
#include "Remote/operator_control.h"
#include "Arm/yaw_config.h"
#include "Arm/yaw_bench.h"
#include "task_protect.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>

extern "C" {
volatile YawLimitRequest g_yaw_limit_request{};
YawLimitStatus g_yaw_limit_status{};
volatile YawDebugRequest g_yaw_debug_request{};
YawDebugStatus g_yaw_debug_status{};
}

extern "C" int Yaw_GetLimitStatus(YawLimitStatus *out) {
    if (!out)
        return 0;
    taskENTER_CRITICAL();
    *out = g_yaw_limit_status;
    taskEXIT_CRITICAL();
    return out->ready;
}

namespace engineer {
bool Yaw_EnvelopeAllowed(float min_deg, float max_deg, float speed_deg_s) {
    const auto &yaw = kArmConfig.joint[Yaw];
    YawLimitStatus limits{};
    Yaw_GetLimitStatus(&limits);
    const float min = limits.ready ? limits.min_deg : yaw.min_deg;
    const float max = limits.ready ? limits.max_deg : yaw.max_deg;
    const float speed = limits.ready ? limits.max_speed_deg_s : yaw.max_speed_deg_s;
    return std::isfinite(min_deg) && std::isfinite(max_deg) && std::isfinite(speed_deg_s) &&
           min_deg < max_deg && min_deg >= min && max_deg <= max && speed_deg_s > 0 &&
           speed_deg_s <= speed;
}

bool Yaw_ConfigurationReady(const ArmControl &arm) {
    return kYawSafetyConfirmed && arm.yaw_limits().version != 0;
}

void Yaw_ApplyLimitRequest(ArmControl &arm, ArmOperator &controller, Interlocks &locks,
                           Interlocks &shared_locks, uint32_t now) {
    YawLimitRequest request{};
    taskENTER_CRITICAL();
    request.half_range_deg = g_yaw_limit_request.half_range_deg;
    request.request_seq = g_yaw_limit_request.request_seq;
    taskEXIT_CRITICAL();
    if (request.request_seq == g_yaw_limit_status.consumed_seq)
        return;

    Result applied = Result::InvalidArgument;
    bool invalidate_interlock = false;
    if (kArmConfig.joint_mask == kFourAxisJoints) {
        applied = Result::InvalidArgument;
    } else if (!kYawSafetyConfirmed) {
        applied = Result::ConfigUnconfirmed;
    } else if (std::isfinite(request.half_range_deg) &&
               request.half_range_deg > kYawBrakingMarginDeg &&
               request.half_range_deg <= kYawAbsoluteHalfRangeDeg) {
        const YawRuntimeLimits proposed{-request.half_range_deg,
                                        request.half_range_deg,
                                        kArmConfig.joint[Yaw].max_speed_deg_s,
                                        kYawBrakingAccelDegS2,
                                        kYawBrakingMarginDeg,
                                        std::fmin(kYawMaxTargetLeadDeg, request.half_range_deg),
                                        request.request_seq};
        applied = arm.set_yaw_limits(proposed, now);
        if (applied == Result::Ok) {
            controller.set_yaw_limits(proposed);
            invalidate_interlock = locks.drive_yaw_allowed &&
                                   (locks.drive_yaw_min_deg < proposed.min_deg ||
                                    locks.drive_yaw_max_deg > proposed.max_deg ||
                                    locks.drive_yaw_max_speed_deg_s > proposed.max_speed_deg_s);
            taskENTER_CRITICAL();
            g_yaw_limit_status.active_half_range_deg = request.half_range_deg;
            g_yaw_limit_status.min_deg = proposed.min_deg;
            g_yaw_limit_status.max_deg = proposed.max_deg;
            g_yaw_limit_status.max_speed_deg_s = proposed.max_speed_deg_s;
            g_yaw_limit_status.active_seq = proposed.version;
            g_yaw_limit_status.ready = 1;
            if (invalidate_interlock)
                shared_locks.valid = false;
            taskEXIT_CRITICAL();
            if (invalidate_interlock)
                locks.valid = false;
        }
    }
    // Rejections consume the sequence too; changing conditions cannot cause a
    // delayed application, and changing only the value never resubmits it.
    taskENTER_CRITICAL();
    g_yaw_limit_status.consumed_seq = request.request_seq;
    g_yaw_limit_status.result = uint32_t(applied);
    taskEXIT_CRITICAL();
}

void Yaw_ApplyOperatingEnvelope(ArmControl &arm, const Interlocks &locks, OperatorOutput &output) {
    const auto &limits = arm.yaw_limits();
    const bool driving = output.mode == RobotMode::Drive;
    const bool drive_envelope = driving && locks.drive_yaw_allowed;
    // Selecting Grab/Standby or losing Drive permission must not widen the
    // range while wheels are parking. Keep the current envelope until stopped.
    Result applied = Result::Ok;
    if ((!driving && locks.chassis_stopped) || drive_envelope) {
        applied = arm.set_yaw_envelope(drive_envelope ? locks.drive_yaw_min_deg : limits.min_deg,
                                       drive_envelope ? locks.drive_yaw_max_deg : limits.max_deg,
                                       drive_envelope ? locks.drive_yaw_max_speed_deg_s
                                                      : limits.max_speed_deg_s);
    }
    if (applied != Result::Ok && output.active) {
        output.stop_chassis = true;
        output.longitudinal_brake = true;
        output.reset_chassis_motion = true;
        output.longitudinal_speed_request = 0;
        output.turn_rate_request = 0;
        output.drive_yaw_limited = true;
        output.block = OperatorBlock::DriveYawDenied;
        output.target_deg[Yaw] = arm.status().joint[Yaw].target_deg;
    }
}

uint32_t Yaw_ExecuteBench(ArmControl &arm, const RemoteInput &remote, uint32_t now,
                          uint32_t elapsed, bool stop_requested, const uint32_t &owner_stop_epoch) {
#if YAW_SUSPENDED_BENCH
    static YawBenchGate gate;
    const bool remote_ok =
        remote.valid && uint32_t(now - remote.received_ms) <= kOperatorConfig.remote_timeout_ms;
    bool neutral = remote_ok;
    for (int channel : remote.frame.channel)
        neutral &= std::abs(channel) <= 30;
    neutral &= remote.frame.switches[kOperatorConfig.left_switch_index] == DR16_SWITCH_UP &&
               remote.frame.switches[kOperatorConfig.right_switch_index] == DR16_SWITCH_MID;

    const auto &yaw = arm.status().joint[Yaw];
    const bool yaw_fresh = yaw.received && yaw.calibrated &&
                           uint32_t(now - yaw.last_feedback_ms) <= kArmConfig.dji_unwrap_gap_ms;
    const bool config_ok = kArmConfig.joint_mask == kAllJoints &&
                           arm.status().configuration_valid && kArmConfig.hardware_confirmed &&
                           kOperatorConfig.mapping_confirmed && Yaw_ConfigurationReady(arm);
    const bool timing_ok = elapsed > 0 && elapsed <= kArmConfig.control_period_ms + 2;
    const TaskProtectInput protection{uint8_t(config_ok),
                                      uint8_t(neutral),
                                      1,
                                      uint8_t(stop_requested),
                                      uint8_t(timing_ok),
                                      uint8_t(arm.status().faults != 0),
                                      0};
    Task_Protect_Tick(&protection, now);

    YawBenchRequest request{};
    taskENTER_CRITICAL();
    request.mode = g_yaw_debug_request.mode;
    request.speed_deg_s = g_yaw_debug_request.speed_deg_s;
    request.duration_ms = g_yaw_debug_request.duration_ms;
    request.suspended_confirm = g_yaw_debug_request.suspended_confirm;
    request.request_seq = g_yaw_debug_request.request_seq;
    const uint32_t stop_epoch = owner_stop_epoch;
    taskEXIT_CRITICAL();

    YawBenchHealth health;
    health.ready = config_ok && neutral && timing_ok && yaw_fresh &&
                   yaw.temperature_c < kArmConfig.joint[Yaw].max_temperature_c &&
                   (arm.status().enabled_mask & ~(1u << Yaw)) == 0;
    health.enabled = yaw.enabled;
    health.faulted = arm.status().faults != 0;
    health.stationary =
        std::fabs(yaw.velocity_deg_s) <= kArmConfig.joint[Yaw].stationary_speed_deg_s;
    health.stop = stop_requested;
    health.stop_epoch = stop_epoch;
    auto pulse = gate.step(now, request, health);
    if (pulse.active) {
        const bool enable_failed =
            !yaw.enabled && (!Task_Protect_TryArm() || arm.enable(1u << Yaw, now) != Result::Ok);
        if (enable_failed ||
            arm.set_effort_limit(Yaw, std::fmin(150.0f, kArmConfig.joint[Yaw].max_effort)) !=
                Result::Ok ||
            arm.set_yaw_test_speed(pulse.speed_deg_s, now) != Result::Ok) {
            arm.disable();
            Task_Protect_Trip(PROTECT_ENABLE_REJECTED, now);
            health.stop = true;
            pulse = gate.step(now, request, health);
        }
    } else {
        arm.disable();
    }

    taskENTER_CRITICAL();
    g_yaw_debug_status = {pulse.consumed_seq, pulse.result, pulse.until_ms, pulse.speed_deg_s,
                          uint8_t(pulse.active)};
    taskEXIT_CRITICAL();
    return stop_epoch;
#else
    // Normal vehicle and four-axis builds never consume tuning-output requests.
    (void)arm;
    (void)remote;
    (void)now;
    (void)elapsed;
    (void)stop_requested;
    (void)owner_stop_epoch;
    return 0;
#endif
}
} // namespace engineer
