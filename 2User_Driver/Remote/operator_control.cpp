#include "Remote/operator_control.h"
#include <cmath>

namespace engineer {
namespace {
float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
RobotMode mode_for(uint8_t value) {
    return value == DR16_SWITCH_UP    ? RobotMode::Drive
           : value == DR16_SWITCH_MID ? RobotMode::Grab
                                      : RobotMode::Standby;
}
bool finite(float x) {
    return std::isfinite(x);
}
} // namespace
ArmOperator::ArmOperator(const Config &a, const OperatorConfig &c) : arm_(a), config_(c) {
    const auto &y = a.joint[Yaw];
    yaw_limits_ = {
        y.min_deg, y.max_deg, y.max_speed_deg_s, y.max_accel_deg_s2, 0, y.max_deg - y.min_deg, 0};
    valid_ = c.mapping_confirmed && c.left_switch_index < 2 && c.right_switch_index < 2 &&
             c.left_switch_index != c.right_switch_index && finite(c.deadband) && c.deadband > 0 &&
             c.deadband < 0.5f && c.remote_timeout_ms &&
             (a.joint_mask == kFourAxisJoints || c.interlock_timeout_ms) && c.neutral_ms &&
             finite(c.gripper_open_deg) && finite(c.gripper_closed_deg) &&
             c.gripper_open_deg != c.gripper_closed_deg && finite(c.gripper_hold_current) &&
             c.gripper_hold_current > 0 && c.gripper_hold_current <= a.joint[Gripper].max_effort &&
             c.gripper_close_timeout_ms && finite(c.big_pitch_target_speed_deg_s) &&
             c.big_pitch_target_speed_deg_s > 0;
    const unsigned channels[] = {c.left_y_channel, c.left_x_channel, c.right_y_channel,
                                 c.right_x_channel};
    for (unsigned i = 0; i < 4; ++i) {
        if (channels[i] >= 4 || (c.axis_direction[i] != 1 && c.axis_direction[i] != -1))
            valid_ = false;
        for (unsigned j = 0; j < i; ++j)
            if (channels[i] == channels[j])
                valid_ = false;
    }
    const auto &g = a.joint[Gripper];
    if (c.gripper_open_deg < g.min_deg || c.gripper_open_deg > g.max_deg ||
        c.gripper_closed_deg < g.min_deg || c.gripper_closed_deg > g.max_deg)
        valid_ = false;
}
void ArmOperator::set_yaw_limits(const YawRuntimeLimits &l) {
    yaw_limits_ = l;
    arm_.joint[Yaw].min_deg = l.min_deg;
    arm_.joint[Yaw].max_deg = l.max_deg;
    arm_.joint[Yaw].max_speed_deg_s = l.max_speed_deg_s;
    targets_[Yaw] = clamp(targets_[Yaw], l.min_deg, l.max_deg);
}
void ArmOperator::set_yaw_remote_lead(float deg) {
    if (finite(deg) && deg >= 2.0f && deg <= 180.0f)
        yaw_remote_lead_deg_ = deg;
}
void ArmOperator::freeze(const Status &s) {
    for (unsigned i = 0; i < kJointCount; ++i)
        targets_[i] =
            (arm_.joint_mask & (1u << i))
                ? clamp(s.joint[i].position_deg, arm_.joint[i].min_deg, arm_.joint[i].max_deg)
                : 0;
    close_tracking_ = false;
    close_limited_ = false;
}
void ArmOperator::resume_from_tuning(const Status &s, const RemoteInput &r, uint32_t now) {
    freeze(s);
    mode_ = config_.left_switch_index < 2
                ? mode_for(r.frame.switches[config_.left_switch_index])
                : RobotMode::Safe;
    waiting_ = false;
    active_ = s.enabled_mask == arm_.joint_mask;
    safe_latched_ = false;
    neutral_tracking_ = false;
    neutral_since_ = now;
    last_step_ = now - arm_.control_period_ms;
    have_step_ = true;
    last_right_ = DR16_SWITCH_MID;
    bench_up_seen_ = false;
    bench_arm_pending_ = false;
    permission_blocked_ = false;
    yaw_was_allowed_ = false;
    close_since_ = 0;
    ++motion_epoch_;
}
void ArmOperator::wait_for_neutral(const Status &s) {
    waiting_ = true;
    neutral_tracking_ = false;
    last_right_ = 0;
    ++motion_epoch_;
    yaw_was_allowed_ = false;
    freeze(s);
}
float remote_axis_value(const DR16Frame &f, unsigned ch, int sign, float deadband) {
    if (ch >= 4 || (sign != 1 && sign != -1) || !finite(deadband) || deadband <= 0 ||
        deadband >= .5f)
        return 0;
    const float x = clamp(float(f.channel[ch]) / 660.0f, -1, 1) * sign;
    if (std::fabs(x) <= deadband)
        return 0;
    return (x > 0 ? 1.0f : -1.0f) * (std::fabs(x) - deadband) / (1 - deadband);
}

OperatorOutput ArmOperator::step(const RemoteInput &r, const Interlocks &in, const Status &s,
                                 uint32_t now, bool arm_request, bool stop_request,
                                 bool parked_tuning, float parked_deadband) {
    OperatorOutput o;
    const bool bench = arm_.joint_mask == kFourAxisJoints;
    o.gripper_effort_limit = config_.gripper_hold_current;
    const uint32_t elapsed = have_step_ ? uint32_t(now - last_step_) : arm_.control_period_ms;
    last_step_ = now;
    have_step_ = true;
    const bool timing_ok = elapsed > 0 && elapsed <= arm_.control_period_ms + 2;
    const bool remote_ok = r.valid && uint32_t(now - r.received_ms) <= config_.remote_timeout_ms;
    const bool link_ok =
        bench || (in.valid && uint32_t(now - in.received_ms) <= config_.interlock_timeout_ms);
    auto finish = [&]() {
        o.mode = mode_;
        o.active = active_;
        o.wait_neutral = waiting_;
        o.motion_epoch = motion_epoch_;
        o.yaw_relative_deg = bench ? 0 : s.joint[Yaw].position_deg;
        if (bench) {
            o.stop_chassis = true;
            o.emergency_stop = true;
        }
        for (unsigned i = 0; i < kJointCount; ++i)
            o.target_deg[i] = targets_[i];
        return o;
    };
    auto fail = [&](OperatorBlock reason, RobotMode stopped_mode = RobotMode::Safe) {
        if (!safe_latched_)
            wait_for_neutral(s);
        safe_latched_ = true;
        active_ = false;
        mode_ = stopped_mode;
        bench_up_seen_ = false;
        bench_arm_pending_ = false;
        permission_blocked_ = false;
        o.block = reason;
        return finish(); // 关控/失联保持此前确认的全臂失能策略。
    };
    if (!valid_)
        return fail(OperatorBlock::InvalidConfig);
    if (!remote_ok)
        return fail(OperatorBlock::RemoteOffline);
    if (!link_ok)
        return fail(OperatorBlock::InterlockOffline);
    if (stop_request)
        return fail(OperatorBlock::NeedArm);
    if (!timing_ok)
        return fail(OperatorBlock::Timing);
    for (unsigned i = 0; i < 4; ++i)
        if (r.frame.channel[i] < -700 || r.frame.channel[i] > 700)
            return fail(OperatorBlock::RemoteOffline);
    const uint8_t left = r.frame.switches[config_.left_switch_index];
    const uint8_t right = r.frame.switches[config_.right_switch_index];
    if (left < 1 || left > 3 || right < 1 || right > 3)
        return fail(OperatorBlock::RemoteOffline);
    const auto &yaw = arm_.joint[Yaw];
    if (!bench && in.drive_yaw_allowed &&
        (!finite(in.drive_yaw_min_deg) || !finite(in.drive_yaw_max_deg) ||
         !finite(in.drive_yaw_max_speed_deg_s) || in.drive_yaw_min_deg >= in.drive_yaw_max_deg ||
         in.drive_yaw_min_deg < yaw.min_deg || in.drive_yaw_max_deg > yaw.max_deg ||
         in.drive_yaw_max_speed_deg_s <= 0 || in.drive_yaw_max_speed_deg_s > yaw.max_speed_deg_s))
        return fail(OperatorBlock::InvalidInterlock);
    // The parked arm follows the current switch position.
    if (parked_tuning && left == DR16_SWITCH_DOWN)
        return fail(OperatorBlock::Standby, RobotMode::Standby);

    if (bench && left == DR16_SWITCH_UP) {
        // UP是台架关控位；只有本次健康连接中实际见过UP，MID才能产生开控事件。
        const auto stopped = fail(OperatorBlock::NeedArm);
        bench_up_seen_ = s.configuration_valid && !s.faults;
        return stopped;
    }
    safe_latched_ = false;
    const uint8_t controlled_joints = parked_tuning ? kFourAxisJoints : arm_.joint_mask;
    uint8_t ready_joints = 0;
    for (unsigned i = 0; i < kJointCount; ++i)
        if ((controlled_joints & (1u << i)) && s.joint[i].online && s.joint[i].calibrated &&
            !s.joint[i].faults)
            ready_joints |= 1u << i;
    const bool joints_ready = parked_tuning ? ready_joints != 0 : ready_joints == controlled_joints;
    const bool arm_fault = parked_tuning ? (s.faults & kSharedArmFaults) != 0 : s.faults != 0;
    const bool enabled = parked_tuning
                             ? s.enabled_mask != 0 && !(s.enabled_mask & ~controlled_joints)
                             : s.enabled_mask == controlled_joints;
    if (active_ && (!enabled || arm_fault || !s.configuration_valid || !joints_ready))
        return fail(OperatorBlock::ArmUnavailable);
    if (bench_up_seen_) {
        bench_arm_pending_ = left == DR16_SWITCH_MID;
        bench_up_seen_ = false;
    }
    const RobotMode wanted = mode_for(left);
    if (wanted != mode_) {
        mode_ = wanted;
        wait_for_neutral(s);
        permission_blocked_ = false;
    }
    const float deadband = parked_tuning && parked_deadband > 0 && parked_deadband < .5f
                               ? parked_deadband : config_.deadband;
    const float ly = remote_axis_value(r.frame, config_.left_y_channel,
                                       config_.axis_direction[0], deadband);
    const float lx = remote_axis_value(r.frame, config_.left_x_channel,
                                       config_.axis_direction[1], deadband);
    const float ry = remote_axis_value(r.frame, config_.right_y_channel,
                                       config_.axis_direction[2], deadband);
    const float rx = remote_axis_value(r.frame, config_.right_x_channel,
                                       config_.axis_direction[3], deadband);
    const bool neutral = ly == 0 && lx == 0 && ry == 0 && rx == 0 && right == DR16_SWITCH_MID;
    if (bench && (!neutral || mode_ != RobotMode::Grab || !joints_ready || !s.configuration_valid ||
                  s.faults))
        bench_arm_pending_ = false;
    // 正常阻塞/待机可保持已使能关节；异常路径始终保持默认emergency_stop/disable_arm。
    auto hold = [&](OperatorBlock reason) {
        o.block = reason;
        o.disable_arm = !active_;
        o.emergency_stop = !active_;
        return finish();
    };
    if (mode_ == RobotMode::Standby) {
        // 进入时已抓取一次反馈为保持目标。后续不能跟着下垂反馈不断重抓目标。
        // 即使arm_request为真也不在待机中开控；退出时重新经过停车和回中。
        return hold(OperatorBlock::Standby);
    }
    bool permitted = bench || parked_tuning ||
                     (mode_ == RobotMode::Grab ? in.chassis_stopped : in.arm_allows_drive);
    OperatorBlock denial =
        mode_ == RobotMode::Grab ? OperatorBlock::WaitChassisStop : OperatorBlock::PostureDenied;
    if (mode_ == RobotMode::Drive && in.drive_yaw_allowed &&
        (s.joint[Yaw].position_deg < in.drive_yaw_min_deg ||
         s.joint[Yaw].position_deg > in.drive_yaw_max_deg || targets_[Yaw] < in.drive_yaw_min_deg ||
         targets_[Yaw] > in.drive_yaw_max_deg ||
         s.joint[Yaw].reference_deg < in.drive_yaw_min_deg ||
         s.joint[Yaw].reference_deg > in.drive_yaw_max_deg)) {
        permitted = false;
        denial = OperatorBlock::DriveYawDenied;
    }
    if (!permitted) {
        if (!permission_blocked_)
            wait_for_neutral(s);
        permission_blocked_ = true;
        neutral_tracking_ = false;
        return hold(denial);
    }
    permission_blocked_ = false;
    if (parked_tuning)
        waiting_ = false;
    if (waiting_) {
        // 只在切换/重新许可阶段要求停稳；正常行驶不能永久要求chassis_stopped。
        if (!bench && !in.chassis_stopped) {
            neutral_tracking_ = false;
            return hold(OperatorBlock::WaitChassisStop);
        }
        if (!neutral)
            neutral_tracking_ = false;
        else if (!neutral_tracking_) {
            neutral_tracking_ = true;
            neutral_since_ = now;
        } else if (uint32_t(now - neutral_since_) >= config_.neutral_ms) {
            waiting_ = false;
            last_right_ = DR16_SWITCH_MID;
        }
        if (waiting_)
            return hold(OperatorBlock::WaitNeutral);
    }
    if (!active_) {
        const bool requested = parked_tuning || (neutral && (arm_request || bench_arm_pending_));
        bench_arm_pending_ = false;
        if (!requested || !s.configuration_valid || arm_fault)
            return hold(OperatorBlock::NeedArm);
        if (!joints_ready)
            return hold(OperatorBlock::ArmUnavailable);
        freeze(s);
        active_ = true;
        o.request_enable = true;
    }
    o.disable_arm = false;
    o.emergency_stop = false;
    o.block = OperatorBlock::None;
    const float dt = float(elapsed) * 0.001f;
    auto integrate = [&](unsigned i, float input) {
        const auto &c = arm_.joint[i];
        const float speed = i == BigPitch
                                ? std::fmin(config_.big_pitch_target_speed_deg_s, c.max_speed_deg_s)
                                : c.max_speed_deg_s;
        targets_[i] = clamp(targets_[i] + input * speed * dt, c.min_deg, c.max_deg);
    };
    auto constrain_yaw_target = [&]() {
        // Remote target may lead the actual angle only by the configured amount,
        // but never beyond the active runtime target-lead safety limit. When the
        // limit is reached the stick can no longer increase the target; it waits
        // for feedback. Reversing is allowed because it moves back into the window.
        const float lead = std::fmin(yaw_remote_lead_deg_, yaw_limits_.max_target_lead_deg);
        targets_[Yaw] = clamp(
            targets_[Yaw],
            std::fmax(yaw.min_deg, s.joint[Yaw].position_deg - lead),
            std::fmin(yaw.max_deg, s.joint[Yaw].position_deg + lead));
    };
    if (mode_ == RobotMode::Drive) {
        o.reset_chassis_motion = false;
        o.fast_requested = right == DR16_SWITCH_DOWN;
        o.drive_gear = o.fast_requested && in.arm_allows_fast ? DriveGear::Fast : DriveGear::Slow;
        o.longitudinal_speed_request = ly;
        o.longitudinal_brake = ly == 0; // 转弯时也必须独立将纵向速度制动到0。
        o.turn_rate_request = -rx;      // 右推=顺时针；CCW正，不乘纵向输入或倒车符号。
        o.stop_chassis = ly == 0 && rx == 0;
        if (in.drive_yaw_allowed) {
            targets_[Yaw] = clamp(targets_[Yaw] - lx * in.drive_yaw_max_speed_deg_s * dt,
                                  in.drive_yaw_min_deg, in.drive_yaw_max_deg);
            yaw_was_allowed_ = true;
        } else {
            if (yaw_was_allowed_)
                targets_[Yaw] = clamp(s.joint[Yaw].position_deg, yaw.min_deg, yaw.max_deg);
            yaw_was_allowed_ = false;
            o.drive_yaw_limited = true;
        }
        if (!bench)
            constrain_yaw_target();
        // 其余机械臂轴不接受行驶摇杆新目标。
    } else if (mode_ == RobotMode::Grab) {
        // 两pitch已解耦：左竖只推DM1关节目标，右竖只推DM2关节目标，各按自身范围限位。
        integrate(BigPitch, ly);
        integrate(SmallPitch, ry);
        integrate(Roll, lx);
        if (!bench) {
            integrate(Yaw, -rx); // 遥控Yaw左右反向；不改变电机反馈/输出坐标
            constrain_yaw_target();
        }
        if (right != last_right_) {
            targets_[Gripper] = clamp(s.joint[Gripper].position_deg, arm_.joint[Gripper].min_deg,
                                      arm_.joint[Gripper].max_deg);
            close_tracking_ = false;
            close_limited_ = false;
        }
        const bool manual_gripper = parked_tuning && right == DR16_SWITCH_MID;
        const bool closing = right == DR16_SWITCH_DOWN || (manual_gripper && rx > 0);
        const bool opening = right == DR16_SWITCH_UP || (manual_gripper && rx < 0);
        if (manual_gripper && !closing) {
            close_tracking_ = false;
            close_limited_ = false;
        }
        if (closing) {
            if (!close_tracking_) {
                close_tracking_ = true;
                close_since_ = now;
            }
            if (uint32_t(now - close_since_) >= config_.gripper_close_timeout_ms)
                close_limited_ = true;
        }
        if ((opening || closing) && !close_limited_) {
            const float goal = opening ? config_.gripper_open_deg : config_.gripper_closed_deg;
            const float fraction = manual_gripper ? std::fabs(rx) : 1.0f;
            const float max_step = fraction * arm_.joint[Gripper].max_speed_deg_s * dt;
            targets_[Gripper] += clamp(goal - targets_[Gripper], -max_step, max_step);
            o.gripper_effort_limit = arm_.joint[Gripper].max_effort;
        }
        if (close_limited_) {
            targets_[Gripper] = clamp(s.joint[Gripper].position_deg, arm_.joint[Gripper].min_deg,
                                      arm_.joint[Gripper].max_deg);
            o.gripper_effort_limit = config_.gripper_hold_current;
        }
    }
    last_right_ = right;
    return finish();
}
} // namespace engineer
