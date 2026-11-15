#include "Arm/yaw_config.h"
#include "Arm/arm_control.h"
#include <cmath>
#include <cstring>

namespace engineer {
namespace {
constexpr float kRadToDeg = 57.29577951308232f;
// Below this speed target the Yaw friction feedforward is disabled, so a hold
// command near zero does not buzz against static friction.
constexpr float kYawFrictionDeadbandDegS = 0.5f;
constexpr float kYawPidDerivativeTauS = 0.01f;
// Project4 uses 5 rad/s; keep the reverse-integral behavior while limiting
// windup in the much slower 4 ms loop. Units are the Yaw speed error in deg/s.
constexpr uint32_t kRollZeroStationaryMs = 200;
constexpr float kRollZeroMaxSpanDeg = 0.1f;
constexpr float kMotionFfStartSpeedDegS = 0.5f;
constexpr float kMotionFfStopSpeedDegS = 0.25f;
constexpr float kMotionFfSlewNmS = 2.0f;
float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
bool finite(float x) {
    return std::isfinite(x);
}
bool valid_feedforward(unsigned joint, float bias, float gain, float offset,
                       float up, float down) {
    return finite(bias) && finite(gain) && finite(offset) &&
           finite(up) && up >= 0 && finite(down) && down >= 0 &&
           finite(std::fabs(bias) + std::fabs(gain)) &&
           ((joint == BigPitch || joint == SmallPitch) ||
            (bias == 0 && gain == 0 && offset == 0 && up == 0 && down == 0));
}
bool mask_valid(uint8_t m, uint8_t allowed) {
    return m != 0 && (m & ~allowed) == 0;
}
uint16_t be16(const uint8_t *d) {
    return uint16_t(d[0]) << 8 | d[1];
}
float decode(unsigned x, float max, unsigned bits) {
    return float(x) * (2.0f * max) / float((1u << bits) - 1u) - max;
}
unsigned encode(float x, float max, unsigned bits) {
    return unsigned((clamp(x, -max, max) + max) * float((1u << bits) - 1u) / (2 * max));
}
} // namespace

void pack_dm_torque(float t, float pm, float vm, float tm, uint8_t d[8]) {
    const unsigned p = encode(0, pm, 16), v = encode(0, vm, 12), torque = encode(t, tm, 12);
    d[0] = p >> 8;
    d[1] = p;
    d[2] = v >> 4;
    d[3] = (v & 0xf) << 4; // kp=0
    d[4] = 0;
    d[5] = 0; // kd=0
    d[6] = torque >> 8;
    d[7] = torque;
}

ArmControl::ArmControl(const Config &c)
    : config_(c), startup_effort_cap_{c.joint[0].max_effort, c.joint[1].max_effort,
                                    c.joint[2].max_effort, c.joint[3].max_effort,
                                    c.joint[4].max_effort} {
    status_.configuration_valid = valid_config();
    for (unsigned i = 0; i < kJointCount; ++i) {
        status_.joint[i].fixed_zero_encoder = c.joint[i].fixed_zero_encoder;
        runtime_[i].effort_limit = c.joint[i].max_effort;
        tuning_effort_limit_[i] = c.joint[i].max_effort;
    }
    const auto &y = c.joint[Yaw];
    yaw_limits_ = {
        y.min_deg, y.max_deg, y.max_speed_deg_s, y.max_accel_deg_s2, 0,
        std::fmin(kYawMaxTargetLeadDeg, y.max_deg - y.min_deg), 0};
    yaw_motion_min_ = y.min_deg;
    yaw_motion_max_ = y.max_deg;
    yaw_motion_speed_ = y.max_speed_deg_s;
    yaw_control_dt_s_ = c.control_period_ms ? float(c.control_period_ms) * 0.001f : 0.004f;
    YawPidConfig pos{};
    pos.kp = y.position_kp;
    pos.max_output = y.max_speed_deg_s;
    pos.d_tau = kYawPidDerivativeTauS;
    YawPid_Init(yaw_position_pid_, pos);
    YawPidConfig vel{};
    vel.kp = y.velocity_kp;
    vel.ki = y.velocity_ki;
    vel.max_i_error = kYawVelocityIntegralBandDegS;
    vel.max_sigma = tune_yaw_velocity_max_sigma;
    vel.max_output = y.max_effort;
    vel.d_tau = kYawPidDerivativeTauS;
    YawPid_Init(yaw_velocity_pid_, vel);
}

void ArmControl::reset_yaw_pid() {
    YawPid_Reset(yaw_position_pid_);
    YawPid_Reset(yaw_velocity_pid_);
}

void ArmControl::set_yaw_control_dt(float dt_s) {
    yaw_dt_valid_ = finite(dt_s) && dt_s > 0.0001f && dt_s <= 0.02f;
    if (yaw_dt_valid_)
        yaw_control_dt_s_ = dt_s;
}

YawPidSnapshot ArmControl::yaw_pid_snapshot() const {
    YawPidSnapshot s{};
    s.dt_s = yaw_control_dt_s_;
    s.position_kp = yaw_position_pid_.kp;
    s.position_ki = yaw_position_pid_.ki;
    s.position_kd = yaw_position_pid_.kd;
    s.position_error = yaw_position_pid_.error;
    s.position_max_output = yaw_position_pid_.max_output;
    s.position_max_i_error = yaw_position_pid_.max_i_error;
    s.position_max_sigma = yaw_position_pid_.max_sigma;
    s.position_p = yaw_position_pid_.kp * yaw_position_pid_.error;
    s.position_i = yaw_position_pid_.ki * yaw_position_pid_.sigma_error;
    s.position_d = yaw_position_pid_.kd * yaw_position_pid_.d_lpf;
    s.position_sigma = yaw_position_pid_.sigma_error;
    s.position_output = yaw_position_pid_.output;
    s.velocity_kp = yaw_velocity_pid_.kp;
    s.velocity_ki = yaw_velocity_pid_.ki;
    s.velocity_kd = yaw_velocity_pid_.kd;
    s.velocity_error = yaw_velocity_pid_.error;
    s.velocity_max_output = yaw_velocity_pid_.max_output;
    s.velocity_max_i_error = yaw_velocity_pid_.max_i_error;
    s.velocity_max_sigma = yaw_velocity_pid_.max_sigma;
    s.velocity_p = yaw_velocity_pid_.kp * yaw_velocity_pid_.error;
    s.velocity_i = yaw_velocity_pid_.ki * yaw_velocity_pid_.sigma_error;
    s.velocity_d = yaw_velocity_pid_.kd * yaw_velocity_pid_.d_lpf;
    s.velocity_sigma = yaw_velocity_pid_.sigma_error;
    s.velocity_prelimit = yaw_velocity_pid_.output_prelimit;
    s.velocity_output = yaw_velocity_pid_.output;
    return s;
}

float ArmControl::drive_min(unsigned i) const {
    if (i == Yaw)
        return yaw_tuning_unlimited_ ? -180.0f : yaw_motion_min_;
    return config_.joint[i].min_deg;
}
float ArmControl::drive_max(unsigned i) const {
    if (i == Yaw)
        return yaw_tuning_unlimited_ ? 180.0f : yaw_motion_max_;
    return config_.joint[i].max_deg;
}
float ArmControl::drive_speed_limit(unsigned i) const {
    if (i == Yaw)
        return yaw_tuning_unlimited_ ? yaw_limits_.max_speed_deg_s : yaw_motion_speed_;
    return config_.joint[i].max_speed_deg_s;
}
bool ArmControl::targets_in_range(const float q[kJointCount]) const {
    for (unsigned i = 0; i < kJointCount; ++i)
        if (selected(i))
            if (!finite(q[i]) || (!(i == Yaw && yaw_tuning_unlimited_) &&
                (q[i] < config_.joint[i].min_deg || q[i] > config_.joint[i].max_deg)))
                return false;
    return true;
}
void ArmControl::update_joint_feedback() {
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        s.drive_position_deg = c.direction *
                               (runtime_[i].motor_position_deg - runtime_[i].zero_motor_deg) /
                               c.feedback_per_joint;
        s.position_deg = s.drive_position_deg;
        s.velocity_deg_s = s.drive_velocity_deg_s;
    }
}
void ArmControl::update_drive_references() {
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        auto &s = status_.joint[i];
        s.drive_target_deg = s.target_deg;
        s.drive_reference_deg = s.reference_deg;
        s.drive_reference_velocity_deg_s = runtime_[i].reference_velocity;
    }
}

uint16_t ArmControl::feedback_id(unsigned i) const {
    const auto &c = config_.joint[i];
    return c.motor == Motor::Dm4310Mit
               ? c.feedback_id
               : uint16_t((c.motor == Motor::M2006 ? 0x200 : 0x204) + c.motor_id);
}
uint16_t ArmControl::command_id(unsigned i) const {
    const auto &c = config_.joint[i];
    if (c.motor == Motor::Dm4310Mit)
        return c.motor_id;
    if (c.motor == Motor::M2006)
        return c.motor_id <= 4 ? 0x200 : 0x1ff;
    if (c.motor == Motor::Gm6020Current)
        return c.motor_id <= 4 ? 0x1fe : 0x2fe;
    return c.motor_id <= 4 ? 0x1ff : 0x2ff; // 仅供显式选择的6020电压模式
}
bool ArmControl::valid_config() const {
    if (config_.joint_mask != kAllJoints && config_.joint_mask != kFourAxisJoints)
        return false;
    if (!config_.feedback_timeout_ms || !config_.command_timeout_ms || !config_.dji_unwrap_gap_ms ||
        config_.control_period_ms < 1 || config_.control_period_ms > 10)
        return false;
    if (config_.joint[BigPitch].motor != Motor::Dm4310Mit ||
        config_.joint[SmallPitch].motor != Motor::Dm4310Mit)
        return false;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        const auto &c = config_.joint[i];
        if ((c.motor != Motor::Gm6020Voltage && c.motor != Motor::Gm6020Current &&
             c.motor != Motor::M2006 && c.motor != Motor::Dm4310Mit) ||
            (c.bus != 1 && c.bus != 2) || (c.direction != 1 && c.direction != -1) ||
            !finite(c.feedback_per_joint) || c.feedback_per_joint <= 0 || !finite(c.min_deg) ||
            !finite(c.max_deg) || c.min_deg >= c.max_deg || c.min_deg > 0 || c.max_deg < 0 ||
            !finite(c.max_speed_deg_s) || c.max_speed_deg_s <= 0 || !finite(c.max_accel_deg_s2) ||
            c.max_accel_deg_s2 <= 0 || !finite(c.max_effort) || c.max_effort <= 0 ||
            !finite(c.position_kp) || c.position_kp <= 0 || !finite(c.velocity_kp) ||
            c.velocity_kp < 0 || !finite(c.velocity_ki) || c.velocity_ki < 0 ||
            !finite(c.max_temperature_c) || !c.motor_id || !finite(c.limit_margin_deg) ||
            c.limit_margin_deg < 0 || !finite(c.stationary_speed_deg_s) ||
            c.stationary_speed_deg_s <= 0 || !finite(c.initial_effort_limit) ||
            c.initial_effort_limit > c.max_effort ||
            (c.initial_effort_limit < 0 && c.initial_effort_limit != -1.0f) ||
            !valid_feedforward(i, c.torque_bias_nm, c.gravity_gain_nm, c.gravity_offset_deg,
                               c.up_ff_nm, c.down_ff_nm) ||
            c.up_ff_nm > c.max_effort || c.down_ff_nm > c.max_effort ||
            (c.up_direction != 1 && c.up_direction != -1))
            return false;
        if (c.motor == Motor::Dm4310Mit) {
            // 反馈数据低4位携带电机ID，此版要求ID为1~15且MST_ID各自独立。
            if (c.motor_id > 15 || !c.feedback_id || c.feedback_id > 0x7ff || !finite(c.p_max) ||
                c.p_max <= 0 || !finite(c.v_max) || c.v_max <= 0 || !finite(c.t_max) ||
                c.t_max <= 0 || c.max_effort > c.t_max || c.max_temperature_c <= 0 ||
                c.motor_id == c.feedback_id)
                return false;
        } else if (c.motor_id > (c.motor == Motor::M2006 ? 8 : 7) ||
                   c.max_effort > (c.motor == Motor::M2006           ? 10000
                                   : c.motor == Motor::Gm6020Current ? 3000
                                                                     : 25000))
            return false;
        for (unsigned j = 0; j < i; ++j) {
            if (!selected(j))
                continue;
            if (c.bus != config_.joint[j].bus)
                continue;
            if (feedback_id(i) == feedback_id(j))
                return false;
            const bool dm = c.motor == Motor::Dm4310Mit;
            const bool other_dm = config_.joint[j].motor == Motor::Dm4310Mit;
            // DM指令与其他指令/反馈不可同ID。
            if ((dm || other_dm) && command_id(i) == command_id(j))
                return false;
            if (command_id(i) == feedback_id(j) || command_id(j) == feedback_id(i))
                return false;
        }
    }
    return true;
}

float ArmControl::update_yaw_velocity(uint16_t encoder, uint32_t now, bool reset) {
    if (reset) {
        yaw_velocity_count_ = yaw_velocity_head_ = 0;
        yaw_velocity_ticks_ = 0;
    } else {
        int delta = int(encoder) - int(yaw_velocity_last_encoder_);
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        yaw_velocity_ticks_ += delta;
    }
    yaw_velocity_last_encoder_ = encoder;
    // Keep one sample per reception millisecond, preserving the latest angle.
    if (yaw_velocity_count_ && yaw_velocity_history_[(yaw_velocity_head_ + 63) % 64].ms == now) {
        yaw_velocity_history_[(yaw_velocity_head_ + 63) % 64].ticks = yaw_velocity_ticks_;
    } else {
        yaw_velocity_history_[yaw_velocity_head_] = {now, yaw_velocity_ticks_};
        yaw_velocity_head_ = (yaw_velocity_head_ + 1) % 64;
        if (yaw_velocity_count_ < 64) ++yaw_velocity_count_;
    }
    const VelocitySample *oldest = nullptr;
    for (uint32_t n = 0; n < yaw_velocity_count_; ++n) {
        const auto &sample = yaw_velocity_history_[(yaw_velocity_head_ + 63 - n) % 64];
        oldest = &sample;
        if (uint32_t(now - sample.ms) >= 40) break;
    }
    auto &s = status_.joint[Yaw];
    s.velocity_window_ms = oldest ? uint32_t(now - oldest->ms) : 0;
    if (s.velocity_window_ms < 4) return 0;
    const auto &c = config_.joint[Yaw];
    return float(yaw_velocity_ticks_ - oldest->ticks) * (360.0f / 8192.0f) *
           1000.0f / s.velocity_window_ms * c.direction / c.feedback_per_joint;
}

bool ArmControl::receive(uint8_t bus, uint16_t id, const uint8_t *d, uint8_t len, uint32_t now) {
    if (!d || len != 8 || !status_.configuration_valid)
        return false;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        const auto &c = config_.joint[i];
        if (bus != c.bus || id != feedback_id(i))
            continue;
        auto &r = runtime_[i];
        auto &s = status_.joint[i];
        if (c.motor == Motor::Dm4310Mit) {
            if ((d[0] & 0xf) != c.motor_id)
                return false;
            r.motor_position_deg = decode(be16(d + 1), c.p_max, 16) * kRadToDeg;
            s.drive_velocity_deg_s = c.direction *
                                     decode((unsigned(d[3]) << 4) | (d[4] >> 4), c.v_max, 12) *
                                     kRadToDeg / c.feedback_per_joint;
            s.motor_effort_feedback = decode((unsigned(d[4] & 0xf) << 8) | d[5], c.t_max, 12);
            s.motor_state = d[0] >> 4;
            s.temperature_c = d[6] > d[7] ? d[6] : d[7];
        } else {
            const auto encoder = be16(d);
            if (encoder >= 8192)
                return false;
            const bool continuous = r.have_feedback &&
                                    uint32_t(now - s.last_feedback_ms) <= config_.dji_unwrap_gap_ms;
            s.raw_encoder = encoder;
            const bool fixed_absolute_zero = i == Yaw && c.fixed_zero_encoder >= 0;
            if (fixed_absolute_zero) {
                // The fixed Yaw zero is an absolute encoder reference. A long
                // feedback gap must not invalidate it or reintroduce multi-turn
                // unwrap error; recompute the single-turn delta every frame.
                int delta = int(encoder) - c.fixed_zero_encoder;
                if (delta > 4096) delta -= 8192;
                if (delta < -4096) delta += 8192;
                r.encoder_sum = delta;
                r.zero_motor_deg = 0;
                s.calibrated = true;
            } else if (!r.have_feedback) {
                r.encoder_sum = encoder;
            } else if (uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms) {
                r.encoder_sum = encoder;
                s.calibrated = false;
                if (s.enabled)
                    trip(PositionLost, i);
            } else {
                int delta = int(encoder) - int(r.last_encoder);
                if (delta > 4096)
                    delta -= 8192;
                if (delta < -4096)
                    delta += 8192;
                r.encoder_sum += delta;
            }
            r.last_encoder = encoder;
            if (i == Roll) {
                auto &still = roll_zero_stillness_;
                if (!continuous || !still.tracking)
                    still = {r.encoder_sum, r.encoder_sum, now, true};
                if (r.encoder_sum < still.min_encoder)
                    still.min_encoder = r.encoder_sum;
                if (r.encoder_sum > still.max_encoder)
                    still.max_encoder = r.encoder_sum;
                const float span_deg = float(still.max_encoder - still.min_encoder) *
                                       (360.0f / 8192.0f) / c.feedback_per_joint;
                // Use the unwrapped encoder range so noisy C610 RPM cannot reject a still axis.
                if (span_deg > kRollZeroMaxSpanDeg)
                    still = {r.encoder_sum, r.encoder_sum, now, true};
            }
            r.motor_position_deg = float(r.encoder_sum) * (360.0f / 8192.0f);
            s.can_velocity_deg_s =
                c.direction * float(int16_t(be16(d + 2))) * 6.0f / c.feedback_per_joint;
            if (i == Yaw) {
                s.encoder_velocity_deg_s = update_yaw_velocity(encoder, now,
                    !r.have_feedback || uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms);
                s.drive_velocity_deg_s = c.encoder_velocity ? s.encoder_velocity_deg_s : s.can_velocity_deg_s;
            } else {
                s.drive_velocity_deg_s = s.can_velocity_deg_s;
            }
            s.motor_effort_feedback = int16_t(be16(d + 4));
            // C610手册的DATA[6:7]保留，不当成温度反馈。
            s.temperature_c = c.motor != Motor::M2006 ? float(d[6]) : 0;
        }
        s.last_feedback_ms = now;
        ++s.received;
        r.have_feedback = true;
        s.online = true;
        update_joint_feedback(); // step在输出前统一refresh并检查时差；enable也会重新检查。
        return true;
    }
    return false;
}

void ArmControl::trip(uint32_t fault, unsigned i) {
    status_.faults |= fault;
    status_.joint[i].faults |= fault;
    if (i == BigPitch || i == SmallPitch || fault == TransportError)
        status_.joint[i].calibrated = false;
    if (fault == TransportError) {
        for (unsigned j = 0; j < kJointCount; ++j)
            status_.joint[j].calibrated = false;
    }
    if (parked_tuning_ && i != Yaw && !(fault & kSharedArmFaults)) {
        const Result last_result = status_.last_result;
        disable_joints(1u << i);
        status_.last_result = last_result;
    } else {
        disable();
    }
}
void ArmControl::disable() {
    yaw_position_ff_ = 0;
                        yaw_position_ff_active_ = false;
    yaw_speed_ff_ = 0;
    parked_tuning_ = false;
    status_.enabled_mask = 0;
    for (unsigned i = 0; i < kJointCount; ++i) {
        auto &s = status_.joint[i];
        auto &r = runtime_[i];
        s.enabled = false;
        s.effort = 0;
        s.feedforward_effort = 0;
        s.motion_feedforward_effort = 0;
        s.motion_ff_direction = 0;
        r.integral = 0;
        r.reference_velocity = 0;
        r.dm_was_enabled = false;
        s.drive_reference_velocity_deg_s = 0;
        s.velocity_target_deg_s = 0;
    }
    yaw_test_active_ = false;
    reset_yaw_pid();
}
Result ArmControl::disable_joints(uint8_t mask) {
    if (!mask_valid(mask, config_.joint_mask))
        return status_.last_result = Result::InvalidArgument;
    status_.enabled_mask &= ~mask;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(mask & (1u << i)))
            continue;
        auto &s = status_.joint[i];
        auto &r = runtime_[i];
        s.enabled = false;
        s.effort = 0;
        s.feedforward_effort = 0;
        s.motion_feedforward_effort = 0;
        s.motion_ff_direction = 0;
        s.velocity_target_deg_s = 0;
        s.drive_reference_velocity_deg_s = 0;
        r.integral = 0;
        r.reference_velocity = 0;
        r.dm_was_enabled = false;
    }
    if (mask & (1u << Yaw)) {
        yaw_position_ff_ = 0;
                        yaw_position_ff_active_ = false;
        yaw_speed_ff_ = 0;
        yaw_test_active_ = false;
        yaw_test_speed_ = 0;
        reset_yaw_pid();
    }
    return status_.last_result = Result::Ok;
}
Result ArmControl::hold(uint8_t mask, uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask))
        return status_.last_result = Result::InvalidArgument;
    refresh(now);
    // Disabled axes are deliberately left untouched, including their targets.
    const uint8_t active = mask & status_.enabled_mask;
    // 只检查本次mask内的轴和共享CAN/时序故障；其他轴的局部故障不阻止本轴保持。
    uint32_t relevant_faults = status_.faults & kSharedArmFaults;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (mask & (1u << i))
            relevant_faults |= status_.joint[i].faults;
    if (relevant_faults)
        return status_.last_result = Result::FaultLatched;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(active & (1u << i)))
            continue;
        const auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        if (!s.online)
            return status_.last_result = Result::NotOnline;
        if (!s.calibrated)
            return status_.last_result = Result::NotCalibrated;
        if (!finite(s.position_deg) ||
            (!(i == Yaw && yaw_tuning_unlimited_) && (s.position_deg < c.min_deg || s.position_deg > c.max_deg)) ||
            !finite(s.drive_position_deg) || s.drive_position_deg < drive_min(i) ||
            s.drive_position_deg > drive_max(i))
            return status_.last_result = Result::OutOfRange;
    }
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(active & (1u << i)))
            continue;
        auto &s = status_.joint[i];
        auto &r = runtime_[i];
        s.target_deg = s.reference_deg = s.position_deg;
        s.effort = 0;
        s.feedforward_effort = 0;
        s.motion_feedforward_effort = 0;
        s.motion_ff_direction = 0;
        s.velocity_target_deg_s = 0;
        r.integral = r.reference_velocity = 0;
        r.last_command_ms = now;
    }
    if (active & (1u << Yaw)) {
        yaw_test_active_ = false;
        yaw_test_speed_ = 0;
    }
    update_drive_references();
    return status_.last_result = Result::Ok;
}
Result ArmControl::set_yaw_limits(const YawRuntimeLimits &l, uint32_t now) {
    if (!selected(Yaw))
        return status_.last_result = Result::InvalidArgument;
    if (status_.enabled_mask)
        return status_.last_result = Result::Busy;
    const auto &c = config_.joint[Yaw];
    if (!finite(l.min_deg) || !finite(l.max_deg) || l.min_deg >= l.max_deg ||
        l.min_deg < c.min_deg || l.max_deg > c.max_deg || !finite(l.max_speed_deg_s) ||
        l.max_speed_deg_s <= 0 || l.max_speed_deg_s > c.max_speed_deg_s ||
        !finite(l.braking_accel_deg_s2) || l.braking_accel_deg_s2 <= 0 ||
        l.braking_accel_deg_s2 > c.max_accel_deg_s2 || !finite(l.braking_margin_deg) ||
        l.braking_margin_deg < 0 || 2 * l.braking_margin_deg >= l.max_deg - l.min_deg ||
        !finite(l.max_target_lead_deg) || l.max_target_lead_deg <= 0 ||
        l.max_target_lead_deg > l.max_deg - l.min_deg || !l.version)
        return status_.last_result = Result::InvalidArgument;
    refresh(now);
    const auto &s = status_.joint[Yaw];
    if (!s.online)
        return status_.last_result = Result::NotOnline;
    if (!s.calibrated)
        return status_.last_result = Result::NotCalibrated;
    if (std::fabs(s.velocity_deg_s) > c.stationary_speed_deg_s)
        return status_.last_result = Result::Moving;
    const float lo = l.min_deg + l.braking_margin_deg, hi = l.max_deg - l.braking_margin_deg;
    if (s.position_deg < lo || s.position_deg > hi || s.target_deg < lo || s.target_deg > hi ||
        s.reference_deg < lo || s.reference_deg > hi)
        return status_.last_result = Result::OutOfRange;
    yaw_limits_ = l;
    yaw_motion_min_ = l.min_deg;
    yaw_motion_max_ = l.max_deg;
    yaw_motion_speed_ = l.max_speed_deg_s;
    runtime_[Yaw].integral = 0;
    return status_.last_result = Result::Ok;
}
Result ArmControl::set_yaw_envelope(float lo, float hi, float speed) {
    // Modes 5/6 always run without the confirmed wire range; the normal
    // control path restores it when it leaves those modes.
    const float allowed_lo = yaw_tuning_unlimited_ ? -180.0f : yaw_limits_.min_deg;
    const float allowed_hi = yaw_tuning_unlimited_ ? 180.0f : yaw_limits_.max_deg;
    if (!selected(Yaw) || !finite(lo) || !finite(hi) || !finite(speed) || lo >= hi ||
        lo < allowed_lo || hi > allowed_hi || speed <= 0 ||
        speed > yaw_limits_.max_speed_deg_s)
        return status_.last_result = Result::InvalidArgument;
    const auto &s = status_.joint[Yaw];
    if (s.position_deg < lo || s.position_deg > hi || s.target_deg < lo || s.target_deg > hi ||
        s.reference_deg < lo || s.reference_deg > hi)
        return status_.last_result = Result::OutOfRange;
    yaw_motion_min_ = lo;
    yaw_motion_max_ = hi;
    yaw_motion_speed_ = speed;
    return status_.last_result = Result::Ok;
}
Result ArmControl::set_yaw_test_speed(float speed, uint32_t now) {
    if (!selected(Yaw) || !(status_.enabled_mask & (1u << Yaw)) || !finite(speed) ||
        std::fabs(speed) > 100.0f)
        return status_.last_result = Result::InvalidArgument;
    // A run of the old sign must not keep pushing after the operator reverses
    // or zeroes the speed target. Drop the velocity-PI history once on that
    // transition; the feedforward itself follows the new target sign.
    const bool reversing = (speed > 0 && yaw_test_speed_ < 0) ||
                           (speed < 0 && yaw_test_speed_ > 0);
    const bool stopping = speed == 0 && yaw_test_speed_ != 0;
    if (reversing || stopping)
        yaw_velocity_pid_.sigma_error = 0; // Actual project4 PID integral, not legacy PI state.
    // Entering/leaving the speed test is a control-mode change: start the new
    // Yaw PID pair from a clean state. Target changes inside a mode do not reset.
    if (!yaw_test_active_)
        reset_yaw_pid();
    // Speed-loop tuning explicitly takes ownership instead of startup homing.
    status_.joint[Yaw].homing = false;
    yaw_test_active_ = true;
    yaw_test_speed_ = speed;
    runtime_[Yaw].last_command_ms = now;
    status_.joint[Yaw].target_deg = status_.joint[Yaw].position_deg;
    status_.joint[Yaw].reference_deg = status_.joint[Yaw].position_deg;
    runtime_[Yaw].reference_velocity = 0;
    return status_.last_result = Result::Ok;
}
void ArmControl::clear_yaw_test() {
    if (!yaw_test_active_)
        return;
    yaw_test_active_ = false;
    yaw_test_speed_ = 0;
    yaw_position_ff_ = 0;
                        yaw_position_ff_active_ = false;
    yaw_speed_ff_ = 0;
    reset_yaw_pid();
    auto &s = status_.joint[Yaw];
    auto &r = runtime_[Yaw];
    s.target_deg = s.reference_deg = s.position_deg;
    s.effort = 0;
    s.velocity_target_deg_s = 0;
    r.integral = r.reference_velocity = 0;
    update_drive_references();
}
void ArmControl::set_yaw_friction_feedforward(float raw) {
    if (finite(raw) && raw >= 0 && raw <= startup_effort_cap_[Yaw])
        yaw_friction_ff_raw_ = raw;
}
float ArmControl::limit_yaw_speed(float requested, float dt) const {
    if (yaw_tuning_unlimited_) return requested;
    const auto &s = status_.joint[Yaw];
    const float a = yaw_limits_.braking_accel_deg_s2;
    const float margin = yaw_limits_.braking_margin_deg + std::fabs(s.velocity_deg_s) * dt;
    const float left = std::fmax(0.0f, s.position_deg - yaw_motion_min_ - margin);
    const float right = std::fmax(0.0f, yaw_motion_max_ - s.position_deg - margin);
    const float speed_cap = yaw_test_active_ ? 100.0f : yaw_motion_speed_;
    const float lo = -std::fmin(speed_cap, std::sqrt(2 * a * left));
    const float hi = std::fmin(speed_cap, std::sqrt(2 * a * right));
    return clamp(requested, lo, hi);
}
void ArmControl::transport_fault() {
    trip(TransportError, selected(Yaw) ? Yaw : BigPitch);
}
Result ArmControl::set_effort_limit(unsigned i, float limit) {
    if (i >= kJointCount || !selected(i) || !finite(limit) || limit < 0 ||
        limit > config_.joint[i].max_effort)
        return status_.last_result = Result::InvalidArgument;
    runtime_[i].effort_limit = limit;
    runtime_[i].integral = clamp(runtime_[i].integral, -limit, limit);
    return status_.last_result = Result::Ok;
}
bool ArmControl::get_tuning(unsigned i, JointTuning &tuning) const {
    if (i >= kJointCount || !selected(i))
        return false;
    const auto &c = config_.joint[i];
    tuning = {c.position_kp, c.velocity_kp, c.velocity_ki, tuning_effort_limit_[i],
              c.torque_bias_nm, c.gravity_gain_nm, c.gravity_offset_deg,
              c.up_ff_nm, c.down_ff_nm};
    return true;
}
Result ArmControl::set_tuning(unsigned i, const JointTuning &tuning, uint32_t now) {
    if (i >= kJointCount || !selected(i))
        return status_.last_result = Result::InvalidArgument;
    if (status_.enabled_mask)
        return status_.last_result = Result::Busy;
    return set_tuning_checked(i, tuning, now, false);
}
Result ArmControl::set_tuning_runtime(unsigned i, const JointTuning &tuning, uint32_t now) {
    if (i >= kJointCount || !selected(i))
        return status_.last_result = Result::InvalidArgument;
    return set_tuning_checked(i, tuning, now, true);
}
Result ArmControl::set_tuning_checked(unsigned i, const JointTuning &tuning, uint32_t now,
                                      bool runtime) {
    // Numeric typo bounds, not a claim that the full range is safe on hardware.
    if (!finite(tuning.position_kp) || tuning.position_kp <= 0 || tuning.position_kp > 1000 ||
        !finite(tuning.velocity_kp) || tuning.velocity_kp < 0 || tuning.velocity_kp > 1.0e6f ||
        !finite(tuning.velocity_ki) || tuning.velocity_ki < 0 || tuning.velocity_ki > 1.0e6f ||
        !finite(tuning.effort_limit) || tuning.effort_limit < 0 ||
        tuning.effort_limit > startup_effort_cap_[i] ||
        tuning.up_ff_nm > startup_effort_cap_[i] ||
        tuning.down_ff_nm > startup_effort_cap_[i] ||
        !valid_feedforward(i, tuning.torque_bias_nm, tuning.gravity_gain_nm,
                           tuning.gravity_offset_deg, tuning.up_ff_nm, tuning.down_ff_nm))
        return status_.last_result = Result::InvalidArgument;
    if (!status_.configuration_valid)
        return status_.last_result = Result::ConfigUnconfirmed;
    const bool parked_runtime = runtime && parked_tuning_ && i != Yaw;
    const auto &previous = config_.joint[i];
    const bool open_loop_adjustment =
        parked_runtime && (i == BigPitch || i == SmallPitch) &&
        previous.velocity_kp == 0 && previous.velocity_ki == 0 &&
        tuning.velocity_kp == 0 && tuning.velocity_ki == 0;
    refresh(now);
    if (status_.faults & ((parked_runtime || i == Yaw) ? kSharedArmFaults : ~uint32_t{0}))
        return status_.last_result = Result::FaultLatched;
    const uint8_t affected = uint8_t(1u << i);
    for (unsigned j = 0; j < kJointCount; ++j) {
        if (!(affected & (1u << j)))
            continue;
        const auto &s = status_.joint[j];
        const auto &c = config_.joint[j];
        if (!s.online)
            return status_.last_result = Result::NotOnline;
        if (!s.calibrated)
            return status_.last_result = Result::NotCalibrated;
        const bool live_yaw = runtime && i == Yaw;
        if (!finite(s.drive_velocity_deg_s) ||
            (!parked_runtime && !live_yaw && !open_loop_adjustment &&
             std::fabs(s.drive_velocity_deg_s) > c.stationary_speed_deg_s))
            return status_.last_result = Result::Moving;
        // Parked arm tuning may change gains while the joint is moving; the
        // shared effort cap, soft limits and per-axis health checks still apply.
        if (runtime && !open_loop_adjustment && !parked_runtime && !live_yaw &&
            (!finite(s.drive_target_deg) || !finite(s.drive_reference_deg) ||
                        std::fabs(s.drive_target_deg - s.drive_position_deg) > .5f ||
                        std::fabs(s.drive_reference_deg - s.drive_position_deg) > .5f ||
                        std::fabs(runtime_[j].reference_velocity) > .00001f ||
                        (j == Yaw && yaw_test_active_ && yaw_test_speed_ != 0)))
            return status_.last_result = Result::Busy;
        if (s.faults || (c.motor != Motor::M2006 && s.temperature_c >= c.max_temperature_c) ||
            (c.motor == Motor::Dm4310Mit && s.motor_state >= 8))
            return status_.last_result = Result::FaultLatched;
        if (parked_runtime && (!s.enabled || (c.motor == Motor::Dm4310Mit && s.motor_state != 1)))
            return status_.last_result = Result::Busy;
    }
    // Commit only after every field and both pitch drives have passed validation.
    auto &c = config_.joint[i];
    c.position_kp = tuning.position_kp;
    c.velocity_kp = tuning.velocity_kp;
    c.velocity_ki = tuning.velocity_ki;
    c.torque_bias_nm = tuning.torque_bias_nm;
    c.gravity_gain_nm = tuning.gravity_gain_nm;
    c.gravity_offset_deg = tuning.gravity_offset_deg;
    c.up_ff_nm = tuning.up_ff_nm;
    c.down_ff_nm = tuning.down_ff_nm;
    tuning_effort_limit_[i] = tuning.effort_limit;
    if (parked_runtime) {
        // Ki=0 must remove the accumulated effort for open-loop calibration.
        auto &r = runtime_[i];
        const float limit = std::fmin(r.effort_limit, tuning.effort_limit);
        r.integral = tuning.velocity_ki == 0 ? 0 : clamp(r.integral, -limit, limit);
        return status_.last_result = Result::Ok;
    }
    for (unsigned j = 0; j < kJointCount; ++j) {
        if (!(affected & (1u << j)))
            continue;
        if (runtime && i == Yaw) {
            // Live gain/limit edits must not reset the active motion trajectory.
            runtime_[j].integral = clamp(runtime_[j].integral,
                                         -tuning.effort_limit, tuning.effort_limit);
        } else {
            runtime_[j].integral = 0;
            runtime_[j].reference_velocity = 0;
            status_.joint[j].drive_reference_velocity_deg_s = 0;
            status_.joint[j].velocity_target_deg_s = 0;
        }
    }
    return status_.last_result = Result::Ok;
}
Result ArmControl::clear_faults() {
    if (status_.enabled_mask)
        return status_.last_result = Result::Busy;
    status_.faults = 0;
    for (auto &s : status_.joint)
        s.faults = 0;
    return status_.last_result = Result::Ok;
}
void ArmControl::refresh(uint32_t now) {
    update_joint_feedback();
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        s.online = runtime_[i].have_feedback &&
                   uint32_t(now - s.last_feedback_ms) <= config_.feedback_timeout_ms;
        if (!s.online) {
            s.calibrated = false;
            if (s.enabled)
                trip(FeedbackTimeout, i);
        }
        if (runtime_[i].have_feedback && c.motor != Motor::Dm4310Mit &&
            !(i == Yaw && c.fixed_zero_encoder >= 0) &&
            uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms) {
            s.calibrated = false;
            if (i == Roll)
                roll_zero_stillness_.tracking = false;
            if (s.enabled)
                trip(PositionLost, i);
        }
    }
    for (unsigned i = 0; i < kJointCount; ++i) {
        auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        if (!s.enabled)
            continue;
        if (uint32_t(now - runtime_[i].last_command_ms) > config_.command_timeout_ms)
            trip(CommandTimeout, i);
        else if (c.motor != Motor::M2006 && s.temperature_c >= c.max_temperature_c)
            trip(OverTemperature, i);
        else if (c.motor == Motor::Dm4310Mit && s.motor_state >= 8)
            trip(MotorError, i);
        else if (!(i == Yaw && yaw_tuning_unlimited_) &&
                 (s.position_deg < c.min_deg - c.limit_margin_deg ||
                  s.position_deg > c.max_deg + c.limit_margin_deg))
            trip(PositionLimit, i);
        else if (s.drive_position_deg < drive_min(i) - c.limit_margin_deg ||
                 s.drive_position_deg > drive_max(i) + c.limit_margin_deg)
            trip(PositionLimit, i);
    }
}
Result ArmControl::capture_zero(uint8_t mask, uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask))
        return status_.last_result = Result::InvalidArgument;
    if (status_.enabled_mask)
        return status_.last_result = Result::Busy;
    refresh(now);
    for (unsigned i = 0; i < kJointCount; ++i)
        if (mask & (1u << i)) {
            if (!status_.joint[i].online)
                return status_.last_result = Result::NotOnline;
            if (i == Roll) {
                if (!roll_zero_stillness_.tracking ||
                    uint32_t(now - roll_zero_stillness_.since_ms) < kRollZeroStationaryMs)
                    return status_.last_result = Result::Moving;
            } else if (std::fabs(status_.joint[i].drive_velocity_deg_s) >
                       config_.joint[i].stationary_speed_deg_s) {
                return status_.last_result = Result::Moving;
            }
        }
    for (unsigned i = 0; i < kJointCount; ++i)
        if (mask & (1u << i)) {
            // Fixed yaw zero must not be replaced by a RAM capture command.
            if (i == Yaw && config_.joint[i].fixed_zero_encoder >= 0)
                continue;
            runtime_[i].zero_motor_deg = runtime_[i].motor_position_deg;
            auto &s = status_.joint[i];
            s.calibrated = true;
            s.position_deg = 0;
            s.target_deg = 0;
            s.reference_deg = 0;
        }
    update_joint_feedback();
    update_drive_references();
    return status_.last_result = Result::Ok;
}
Result ArmControl::capture_parked_zero(uint32_t now) {
    Result first_error = Result::Ok;
    for (const uint8_t group : kParkedArmGroups) {
        const Result result = capture_zero(group, now);
        if (first_error == Result::Ok && result != Result::Ok)
            first_error = result;
    }
    return status_.last_result = first_error;
}

Result ArmControl::enable(uint8_t mask, uint32_t now, bool independent_yaw) {
    if (parked_tuning_)
        return status_.last_result = Result::Busy;
    return enable_checked(mask, now, !(independent_yaw && mask == (1u << Yaw)));
}
Result ArmControl::enable_parked_tuning(uint32_t now) {
    if (status_.enabled_mask)
        return status_.last_result = Result::Busy;
    parked_tuning_ = true;
    Result first_error = Result::Ok;
    for (const uint8_t group : kParkedArmGroups) {
        const Result result = enable_checked(group, now, false);
        if (first_error == Result::Ok && result != Result::Ok)
            first_error = result;
    }
    if (status_.enabled_mask)
        return status_.last_result = Result::Ok;
    parked_tuning_ = false;
    return status_.last_result = first_error;
}
Result ArmControl::enable_checked(uint8_t mask, uint32_t now, bool require_confirmation) {
    if (!mask_valid(mask, config_.joint_mask))
        return status_.last_result = Result::InvalidArgument;
    const uint8_t added = mask & ~status_.enabled_mask;
    if (!added)
        return status_.last_result = Result::Busy;
    if (!status_.configuration_valid || (require_confirmation && !config_.hardware_confirmed))
        return status_.last_result = Result::ConfigUnconfirmed;
    refresh(now);
    // 只检查本次要新增的轴和共享CAN/时序故障；其他轴的局部故障不阻止本轴使能。
    uint32_t relevant_faults = status_.faults & kSharedArmFaults;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (added & (1u << i))
            relevant_faults |= status_.joint[i].faults;
    if (relevant_faults)
        return status_.last_result = Result::FaultLatched;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (added & (1u << i)) {
            const auto &s = status_.joint[i];
            const auto &c = config_.joint[i];
            if (s.faults)
                return status_.last_result = Result::FaultLatched;
            if (!s.online)
                return status_.last_result = Result::NotOnline;
            if (!s.calibrated)
                return status_.last_result = Result::NotCalibrated;
            if (!(i == Yaw && yaw_tuning_unlimited_) &&
                (s.position_deg < c.min_deg || s.position_deg > c.max_deg))
                return status_.last_result = Result::OutOfRange;
            if (s.drive_position_deg < drive_min(i) || s.drive_position_deg > drive_max(i))
                return status_.last_result = Result::OutOfRange;
            if ((c.motor != Motor::M2006 && s.temperature_c >= c.max_temperature_c) ||
                (c.motor == Motor::Dm4310Mit && s.motor_state >= 8))
                return status_.last_result = Result::FaultLatched;
        }
    status_.enabled_mask |= added;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (added & (1u << i)) {
            auto &s = status_.joint[i];
            auto &r = runtime_[i];
            s.enabled = true;
            s.homing = i == Yaw && config_.joint[i].fixed_zero_encoder >= 0;
            s.target_deg = s.homing ? 0.0f : s.position_deg;
            s.reference_deg = s.position_deg;
            s.motion_feedforward_effort = 0;
            s.motion_ff_direction = 0;
            r.integral = 0;
            r.reference_velocity = 0;
            r.last_command_ms = now;
            r.enable_ms = now;
        }
    if (added & (1u << Yaw))
        reset_yaw_pid();
    update_drive_references();
    return status_.last_result = Result::Ok;
}
Result ArmControl::set_targets(uint8_t mask, const float q[kJointCount], uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask) || !q)
        return status_.last_result = Result::InvalidArgument;
    float next[kJointCount];
    for (unsigned i = 0; i < kJointCount; ++i)
        next[i] = status_.joint[i].target_deg;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (mask & (1u << i)) {
            const auto &c = config_.joint[i];
            if (!finite(q[i]))
                return status_.last_result = Result::InvalidArgument;
            if (!(i == Yaw && yaw_tuning_unlimited_) &&
                (q[i] < c.min_deg || q[i] > c.max_deg))
                return status_.last_result = Result::OutOfRange;
            auto &s = status_.joint[i];
            if (s.homing && std::fabs(s.position_deg) <= 5.0f &&
                std::fabs(s.velocity_deg_s) <= c.stationary_speed_deg_s)
                s.homing = false;
            next[i] = s.homing ? 0.0f : q[i];
        }
    // 两pitch已解耦，整包只校验各轴自身范围，不再组合校验a2=q肩+q肘。
    if (!targets_in_range(next))
        return status_.last_result = Result::OutOfRange;
    for (unsigned i = 0; i < kJointCount; ++i)
        if (mask & (1u << i)) {
            if (i == Yaw && !yaw_test_active_ && next[i] != status_.joint[i].target_deg) {
                const float new_error = next[i] - status_.joint[i].position_deg;
                // Explicit destination update: discard I only when it fights the
                // new destination. Also works when the old destination was reached.
                // Unchanged targets / natural overshoot never repeatedly clear I.
                if (std::fabs(new_error) > kYawPositionFfStartErrorDeg &&
                    new_error * yaw_velocity_pid_.sigma_error < 0)
                    yaw_velocity_pid_.sigma_error = 0;
            }
            status_.joint[i].target_deg =
                i == Yaw
                    ? (yaw_tuning_unlimited_
                           ? clamp(next[i], -180.0f, 180.0f)
                           : clamp(next[i], yaw_motion_min_, yaw_motion_max_))
                    : q[i];
            runtime_[i].last_command_ms = now;
        }
    update_drive_references();
    return status_.last_result = Result::Ok;
}

unsigned ArmControl::step(uint32_t now, Frame out[kJointCount]) {
    yaw_friction_ff_applied_ = 0;
    if (!status_.configuration_valid || !out)
        return 0;
    const uint32_t elapsed = have_step_ ? uint32_t(now - last_step_ms_) : config_.control_period_ms;
    if (have_step_ && (elapsed == 0 || elapsed > config_.control_period_ms + 2) &&
        status_.enabled_mask)
        trip(TimingError, selected(Yaw) ? Yaw : BigPitch);
    last_step_ms_ = now;
    have_step_ = true;
    const float dt = float(elapsed ? elapsed : 1) * 0.001f;
    const float yaw_dt = yaw_control_dt_s_ > 0 ? yaw_control_dt_s_ : dt;
    if (!yaw_dt_valid_ && status_.joint[Yaw].enabled)
        trip(TimingError, Yaw);
    refresh(now);
    // 在生成输出前检查两台 DM 的使能应答。
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &c = config_.joint[i];
        auto &r = runtime_[i];
        auto &s = status_.joint[i];
        if (c.motor != Motor::Dm4310Mit || !s.enabled)
            continue;
        if (s.motor_state == 1)
            r.dm_was_enabled = true;
        else if (r.dm_was_enabled)
            trip(MotorError, i);
        else if (uint32_t(now - r.enable_ms) > 200)
            trip(EnableTimeout, i);
    }
    // 先完成各轴参考规划与边界检查，再计算这一轮出力；两pitch各自独立规划。
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &c = config_.joint[i];
        auto &r = runtime_[i];
        auto &s = status_.joint[i];
        if (s.enabled && (c.motor != Motor::Dm4310Mit || s.motor_state == 1)) {
            if (i == Yaw) {
                // Yaw cascade: position P uses the actual angle directly.
                // No independent reference planner or per-cycle integral reset.
                s.reference_deg = yaw_test_active_ ? s.position_deg : s.target_deg;
                r.reference_velocity = 0;
                continue;
            }
            const float error = s.target_deg - s.reference_deg;
            // 到目标前按制动距离减速；每周期约束参考速度和加速度。
            const float braking = std::sqrt(2 * c.max_accel_deg_s2 * std::fabs(error));
            const float max_speed = drive_speed_limit(i);
            float desired = clamp(error / dt, -max_speed, max_speed);
            desired = clamp(desired, -braking, braking);
            if (i == Yaw)
                desired = limit_yaw_speed(desired, dt);
            r.reference_velocity += clamp(desired - r.reference_velocity, -c.max_accel_deg_s2 * dt,
                                          c.max_accel_deg_s2 * dt);
            if (i == Yaw)
                r.reference_velocity = limit_yaw_speed(r.reference_velocity, dt);
            s.reference_deg =
                clamp(s.reference_deg + r.reference_velocity * dt, drive_min(i), drive_max(i));

        }
    }
    update_drive_references();
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        if (s.enabled &&
            (!finite(s.reference_deg) || !finite(s.drive_reference_deg) ||
             (!(i == Yaw && yaw_tuning_unlimited_) &&
              (s.reference_deg < c.min_deg - 0.0001f || s.reference_deg > c.max_deg + 0.0001f)) ||
             s.drive_reference_deg < drive_min(i) - 0.0001f ||
             s.drive_reference_deg > drive_max(i) + 0.0001f))
            trip(PositionLimit, i);
    }
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i))
            continue;
        const auto &c = config_.joint[i];
        auto &r = runtime_[i];
        auto &s = status_.joint[i];
        const bool ready = (c.motor != Motor::Dm4310Mit || s.motor_state == 1);
        if (s.enabled && ready) {
            const float limit = std::fmin(r.effort_limit, tuning_effort_limit_[i]);
            if (i == Yaw) {
                float speed_target;
                if (yaw_test_active_) {
                    speed_target = limit_yaw_speed(yaw_test_speed_, yaw_dt);
                } else {
                    // Position PID feeds the same speed PID used by mode 5.
                    yaw_position_pid_.kp = c.position_kp;
                    yaw_position_pid_.ki = tune_yaw_position_ki;
                    yaw_position_pid_.kd = tune_yaw_position_kd;
                    yaw_position_pid_.max_i_error = tune_yaw_position_max_i_error;
                    yaw_position_pid_.max_sigma = tune_yaw_position_max_sigma;
                    yaw_position_pid_.max_output = tune_yaw_position_max_output;
                    yaw_position_pid_.d_tau = tune_yaw_position_d_tau;
                    const float position_command = YawPid_Calculate(
                        yaw_position_pid_, s.target_deg, s.drive_position_deg, yaw_dt);
                    speed_target = limit_yaw_speed(position_command, yaw_dt);
                }
                s.velocity_target_deg_s = speed_target;

                float feedforward = 0;
                if (c.motor == Motor::Gm6020Voltage) {
                    if (yaw_test_active_) {
                        yaw_position_ff_ = 0;
                        yaw_position_ff_active_ = false;
                        const float requested_ff = std::fabs(speed_target) > kYawFrictionDeadbandDegS
                            ? (speed_target > 0 ? yaw_friction_ff_raw_ : -yaw_friction_ff_raw_)
                            : 0;
                        const float max_delta = yaw_friction_ff_raw_ * yaw_dt / kYawSpeedFfRiseTimeS;
                        if (requested_ff == 0)
                            yaw_speed_ff_ = 0;
                        else
                            yaw_speed_ff_ += clamp(requested_ff - yaw_speed_ff_, -max_delta, max_delta);
                        feedforward = yaw_speed_ff_;
                    } else {
                        yaw_speed_ff_ = 0;
                        const float position_error = s.target_deg - s.drive_position_deg;
                        const float abs_error = std::fabs(position_error);
                        if (abs_error <= kYawPositionFfStopErrorDeg || speed_target == 0)
                            yaw_position_ff_active_ = false;
                        else if (abs_error >= kYawPositionFfStartErrorDeg)
                            yaw_position_ff_active_ = true;
                        // Never assist a command pointing away from the actual destination.
                        const bool toward_target = speed_target * position_error > 0;
                        const float requested_ff = yaw_position_ff_active_ && toward_target
                            ? (speed_target > 0 ? yaw_friction_ff_raw_ : -yaw_friction_ff_raw_) : 0;
                        const float max_delta = yaw_friction_ff_raw_ * yaw_dt / kYawPositionFfRiseTimeS;
                        if (yaw_friction_ff_raw_ == 0) {
                            yaw_position_ff_ = 0;
                            yaw_position_ff_active_ = false;
                        } else
                            yaw_position_ff_ += clamp(requested_ff - yaw_position_ff_,
                                                      -max_delta, max_delta);
                        feedforward = yaw_position_ff_;
                    }
                    yaw_friction_ff_applied_ = feedforward;
                }

                yaw_velocity_pid_.kp = c.velocity_kp;
                yaw_velocity_pid_.ki = c.velocity_ki;
                yaw_velocity_pid_.kd = tune_yaw_velocity_kd;
                yaw_velocity_pid_.max_i_error = tune_yaw_velocity_max_i_error;
                yaw_velocity_pid_.max_sigma = tune_yaw_velocity_max_sigma;
                yaw_velocity_pid_.max_output = limit;
                yaw_velocity_pid_.d_tau = tune_yaw_velocity_d_tau;
                const float pid_output = YawPid_Calculate(
                    yaw_velocity_pid_, speed_target, s.drive_velocity_deg_s, yaw_dt);
                // project4: PID limits itself, then add friction FF and limit total.
                float final = clamp(pid_output + feedforward, -limit, limit);
                if ((s.drive_position_deg <= drive_min(i) && final < 0) ||
                    (s.drive_position_deg >= drive_max(i) && final > 0))
                    final = 0;
                s.effort = final;
            } else {
                const float speed_limit = drive_speed_limit(i);
                float speed_target =
                    clamp(r.reference_velocity +
                              c.position_kp * (s.drive_reference_deg - s.drive_position_deg),
                          -speed_limit, speed_limit);
                s.velocity_target_deg_s = speed_target;
                // 每轴使用自身驱动速度；解耦后不再有q肘=a2-a1，也不再把相对速度送入第二DM。
                const float e = speed_target - s.drive_velocity_deg_s;
                const float limit = std::fmin(r.effort_limit, tuning_effort_limit_[i]);
                const float next_i = clamp(r.integral + c.velocity_ki * e * dt, -limit, limit);
                s.feedforward_effort = 0;
                if (i == BigPitch || i == SmallPitch) {
                    s.feedforward_effort =
                        c.torque_bias_nm + c.gravity_gain_nm *
                            std::cos((s.drive_position_deg + c.gravity_offset_deg) / kRadToDeg);
                    const float up_speed = c.up_direction * speed_target;
                    if (up_speed >= kMotionFfStartSpeedDegS)
                        s.motion_ff_direction = 1;
                    else if (up_speed <= -kMotionFfStartSpeedDegS)
                        s.motion_ff_direction = -1;
                    else if (s.motion_ff_direction * up_speed <= kMotionFfStopSpeedDegS)
                        s.motion_ff_direction = 0;
                    float desired_ff = 0;
                    if (s.motion_ff_direction > 0)
                        desired_ff = c.up_direction * c.up_ff_nm;
                    else if (s.motion_ff_direction < 0)
                        desired_ff = -c.up_direction * c.down_ff_nm;
                    const float max_change = kMotionFfSlewNmS * dt;
                    s.motion_feedforward_effort +=
                        clamp(desired_ff - s.motion_feedforward_effort, -max_change, max_change);
                }
                const float u = c.velocity_kp * e + next_i + s.feedforward_effort +
                                s.motion_feedforward_effort;
                float final = clamp(u, -limit, limit);
                if ((s.drive_position_deg <= drive_min(i) && final < 0) ||
                    (s.drive_position_deg >= drive_max(i) && final > 0))
                    final = 0;
                if ((u - final) * e <= 0)
                    r.integral = next_i;
                s.effort = final;
            }
        } else {
            s.effort = 0;
            s.feedforward_effort = 0;
            s.motion_feedforward_effort = 0;
            s.motion_ff_direction = 0;
            r.integral = 0;
            s.velocity_target_deg_s = 0;
            if (i == Yaw) {
                yaw_position_ff_ = 0;
                        yaw_position_ff_active_ = false;
                yaw_speed_ff_ = 0;
            }
        }

    }
    return output_frames(out);
}

Result ArmControl::start_jog(unsigned axis, int8_t direction, uint32_t now) {
    if (status_.faults)
        return Result::FaultLatched;
    // Jog是独立台架维护路径，仍要求两pitch同时在线/静止；正常控制已解耦。
    const uint8_t required = kPitchPair | (1u << axis);
    for (unsigned i = BigPitch; i <= Gripper; ++i) {
        if (!(required & (1u << i)))
            continue;
        const auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        if (!s.online ||
            (i >= Roll && uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms))
            return Result::NotOnline;
        if (std::fabs(s.drive_velocity_deg_s) > c.stationary_speed_deg_s)
            return Result::Moving;
        if (i <= SmallPitch) {
            if (s.motor_state != 0 || s.temperature_c >= c.max_temperature_c)
                return Result::FaultLatched;
            if (!s.calibrated)
                return Result::NotCalibrated;
            if (s.drive_position_deg < drive_min(i) || s.drive_position_deg > drive_max(i) ||
                s.position_deg < c.min_deg || s.position_deg > c.max_deg)
                return Result::OutOfRange;
        }
    }
    if (axis >= Roll) {
        // The C610 pulse uses its current stationary angle as a local origin.
        const auto result = capture_zero(1u << axis, now);
        if (result != Result::Ok)
            return result;
    }
    for (unsigned i = BigPitch; i <= Gripper; ++i)
        jog_start_deg_[i] = status_.joint[i].drive_position_deg;
    jog_axis_ = axis;
    jog_direction_ = direction;
    status_.enabled_mask = 1u << axis;
    status_.joint[axis].enabled = true;
    runtime_[axis].enable_ms = runtime_[axis].last_command_ms = now;
    runtime_[axis].dm_was_enabled = false;
    return Result::Ok;
}

void ArmControl::run_jog(uint32_t now) {
    constexpr float kStopTravelDeg = 1.0f;
    constexpr float kMaxTravelDeg = 2.0f;
    constexpr float kMaxSpeedDegS = 5.0f;
    constexpr float kEffortCap[kJointCount] = {0, 0.12f, 0.12f, 200.0f, 120.0f};
    const unsigned axis = jog_axis_;
    // 与start_jog一致：台架jog仍把两pitch当作同一安全组。
    const uint8_t required = kPitchPair | (1u << axis);
    const uint32_t age = now - runtime_[axis].enable_ms;
    for (unsigned i = BigPitch; i <= Gripper; ++i) {
        if (!(required & (1u << i)))
            continue;
        const auto &s = status_.joint[i];
        const auto &c = config_.joint[i];
        if (!s.online)
            return trip(FeedbackTimeout, i);
        if (i <= SmallPitch && (s.motor_state >= 8 || (i != axis && s.motor_state != 0)))
            return trip(MotorError, i);
        if (i <= SmallPitch && s.temperature_c >= c.max_temperature_c)
            return trip(OverTemperature, i);
        // A commanded pitch can pick up speed before reaching the pulse travel limit.
        const bool check_speed = i != axis || i >= Roll;
        if ((check_speed && std::fabs(s.drive_velocity_deg_s) > kMaxSpeedDegS) ||
            std::fabs(s.drive_position_deg - jog_start_deg_[i]) > kMaxTravelDeg ||
            (i <= SmallPitch &&
             (s.drive_position_deg < drive_min(i) || s.drive_position_deg > drive_max(i) ||
              s.position_deg < c.min_deg || s.position_deg > c.max_deg)))
            return trip(PositionLimit, i);
    }

    auto &s = status_.joint[axis];
    auto &r = runtime_[axis];
    if (std::fabs(s.drive_position_deg - jog_start_deg_[axis]) >= kStopTravelDeg)
        return disable();
    if (axis <= SmallPitch && s.motor_state != 1) {
        if (r.dm_was_enabled)
            return trip(MotorError, axis);
        if (age >= 200)
            return trip(EnableTimeout, axis);
        return;
    }
    r.dm_was_enabled = axis <= SmallPitch;
    const float cap = std::fmin(kEffortCap[axis], startup_effort_cap_[axis]);
    // Ramp a small open-loop pulse for direction checks before kinematic validation.
    s.effort = jog_direction_ * cap * std::fmin(float(age) / 200.0f, 1.0f);
}

unsigned ArmControl::step_jog(uint32_t now, int32_t request, bool permit, Frame out[kJointCount]) {
    constexpr uint32_t kJogTimeoutMs = 700;
    if (!out || !status_.configuration_valid || config_.joint_mask != kFourAxisJoints)
        return 0;
    const bool was_active = status_.enabled_mask != 0;
    if (was_active) {
        const uint32_t elapsed = now - last_step_ms_;
        auto &r = runtime_[jog_axis_];
        if (request == kJogStop || !permit) {
            disable();
            status_.last_result = request == kJogStop ? Result::Ok : Result::InterlockOpen;
        } else if (request != 0) {
            disable();
            status_.last_result = Result::Busy;
        } else if (uint32_t(now - r.enable_ms) >= kJogTimeoutMs) {
            disable();
        } else if (elapsed == 0 || elapsed > config_.control_period_ms + 2) {
            trip(TimingError, jog_axis_);
        } else {
            r.last_command_ms = now;
        }
    }
    last_step_ms_ = now;
    refresh(now);
    if (status_.enabled_mask) {
        run_jog(now);
    } else if (!was_active && request != 0) {
        if (request == kJogStop)
            status_.last_result = Result::Ok;
        else if (request == kJogClearFaults)
            clear_faults();
        else if (request == kJogCapturePitchZero)
            capture_zero(kPitchPair, now);
        else if (request < -int(Gripper) || request > int(Gripper))
            status_.last_result = Result::InvalidArgument;
        else if (!permit)
            status_.last_result = Result::InterlockOpen;
        else
            status_.last_result =
                start_jog(unsigned(request < 0 ? -request : request), request < 0 ? -1 : 1, now);
    }
    return output_frames(out);
}

unsigned ArmControl::output_frames(Frame out[kJointCount]) const {
    unsigned count = 0;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(config_.joint_mask & (1u << i))) continue;
        const auto &c = config_.joint[i];
        const auto &s = status_.joint[i];
        if (c.motor == Motor::Dm4310Mit) {
            auto &f = out[count++];
            f = {};
            f.bus = c.bus;
            f.id = command_id(i);
            if (!s.enabled || s.motor_state != 1) {
                std::memset(f.data, 0xff, 8);
                f.data[7] = s.enabled ? 0xfc : 0xfd;
            } else
                pack_dm_torque(c.direction * s.effort, c.p_max, c.v_max, c.t_max, f.data);
        } else {
            unsigned slot = 0;
            while (slot < count && (out[slot].bus != c.bus || out[slot].id != command_id(i)))
                ++slot;
            if (slot == count) {
                out[count] = {};
                out[count].bus = c.bus;
                out[count++].id = command_id(i);
            }
            const unsigned byte = ((c.motor_id - 1) % 4) * 2;
            float wire_command = c.direction * s.effort;
            if (c.motor == Motor::Gm6020Current) {
                // GM6020电流协议: ±16384对应±3000mA。不得套用C610的1mA/原码。
                wire_command = clamp(wire_command * (16384.0f / 3000.0f), -16384.0f, 16384.0f);
            }
            const auto raw = int16_t(wire_command);
            out[slot].data[byte] = uint16_t(raw) >> 8;
            out[slot].data[byte + 1] = uint16_t(raw);
        }
    }
    return count;
}
} // namespace engineer
