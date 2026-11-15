#include "Power/chassis_power.h"
#include <cmath>
namespace engineer {
namespace {
constexpr float kPowerOverW = 50;
constexpr uint32_t kDiagnosticMaxGapMs = 100;
// A single bad energy frame must not start a stop. Hold the low reading for
// several meter frames before treating it as a real low-buffer condition.
constexpr uint32_t kLowEnergyDebounceMs = 100;
float clamp(float x, float l, float h) {
    return std::fmax(l, std::fmin(h, x));
}
} // namespace
ChassisPower::ChassisPower(const ChassisPowerConfig &c) : c_(c) {
    const float values[] = {c.budget_w,          c.warning_energy_j, c.stop_energy_j,
                            c.restart_energy_j,  c.drive_current_ma, c.braking_current_ma,
                            c.power_filter_ms};
    bool ok = true;
    for (float v : values)
        ok &= std::isfinite(v) && v > 0;
    ok &= c.budget_w < 50 && c.warning_energy_j > c.stop_energy_j &&
          c.restart_energy_j >= c.warning_energy_j && c.restart_energy_j <= 500 &&
          c.drive_current_ma <= 20000 && c.braking_current_ma <= c.drive_current_ma &&
          c.power_filter_ms <= 5000 && c.timeout_ms > 0 && c.stopping_timeout_ms > 0 &&
          c.stopping_timeout_ms < 10000;
    ok &= std::isfinite(c.meter_stale_speed_scale) && c.meter_stale_speed_scale >= 0 &&
          c.meter_stale_speed_scale <= 1 && c.meter_hold_ms <= 5000 &&
          c.meter_stale_ramp_ms <= 5000 && c.meter_recovery_ramp_ms <= 5000;
    s_.configuration_valid = ok;
    s_.confirmed = c.confirmed;
}
void ChassisPower::set_power_tuning(float budget_w, float filter_ms) {
    // Invalid writes leave the active values unchanged.
    if (std::isfinite(budget_w) && budget_w > 0 && budget_w < 50) c_.budget_w = budget_w;
    if (std::isfinite(filter_ms) && filter_ms > 0 && filter_ms <= 5000)
        c_.power_filter_ms = filter_ms;
}
bool ChassisPower::fresh(uint32_t now, const PowerMeterStatus &m) const {
    return m.frames && uint32_t(now - m.received_ms) <= c_.timeout_ms && std::isfinite(m.power_w) &&
           std::isfinite(m.energy_j);
}
float ChassisPower::advance_scale(uint32_t now, float target, uint32_t duration_ms,
                                 ScaleTransition kind) {
    if (scale_transition_ != kind) {
        // No-op when the current value already equals the requested target; this
        // prevents a completed stale ramp from restarting every control cycle.
        if (scale_transition_ == ScaleTransition::None && effective_speed_scale_ == target)
            return effective_speed_scale_;
        scale_from_ = effective_speed_scale_;
        scale_start_ms_ = now;
        scale_transition_ = kind;
    }
    scale_to_ = target;
    scale_duration_ms_ = duration_ms;
    if (scale_duration_ms_ == 0) {
        effective_speed_scale_ = target;
        scale_transition_ = ScaleTransition::None;
        return effective_speed_scale_;
    }
    const uint32_t elapsed = uint32_t(now - scale_start_ms_);
    const float t = elapsed >= scale_duration_ms_
                        ? 1.0f
                        : float(elapsed) / float(scale_duration_ms_);
    // Smoothstep: no value jump at either end and zero slope at completion.
    const float blend = t * t * (3.0f - 2.0f * t);
    effective_speed_scale_ = scale_from_ + (scale_to_ - scale_from_) * blend;
    if (t >= 1.0f) {
        effective_speed_scale_ = target;
        scale_transition_ = ScaleTransition::None;
    }
    return effective_speed_scale_;
}
void ChassisPower::reset_diagnostics() {
    s_.power_over_50_ms = 0;
    s_.min_energy_j = 0;
    s_.power_over_latched = 0;
    diagnostic_previous_ms_ = 0;
    diagnostic_have_step_ = false;
    diagnostic_energy_valid_ = false;
}
void ChassisPower::update_diagnostics(uint32_t now, const PowerMeterStatus &m) {
    // 只统计连续的新鲜帧，丢帧/超时期间不累计；不影响功率环相位。
    if (!fresh(now, m)) {
        diagnostic_previous_ms_ = now;
        diagnostic_have_step_ = false;
        return;
    }
    if (!diagnostic_energy_valid_ || m.energy_j < s_.min_energy_j) {
        s_.min_energy_j = m.energy_j;
        diagnostic_energy_valid_ = true;
    }
    if (m.power_w > kPowerOverW)
        s_.power_over_latched = 1;
    if (diagnostic_have_step_ && m.power_w > kPowerOverW) {
        const uint32_t elapsed = uint32_t(now - diagnostic_previous_ms_);
        if (elapsed > 0 && elapsed <= kDiagnosticMaxGapMs) {
            if (s_.power_over_50_ms > UINT32_MAX - elapsed)
                s_.power_over_50_ms = UINT32_MAX;
            else
                s_.power_over_50_ms += elapsed;
        }
    }
    diagnostic_previous_ms_ = now;
    diagnostic_have_step_ = true;
}
const ChassisPowerState &ChassisPower::step(uint32_t now, const PowerMeterStatus &m, bool enabled,
                                            bool stationary) {
    const bool had_permission = s_.session_allowed;
    const bool was_online = was_online_;
    s_.updated_ms = now;

    const bool online = fresh(now, m);
    const bool had_sample = have_meter_sample_;
    const uint32_t previous_last_valid = last_valid_ms_;
    if (online) {
        // Count the hold window from the meter's own last frame timestamp, not
        // from the last cycle that still considered the same frame fresh.
        if (!have_meter_sample_ || int32_t(m.received_ms - last_valid_ms_) >= 0)
            last_valid_ms_ = m.received_ms;
        have_meter_sample_ = true;
    }
    // A frame already seen in an earlier cycle is fresh but is not new data.
    // Low-energy debouncing must only advance on genuinely new frames.
    const bool new_frame =
        online && (!had_sample || int32_t(m.received_ms - previous_last_valid) > 0);
    const uint32_t since_valid = have_meter_sample_ ? uint32_t(now - last_valid_ms_) : UINT32_MAX;
    const bool held = !online && have_meter_sample_ && since_valid <= c_.meter_hold_ms;
    const bool stale = !online && !held;
    s_.meter_online = online;
    s_.meter_held = held;
    s_.meter_stale = stale;
    was_online_ = online;
    if (!online) {
        recovering_ = false;
        if (held)
            scale_transition_ = ScaleTransition::None; // Freeze the last factor.
    } else if (had_sample && !was_online) {
        recovering_ = true;
    }
    update_diagnostics(now, m);
    s_.stop_requested = false;
    s_.speed_scale = s_.drive_scale = 0;
    s_.filtered_power_w = 0;
    s_.session_allowed = false;
    if (!s_.configuration_valid || !c_.confirmed) {
        s_.phase = PowerPhase::Disabled;
        s_.reason = s_.configuration_valid ? PowerBlock::Unconfirmed : PowerBlock::InvalidConfig;
        return s_;
    }
    if (s_.inhibited) {
        s_.phase = PowerPhase::Inhibited;
        return s_;
    }
    // An already-confirmed low-energy (or strict meter-loss) stop continues even
    // if the meter link is lost later; it is never auto-cancelled by recovery.
    const bool stopping =
        s_.phase == PowerPhase::StopLowEnergy || s_.phase == PowerPhase::StopFeedback;
    if (stopping) {
        if (!enabled || stationary ||
            uint32_t(now - s_.stopping_since_ms) >= c_.stopping_timeout_ms) {
            s_.inhibited = true;
            s_.phase = PowerPhase::Inhibited;
            s_.reason = stationary
                            ? PowerBlock::Stopped
                            : (!enabled ? PowerBlock::ExternalStop : PowerBlock::StopTimeout);
        } else {
            s_.session_allowed = true;
            s_.stop_requested = true;
        }
        return s_;
    }
    if (!online)
        low_energy_tracking_ = false;
    // Strict legacy policy: a missing meter is itself a stop condition. Only
    // this path is enabled by require_meter; CAN2 transmit faults stay separate.
    if (!online && c_.require_meter) {
        if (enabled && had_permission) {
            s_.phase = PowerPhase::StopFeedback;
            s_.reason = PowerBlock::FeedbackLost;
            s_.stopping_since_ms = now;
            s_.session_allowed = true;
            s_.stop_requested = true;
        } else {
            s_.phase = PowerPhase::Disabled;
            s_.reason = PowerBlock::NoMeter;
        }
        return s_;
    }

    // Low-energy confirmation requires new, low frames. Re-reading the same
    // frame cannot advance the 100ms debounce, and a link timeout clears it.
    const bool low_frame = new_frame && m.energy_j <= c_.stop_energy_j;
    if (low_frame) {
        if (!low_energy_tracking_) {
            low_energy_tracking_ = true;
            low_energy_since_ms_ = now;
        }
    } else if (new_frame || !online) {
        low_energy_tracking_ = false;
    }
    const bool low_energy_confirmed =
        low_energy_tracking_ && new_frame &&
        uint32_t(now - low_energy_since_ms_) >= kLowEnergyDebounceMs;
    if (low_energy_confirmed) {
        if (enabled && had_permission) {
            s_.phase = PowerPhase::StopLowEnergy;
            s_.reason = PowerBlock::LowEnergy;
            s_.stopping_since_ms = now;
            s_.session_allowed = true;
            s_.stop_requested = true;
        } else {
            s_.phase = PowerPhase::Disabled;
            s_.reason = PowerBlock::LowEnergy;
        }
        return s_;
    }

    if (held) {
        // M1 hold: keep the last effective limit factor and keep driving. Old
        // frames are never reused to advance a new low-energy decision.
        s_.phase = enabled ? PowerPhase::Driving : PowerPhase::Ready;
        s_.reason = PowerBlock::FeedbackLost;
        s_.session_allowed = true;
        s_.stop_requested = false;
        s_.speed_scale = effective_speed_scale_;
        s_.drive_scale = 1;
        s_.filtered_power_w = filtered_power_w_;
        return s_;
    }
    if (stale) {
        // Hold expired: ramp the factor to the stale target instead of jumping.
        effective_speed_scale_ = advance_scale(now, c_.meter_stale_speed_scale,
                                               c_.meter_stale_ramp_ms,
                                               ScaleTransition::Stale);
        s_.phase = enabled ? PowerPhase::Driving : PowerPhase::Ready;
        s_.reason = have_meter_sample_ ? PowerBlock::FeedbackLost : PowerBlock::NoMeter;
        s_.session_allowed = true;
        s_.stop_requested = false;
        s_.speed_scale = effective_speed_scale_;
        s_.drive_scale = 1;
        s_.filtered_power_w = 0; // No current measurement; meter_stale is set.
        return s_;
    }

    // Fresh data: normal power/energy scaling.
    s_.phase = enabled ? PowerPhase::Driving : PowerPhase::Ready;
    s_.reason = PowerBlock::None;
    s_.session_allowed = true;
    // Filter the measured power so a short ledge/bump spike does not cut the
    // speed target mid-obstacle. The 500J referee buffer absorbs the spike.
    const uint32_t elapsed = have_step_ ? uint32_t(now - previous_ms_) : 0;
    previous_ms_ = now;
    if (!have_step_ || elapsed == 0 || elapsed > 100)
        filtered_power_w_ = m.power_w;
    else {
        const float dt = elapsed * .001f;
        const float tau = c_.power_filter_ms * .001f;
        filtered_power_w_ += dt / (tau + dt) * (m.power_w - filtered_power_w_);
    }
    have_step_ = true;
    s_.filtered_power_w = filtered_power_w_;
    const float energy_scale =
        clamp((m.energy_j - c_.stop_energy_j) / (c_.warning_energy_j - c_.stop_energy_j), 0, 1);
    const float power_scale = filtered_power_w_ > c_.budget_w
                                  ? clamp(c_.budget_w / filtered_power_w_, 0, 1)
                                  : 1;
    const float target_scale = std::fmin(energy_scale, power_scale);
    if (recovering_) {
        // Ramp back to the freshly computed target instead of stepping from the
        // released stale factor straight to a low limit (or vice versa).
        effective_speed_scale_ = advance_scale(now, target_scale,
                                               c_.meter_recovery_ramp_ms,
                                               ScaleTransition::Recovery);
        if (scale_transition_ == ScaleTransition::None)
            recovering_ = false;
    } else {
        // First-ever frame or uninterrupted online data: use the normal target
        // immediately. The filtered power already provides the normal smoothing.
        effective_speed_scale_ = target_scale;
        scale_transition_ = ScaleTransition::None;
    }
    // The drive-current ceiling stays at the motor/C620 safety value; power
    // limiting only lowers the speed target, so slope/ledge torque is preserved.
    s_.speed_scale = effective_speed_scale_;
    s_.drive_scale = 1;
    return s_;
}
bool ChassisPower::clear(uint32_t now, const PowerMeterStatus &m, bool disabled) {
    if (!disabled || !s_.configuration_valid || !c_.confirmed || !fresh(now, m) ||
        m.energy_j < c_.restart_energy_j)
        return false;
    s_.inhibited = false;
    s_.phase = PowerPhase::Disabled;
    s_.reason = PowerBlock::None;
    s_.session_allowed = false;
    s_.stop_requested = false;
    s_.meter_online = s_.meter_held = s_.meter_stale = false;
    s_.speed_scale = s_.drive_scale = 0;
    s_.filtered_power_w = 0;
    s_.stopping_since_ms = 0;
    filtered_power_w_ = 0;
    effective_speed_scale_ = 1.0f;
    scale_from_ = scale_to_ = 1.0f;
    scale_transition_ = ScaleTransition::None;
    scale_start_ms_ = scale_duration_ms_ = 0;
    last_valid_ms_ = 0;
    have_meter_sample_ = was_online_ = recovering_ = false;
    previous_ms_ = 0;
    have_step_ = false;
    low_energy_since_ms_ = 0;
    low_energy_tracking_ = false;
    reset_diagnostics();
    return true;
}
void ChassisPower::limit(const float candidate[2], const float rpm[2], float out[2]) const {
    out[0] = out[1] = 0;
    if (!s_.session_allowed)
        return;
    for (unsigned i = 0; i < 2; ++i) {
        if (!std::isfinite(candidate[i]) || !std::isfinite(rpm[i])) {
            out[0] = out[1] = 0;
            return;
        }
        const bool braking = std::fabs(rpm[i]) > 5 && candidate[i] * rpm[i] < 0;
        if (s_.stop_requested) {
            // Never propel during a power-stop. Near zero a conservative hold
            // current remains bounded; the measured stationary timer finishes it.
            if (braking || std::fabs(rpm[i]) <= 5)
                out[i] = clamp(candidate[i], -c_.braking_current_ma, c_.braking_current_ma);
        } else {
            const float bound = braking ? c_.braking_current_ma
                                        : c_.drive_current_ma * s_.drive_scale;
            out[i] = clamp(candidate[i], -bound, bound);
        }
    }
}
} // namespace engineer
