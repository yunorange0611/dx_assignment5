#pragma once
#include "Power/power_meter.h"
#include <cstdint>
namespace engineer {
struct ChassisPowerConfig {
    // 50W/500J 是裁判缓冲边界。confirmed 默认关闭；打开后功率环作用到
    // 速度目标，电流上限只做电机/C620 安全底，不因瞬时功率砍过坎扭矩。
    bool confirmed = false;
    // M1: false=功率计短时失联绝不主动停车/锁存，保持最后限制系数后平滑退出；
    // true=保留旧策略，缺帧即进入有界制动并最终锁存。仅影响功率计0x31帧，
    // 不改变CAN2发送故障、反馈/温度/时序和遥控等独立保护。低能量判据只在
    // 新帧上推进，重复读取同一帧不会凑满100ms。
    bool require_meter = false;
    float budget_w = 45, warning_energy_j = 300, stop_energy_j = 80, restart_energy_j = 350;
    float drive_current_ma = 16000, braking_current_ma = 8000;
    float power_filter_ms = 300;
    uint32_t timeout_ms = 200, stopping_timeout_ms = 2000;
    // 均从最后一帧有效数据的时间戳起算。宽限期后把限速系数平滑过渡到
    // meter_stale_speed_scale；数据恢复时再从当前系数平滑回到新计算值。
    uint32_t meter_hold_ms = 1000, meter_stale_ramp_ms = 1000, meter_recovery_ramp_ms = 1000;
    float meter_stale_speed_scale = 1.0f;
};
inline constexpr ChassisPowerConfig kChassisPowerConfig{};
enum class PowerPhase : uint8_t {
    Disabled,
    Ready,
    Driving,
    StopLowEnergy,
    StopFeedback,
    Inhibited
};
enum class PowerBlock : uint8_t {
    None,
    Unconfirmed,
    InvalidConfig,
    NoMeter,
    LowEnergy,
    FeedbackLost,
    StopTimeout,
    Stopped,
    ExternalStop
};
struct ChassisPowerState {
    PowerPhase phase = PowerPhase::Disabled;
    PowerBlock reason = PowerBlock::Unconfirmed;
    bool configuration_valid = false, confirmed = false, meter_online = false,
         meter_held = false, meter_stale = false, session_allowed = false;
    bool stop_requested = false, inhibited = false;
    float speed_scale = 0, drive_scale = 0, filtered_power_w = 0;
    // 只读诊断，不参与控制、不切电。原始功率超50W累计计时和本次最低缓冲
    // 能量用于判断调参是否过头；上电或成功清功率锁存后重新统计。
    uint32_t power_over_50_ms = 0;
    float min_energy_j = 0;
    uint8_t power_over_latched = 0;
    uint32_t updated_ms = 0, stopping_since_ms = 0;
};
class ChassisPower {
  public:
    explicit ChassisPower(const ChassisPowerConfig &config = kChassisPowerConfig);
    const ChassisPowerState &step(uint32_t now, const PowerMeterStatus &meter, bool enabled,
                                  bool wheels_stationary);
    bool clear(uint32_t now, const PowerMeterStatus &meter, bool disabled);
    // FM 在线写入；confirmed=false 时只更新参数，不参与控制。
    void set_power_tuning(float budget_w, float filter_ms);
    // 只清诊断计数，不影响相位、锁存或控制参数。调试时可在每次试跑前调用。
    void reset_diagnostics();
    // Signed mechanical direction after wheel direction conversion. Braking is
    // explicitly bounded too; this conservative feedback limiter is not a fitted model.
    void limit(const float candidate_ma[2], const float rotor_rpm[2], float out_ma[2]) const;
    const ChassisPowerState &status() const {
        return s_;
    }
    const ChassisPowerConfig &config() const {
        return c_;
    }

  private:
    enum class ScaleTransition : uint8_t { None, Stale, Recovery };
    ChassisPowerConfig c_;
    ChassisPowerState s_{};
    float filtered_power_w_ = 0;
    // Last effective speed scale, held while the meter link is briefly missing
    // and ramped on stale/recovery edges so speed never jumps by the full step.
    float effective_speed_scale_ = 1.0f, scale_from_ = 1.0f, scale_to_ = 1.0f;
    uint32_t last_valid_ms_ = 0;
    uint32_t scale_start_ms_ = 0, scale_duration_ms_ = 0;
    uint32_t previous_ms_ = 0;
    uint32_t diagnostic_previous_ms_ = 0;
    uint32_t low_energy_since_ms_ = 0;
    bool have_step_ = false;
    bool have_meter_sample_ = false, was_online_ = false, recovering_ = false;
    ScaleTransition scale_transition_ = ScaleTransition::None;
    bool diagnostic_have_step_ = false;
    bool diagnostic_energy_valid_ = false;
    bool low_energy_tracking_ = false;
    bool fresh(uint32_t now, const PowerMeterStatus &m) const;
    float advance_scale(uint32_t now, float target, uint32_t duration_ms, ScaleTransition kind);
    void update_diagnostics(uint32_t now, const PowerMeterStatus &m);
};
} // namespace engineer
