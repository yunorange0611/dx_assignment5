#pragma once
#include "Power/power_meter.h"
#include <cstdint>
namespace engineer {
struct ChassisPowerConfig {
    // 50W/500J 是裁判缓冲边界。confirmed 默认关闭；打开后功率环作用到
    // 速度目标，电流上限只做电机/C620 安全底，不因瞬时功率砍过坎扭矩。
    bool confirmed = false;
    float budget_w = 45, warning_energy_j = 300, stop_energy_j = 80, restart_energy_j = 350;
    float drive_current_ma = 16000, braking_current_ma = 8000;
    float power_filter_ms = 300;
    uint32_t timeout_ms = 200, stopping_timeout_ms = 2000;
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
         session_allowed = false;
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
    ChassisPowerConfig c_;
    ChassisPowerState s_{};
    float filtered_power_w_ = 0;
    uint32_t previous_ms_ = 0;
    uint32_t diagnostic_previous_ms_ = 0;
    uint32_t low_energy_since_ms_ = 0;
    bool have_step_ = false;
    bool diagnostic_have_step_ = false;
    bool diagnostic_energy_valid_ = false;
    bool low_energy_tracking_ = false;
    bool fresh(uint32_t now, const PowerMeterStatus &m) const;
    void update_diagnostics(uint32_t now, const PowerMeterStatus &m);
};
} // namespace engineer
