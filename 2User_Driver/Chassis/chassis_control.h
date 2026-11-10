#pragma once
#include "Chassis/chassis_config.h"
#include "Chassis/chassis_motor.h"
namespace engineer {
enum ChassisFault : uint32_t {
    ChassisNoFault = 0,
    ChassisConfigFault = 1,
    ChassisFeedbackFault = 2,
    ChassisTransportFault = 4,
    ChassisTimingFault = 8,
    ChassisCommandFault = 16,
    ChassisTemperatureFault = 32,
    ChassisEnergyFault = 64
};
struct ChassisCommand {
    bool valid = false, enabled = false, stop = true, brake = true, reset = true;
    float speed_request = 0, turn_request = 0;
    uint32_t updated_ms = 0, epoch = 0;
    uint32_t stop_epoch = 0; // Explicit stop cancels pulses even when already disabled.
};
struct ChassisState {
    uint32_t updated_ms = 0, faults = 0;
    bool configuration_valid = false, enabled = false, wheels_stationary = false;
    float v_m_s = 0, omega_rad_s = 0, reference_v_m_s = 0, reference_omega_rad_s = 0;
    float target_rpm[2]{}, measured_rpm[2]{}, candidate_ma[2]{}, applied_ma[2]{};
};
enum class ChassisTuneResult : uint32_t { Idle, Applied, Invalid, Busy, NotReady };
class ChassisControl {
  public:
    explicit ChassisControl(const ChassisConfig &c);
    const ChassisState &step(uint32_t now, const WheelFeedback wheel[2], const ChassisCommand &cmd);
    void apply(const float ma[2], bool submitted);          // All final limits, or transport fault.
    void step_debug_speed(unsigned wheel, float rotor_rpm); // Same PI, owner-only suspended pulse.
    void direct_output(); // Open-loop pulse: discard speed-loop state before final submission.
    void trip(uint32_t reason);
    bool clear_faults(); // Disabled only; never reenables.
    ChassisTuneResult set_velocity_tuning(float kp, float ki, float current_ma);
    const ChassisConfig &config() const {
        return c_;
    }
    const ChassisState &status() const {
        return s_;
    }
    static void differential(float v, float omega, float diameter, float track, float ratio,
                             float rpm[2]);

  private:
    ChassisConfig c_;
    const float startup_current_limit_ma_;
    ChassisState s_{};
    float integral_[2]{}, error_[2]{}, dt_ = 0;
    uint32_t previous_ms_ = 0, epoch_ = 0, stationary_since_ = 0;
    bool have_step_ = false, tracking_stationary_ = false, have_epoch_ = false;
    bool previous_reset_ = false, rearm_required_ = false;
};
} // namespace engineer
