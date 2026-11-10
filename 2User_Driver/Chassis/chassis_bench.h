#pragma once
#include "Chassis/chassis_config.h"
#include "Chassis/chassis_motor.h"
#include <cstdint>

namespace engineer {
// Kept numerically compatible with the debugger-facing ChassisDebugMode.
enum class ChassisBenchMode : uint32_t { Stop = 0, Clear = 1, Current = 2, Speed = 3 };
enum class ChassisBenchResult : uint32_t {
    Idle,
    Accepted,
    Stopped,
    ClearRequested,
    InvalidRequest,
    NotReady,
    Busy,
    Cancelled,
    Expired
};
struct ChassisBenchRequest {
    uint32_t mode = 0, wheel = 0, duration_ms = 0;
    float value = 0;
    uint32_t suspended_confirm = 0, request_seq = 0;
};
struct ChassisBenchHealth {
    bool hardware_confirmed = false, mapping_confirmed = false, remote_ok = false;
    bool owner_idle = false, feedback_fresh = false, temperature_ok = false;
    bool wheels_stationary = false, transport_ok = false, timing_ok = false;
    bool controller_faulted = false, controller_enabled = false, global_stop = false;
    uint32_t stop_epoch = 0;
};
// Plain snapshots only: no task headers, remote driver, HAL or RTOS dependency.
// received_ms is the RC source timestamp; robot_updated_ms is owner publication.
struct ChassisBenchRemoteInput {
    bool mapping_confirmed = false, valid = false, switches_in_test_position = false;
    uint32_t received_ms = 0, consecutive_frames = 0;
    int16_t channel[5]{}; // Preserve all original DR16 raw channels, including the dial.
    bool owner_idle = false, robot_ready = false;
    uint32_t robot_updated_ms = 0;
};
struct ChassisBenchFeedbackHealth {
    bool fresh = true, temperature_ok = true, stationary = true;
};
// Preserve the suspended entry gates: RC <=100ms, >=3 frames, sticks within
// 30 raw counts, owner publication <=40ms and the explicit test switch position.
bool chassis_bench_remote_ready(uint32_t now_ms, const ChassisBenchRemoteInput &input);
ChassisBenchFeedbackHealth chassis_bench_feedback_health(uint32_t now_ms,
                                                         const WheelFeedback wheel[2],
                                                         const ChassisConfig &config);
struct ChassisBenchState {
    uint32_t consumed_seq = 0;
    ChassisBenchResult result = ChassisBenchResult::Idle;
    uint32_t mode = 0, wheel = 0;
    float value = 0;
    uint32_t until_ms = 0;
    bool active = false, clear_requested = false;
};

// Owner-only pure policy: no HAL, CAN, RTOS, or motor writes.
// A failed request is consumed once; later feedback never starts an old request.
class ChassisBenchGate {
  public:
    const ChassisBenchState &step(uint32_t now_ms, const ChassisBenchRequest &request,
                                  const ChassisBenchHealth &health);
    const ChassisBenchState &status() const {
        return state_;
    }

  private:
    ChassisBenchState state_{};
    uint32_t previous_stop_epoch_ = 0;
    bool initialized_ = false;
    void cancel(ChassisBenchResult result);
};
} // namespace engineer
