#pragma once
#include <cstdint>

namespace engineer {
struct ChassisConfig {
    bool hardware_confirmed = false;
    uint8_t motor_id[2] = {1, 2}; // Physical test: old index 0 (ID2) was right; ID1 is left.
    int8_t direction[2] = {1, -1}; // Preserve each motor's validated sign: ID1 +1, ID2 -1.
    float diameter_m = .145f, track_m = .330f, gear_ratio = 19.0f;
    float max_speed_m_s = .25f, max_turn_rad_s = .6f;
    float acceleration_m_s2 = .3f, deceleration_m_s2 = .5f, angular_accel_rad_s2 = 1.0f;
    float max_rotor_rpm = 1500, max_current_ma = 10000;
    float velocity_kp = 1.0f, velocity_ki = .5f; // mA/rpm; placeholders, not tuned.
    float stationary_rpm = 15, max_temperature_c = 70;
    uint32_t feedback_timeout_ms = 50, command_timeout_ms = 40, stationary_ms = 200;
    uint32_t period_ms = 4;
};
inline constexpr ChassisConfig kChassisConfig{};
} // namespace engineer
