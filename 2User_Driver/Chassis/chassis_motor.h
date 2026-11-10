#pragma once
#include <cstdint>
namespace engineer {
struct WheelFeedback {
    float rotor_rpm = 0, current_ma = 0, temperature_c = 0;
    uint16_t encoder = 0;
    uint32_t received_ms = 0, frames = 0;
};
class ChassisMotor {
public:
    bool receive(const uint8_t *data, uint8_t len, uint32_t received_ms);
    const WheelFeedback &status() const { return status_; }
    static void pack_current(float left_ma, float right_ma, const uint8_t id[2],
                             const int8_t direction[2], uint8_t data[8]);
private:
    WheelFeedback status_{};
};
}
