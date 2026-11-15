#pragma once
#include "Arm/arm_control.h"

// Existing control scenarios start with an already-settled Roll. Supply its
// feedback history without advancing the other motor or task clocks.
inline void prime_roll_feedback(engineer::ArmControl &arm, uint32_t now,
                                uint16_t encoder = 1000) {
    if (arm.status().joint[engineer::Roll].received)
        return;
    const uint8_t data[8] = {uint8_t(encoder >> 8), uint8_t(encoder), 0, 0, 0, 0, 25, 0};
    for (uint32_t age = 200; age > 0; age -= 4)
        arm.receive(1, arm.feedback_id(engineer::Roll), data, 8, now - age);
}
