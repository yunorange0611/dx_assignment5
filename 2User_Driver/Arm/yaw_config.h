#pragma once
#include "Arm/arm_control.h"

namespace engineer {
// 大yaw负责人维护；仍由ArmControl/arm_task统一闭环并在CAN1发送。
// 禁止另建同一6020的驱动对象或第二个0x1FF发送者。
// 以下方向、零位范围、PID和电压原码上限仍是台架占位，需负责人实测。
// 对正车头且失能时，读取 g_arm_status.joint[0].raw_encoder 后填入这里。
// -1=尚未填写，禁止固定零点自动开控；有效值0..8191。修改后重新编译烧录。
inline constexpr int16_t kYawZeroEncoder = 6352; // 2026-11-13 用户对正车头，SWD连续读取0x18D0
static_assert(kYawZeroEncoder >= -1 && kYawZeroEncoder < 8192, "Yaw zero must be -1 or 0..8191");
inline constexpr JointConfig kYawConfig = {
    Motor::Gm6020Voltage,
    1,
    1,
    0, // 用户确认：只能用电压模式；CAN1；ID1，反馈0x205，指令0x1FF第一槽
    1.0f,
    1, // 37T:37T；右限位ecd4352相对零点6352为-87.890625deg，右负左正
    -85,    // 右运行软限位；实测右边界-87.890625deg，留约2.9deg余量
    85,     // 左运行软限位；按用户确认与右侧对称，留同等余量
    10,     // max_speed_deg_s：静态参考速度上限，行驶还受姿态包络限制
    30,     // max_accel_deg_s2：参考加速度上限
    25000.0f, // max_effort：电压原码上限；旧驱动满量程25000，手册可到30000
    2.0f,     // position_kp：位置P
    250.0f,   // velocity_kp：速度P，raw/(deg/s)
    10.0f,    // velocity_ki：速度I，raw/deg
    0,
    0,
    0,
    70, // 无DM协议量程；温度门槛°C
    2.0f, // 越界余量deg
    2.0f, // 静止速度阈值deg/s
    kYawZeroEncoder
};
// These values are placeholders until the wire routing and stopping distance are
// measured. Confirmation is separate from joint PID/protocol configuration.
inline constexpr bool kYawSafetyConfirmed = false;
inline constexpr float kYawAbsoluteHalfRangeDeg = 85.0f; // 实测边界内的运行上限
inline constexpr float kYawBrakingMarginDeg = 2.0f;
inline constexpr float kYawBrakingAccelDegS2 = 15.0f;
inline constexpr float kYawMaxTargetLeadDeg = 5.0f;
} // namespace engineer
