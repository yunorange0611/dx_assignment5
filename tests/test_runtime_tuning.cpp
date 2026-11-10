#include "robot_control.h"
#include "robot_control_internal.h"
#include "Arm/arm_control.h"
#include "Arm/yaw_config.h"
#include "Chassis/chassis_control.h"
#include "Vehicle/robot_tuning.h"
#include "Remote/operator_control.h"
#include "chassis_task.h"
#include "task_ins.h"
#include "task_protect.h"
#include "vehicle_state_task.h"
#include "dr16_bsp.h"
#include <cmath>
#include <limits>

using namespace engineer;
extern "C" {
volatile unsigned test_checks = 0, test_failed_line = 0, test_done = 0;
extern uint32_t test_hal_tick;
}
#define CHECK(x) do { ++test_checks; if (!(x)) return __LINE__; } while (0)

namespace {
ChassisStatus wheels{};
InsStatus imu{};
uint32_t vehicle_reasons = 0;
unsigned revocations = 0;
bool feed(ArmControl &arm, unsigned joint, uint32_t now, unsigned encoder = 1000, int rpm = 0) {
    if (joint == BigPitch || joint == SmallPitch) {
        const uint8_t d[8] = {uint8_t(0x10 + joint), 0x80, 0, 0x80, 0x08, 0, 25, 25};
        return arm.receive(1, arm.feedback_id(joint), d, 8, now);
    }
    const uint8_t d[8] = {uint8_t(encoder >> 8), uint8_t(encoder),
        uint8_t(uint16_t(rpm) >> 8), uint8_t(rpm), 0, 0, 25, 0};
    return arm.receive(1, arm.feedback_id(joint), d, 8, now);
}
struct Rig {
    ArmControl arm{kArmConfig};
    ChassisCommand command{};
    Frame frames[kJointCount]{};
    uint32_t now = 1000;
    unsigned yaw_encoder = 1000;
    int yaw_rpm = 0;
    uint8_t left = DR16_SWITCH_DOWN;
    uint8_t right = DR16_SWITCH_MID;
    bool remote_online = true;
    bool update_feedback() {
        bool ok = feed(arm, Yaw, now, yaw_encoder, yaw_rpm);
        for (unsigned joint = BigPitch; joint < kJointCount; ++joint)
            ok = feed(arm, joint, now) && ok;
        return ok;
    }
    void tick(bool force_stop = false) {
        now += 4;
        test_hal_tick = now;
        update_feedback();
        wheels.updated_ms = now;
        for (unsigned i = 0; i < 2; ++i)
            wheels.feedback_ms[i] = now;
        imu.updated_ms = imu.received_ms = now;
        g_dr16_status.valid = remote_online;
        g_dr16_status.consecutive_frames = 3;
        g_dr16_status.received_ms = now;
        g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = left;
        g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = right;
        const RobotInterlocks locks{1, 1, 1, 1, -5, 5, 2};
        Robot_SetInterlocks(&locks);
        Robot_Execute(arm, now, force_stop);
        arm.step(now, frames);
        Robot_ReportArmState(arm.status());
        Robot_GetChassisCommand(&command);
    }
    void wait(unsigned cycles = 80) {
        for (unsigned i = 0; i < cycles; ++i)
            tick();
    }
};
int run() {
    CHECK(kArmConfig.hardware_confirmed && kArmConfig.pitch.mapping_confirmed &&
          kOperatorConfig.mapping_confirmed && kYawSafetyConfirmed);
    Rig r;
    wheels.ready = wheels.configuration_valid = wheels.hardware_confirmed = 1;
    wheels.wheels_stationary = 1;
    wheels.feedback_frames[0] = wheels.feedback_frames[1] = 1;
    imu.ready = imu.valid = 1;
    CHECK(r.update_feedback());
    CHECK(r.arm.capture_zero(kAllJoints, r.now) == Result::Ok);
    CHECK(r.arm.set_yaw_limits({-10, 10, 10, 15, 2, 3, 1}, r.now) == Result::Ok);
    CHECK(r.arm.enable(kFourAxisJoints, r.now) == Result::Ok);
    float hold[kJointCount] = {0, 3, -1, 2, 1};
    CHECK(r.arm.set_targets(kFourAxisJoints, hold, r.now) == Result::Ok);

    // Selecting an object is disarmed and can wait for neutral confirmation.
    robot_tune_mode = ROBOT_TUNE_LEFT_CURRENT;
    tune_enable = 1; // A simultaneous mode+enable write must still be disarmed.
    tune_current_ma = 50;
    r.tick();
    CHECK(tune_enable == 0 && !r.command.enabled && r.command.tune_value == 0);
    CHECK(tune_current_ma == 0 && r.arm.status().enabled_mask == kFourAxisJoints);
    r.wait();
    CHECK(!g_robot_tune_status.blocked && !g_robot_tune_status.ready);
    tune_enable = 1;
    r.tick();
    CHECK(!g_robot_tune_status.ready && !r.command.enabled);
    r.tick();
    CHECK(g_robot_tune_status.ready && r.command.enabled && r.command.tune_mode == 1);
    CHECK(!g_robot_status.protection_disabled);
    CHECK(r.arm.status().enabled_mask == kFourAxisJoints);
    tune_current_ma = 50;
    r.tick();
    CHECK(r.command.enabled && r.command.tune_value == 50);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);

    // Disarm stops only the selected drive, preserving every original holder.
    tune_enable = 0;
    wheels.wheels_stationary = 0;
    r.tick();
    CHECK(!r.command.enabled && r.command.tune_value == 0 && tune_current_ma == 0);
    CHECK(r.arm.status().enabled_mask == kFourAxisJoints);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);
    tune_enable = 1;
    r.tick();
    CHECK(!r.command.enabled && g_robot_tune_status.blocked &&
          g_robot_tune_status.reason == TuneWaitingStill);
    CHECK(r.arm.status().enabled_mask == kFourAxisJoints);
    wheels.wheels_stationary = 1;
    r.wait(4);
    CHECK(!r.command.enabled && !tune_enable && g_robot_tune_status.blocked);
    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_LEFT_CURRENT;
    r.tick();
    tune_enable = 1;
    r.tick();
    r.tick();
    CHECK(g_robot_tune_status.ready && r.command.enabled && r.command.tune_value == 0);
    tune_current_ma = 30;
    r.tick();
    const auto prior_epoch = r.command.tune_epoch;
    robot_tune_mode = ROBOT_TUNE_RIGHT_SPEED;
    tune_speed_rpm = 80;
    r.tick();
    CHECK(tune_enable == 0 && !r.command.enabled && r.command.tune_value == 0);
    CHECK(r.command.tune_epoch != prior_epoch && tune_speed_rpm == 0 && tune_current_ma == 0);
    tune_enable = 1;
    r.tick();
    tune_speed_rpm = 80;
    r.tick();
    CHECK(r.command.enabled && r.command.tune_mode == 4 && r.command.tune_value == 80);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);

    // Yaw tuning adds only Yaw; a disarm removes a Yaw that was enabled by this test.
    robot_tune_mode = ROBOT_TUNE_YAW_SPEED;
    r.tick();
    CHECK(!tune_enable && !r.command.enabled && r.arm.status().enabled_mask == kFourAxisJoints);
    tune_enable = 1;
    r.tick();
    r.tick();
    CHECK(g_robot_tune_status.ready && !r.command.enabled && r.arm.status().enabled_mask == kAllJoints);
    tune_yaw_speed_deg_s = 1;
    r.tick();
    CHECK(r.arm.status().joint[Yaw].velocity_target_deg_s == 1 &&
          r.arm.status().joint[Yaw].effort > 0);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);

    // Actual outward motion near the wire boundary must produce inward braking.
    r.yaw_encoder = 1205;
    r.yaw_rpm = 1;
    r.tick();
    CHECK(r.arm.status().joint[Yaw].position_deg > 9);
    CHECK(r.arm.status().joint[Yaw].velocity_target_deg_s <= 0 &&
          r.arm.status().joint[Yaw].effort < 0);
    tune_enable = 0;
    r.yaw_rpm = 0;
    r.tick();
    CHECK(!g_robot_tune_status.ready && r.arm.status().enabled_mask == kFourAxisJoints);
    CHECK(tune_yaw_speed_deg_s == 0 && !r.arm.status().joint[Yaw].effort);

    // An already-enabled Yaw stays enabled holding when its test is disarmed.
    CHECK(r.arm.enable(1u << Yaw, r.now) == Result::Ok);
    tune_enable = 1;
    r.tick();
    tune_yaw_speed_deg_s = -1;
    r.tick();
    CHECK(r.arm.status().joint[Yaw].velocity_target_deg_s < 0);
    tune_enable = 0;
    r.tick();
    CHECK(r.arm.status().enabled_mask == kAllJoints);
    CHECK(r.arm.status().joint[Yaw].target_deg == r.arm.status().joint[Yaw].position_deg);
    CHECK(r.arm.status().joint[Yaw].reference_deg == r.arm.status().joint[Yaw].position_deg);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);

    // Invalid online gains are rejected atomically while the existing arm holds.
    const float accepted_yaw_kp = g_robot_tune_status.yaw_velocity_kp;
    const float accepted_yaw_limit = g_robot_tune_status.yaw_current_limit_ma;
    tune_yaw_velocity_kp = std::numeric_limits<float>::quiet_NaN();
    r.tick();
    CHECK(g_robot_tune_status.yaw_parameter_result == uint32_t(Result::InvalidArgument));
    CHECK(g_robot_tune_status.yaw_velocity_kp == accepted_yaw_kp &&
          g_robot_tune_status.yaw_current_limit_ma == accepted_yaw_limit);
    CHECK(r.arm.status().enabled_mask == kAllJoints);
    tune_yaw_velocity_kp = accepted_yaw_kp;
    r.tick();

    // Position mode uses the same Yaw owner, starts at feedback, and enforces
    // both the lead bound and the configured wire range before submission.
    robot_tune_mode = ROBOT_TUNE_YAW_POSITION;
    tune_yaw_target_deg = -8;
    r.tick();
    CHECK(!tune_enable && !g_robot_tune_status.ready && r.arm.status().enabled_mask == kAllJoints);
    const float yaw_position = r.arm.status().joint[Yaw].position_deg;
    CHECK(tune_yaw_target_deg == yaw_position);
    tune_enable = 1;
    r.tick();
    r.tick();
    CHECK(g_robot_tune_status.ready && r.arm.status().joint[Yaw].target_deg == yaw_position);
    tune_yaw_target_deg = -8;
    r.tick();
    CHECK(g_robot_tune_status.ready && r.arm.status().joint[Yaw].target_deg < yaw_position &&
          r.arm.status().joint[Yaw].target_deg >= yaw_position - 3.0001f);
    tune_yaw_target_deg = 25;
    r.tick();
    CHECK(!tune_enable && g_robot_tune_status.blocked && !g_robot_tune_status.ready);
    CHECK(r.arm.status().enabled_mask == kAllJoints &&
          r.arm.status().joint[Yaw].target_deg == yaw_position);
    for (unsigned i = BigPitch; i < kJointCount; ++i)
        CHECK(r.arm.status().joint[i].target_deg == hold[i]);

    // The right switch is part of neutral: a queued gripper action cannot ride
    // through return-to-normal, and a premature resume request is consumed once.
    robot_tune_mode = ROBOT_TUNE_NORMAL;
    r.left = DR16_SWITCH_MID;
    r.right = DR16_SWITCH_UP;
    g_robot_debug_command = ROBOT_DEBUG_REQUEST_CONTROL;
    r.tick();
    CHECK(!r.command.enabled && r.arm.status().enabled_mask == kAllJoints);
    const auto gripper_before_resume = r.arm.status().joint[Gripper].target_deg;
    r.wait();
    g_robot_debug_command = ROBOT_DEBUG_REQUEST_CONTROL;
    r.tick();
    CHECK(!r.command.enabled && r.arm.status().joint[Gripper].target_deg == gripper_before_resume);
    r.right = DR16_SWITCH_MID;
    g_robot_debug_command = ROBOT_DEBUG_REQUEST_CONTROL;
    r.tick();
    CHECK(!r.command.enabled);
    r.wait();
    CHECK(!r.command.enabled); // No delayed execution of either premature request.

    // Full-axis return with command 4 must preserve holders despite a prior protection latch.
    g_robot_debug_command = ROBOT_DEBUG_REQUEST_CONTROL;
    r.tick();
    CHECK(r.arm.status().enabled_mask == kAllJoints && g_robot_status.active);
    CHECK(!g_robot_status.protection_disabled && !g_robot_status.emergency_stop);
    CHECK(g_robot_status.mode == ROBOT_GRAB && g_robot_tune_status.mode == 0 &&
          !g_robot_tune_status.ready && !g_robot_tune_status.blocked);
    CHECK(r.command.tune_mode == 0 && r.command.enabled);

    // Global STOP wins in the same cycle as selecting/enabling another mode.
    r.left = DR16_SWITCH_DOWN;
    robot_tune_mode = ROBOT_TUNE_LEFT_SPEED;
    tune_enable = 1;
    tune_speed_rpm = 100;
    Robot_StopControl();
    r.tick();
    CHECK(!r.arm.status().enabled_mask && !r.command.enabled && r.command.tune_value == 0);
    CHECK(!tune_enable && !g_robot_tune_status.ready && g_robot_tune_status.blocked);
    CHECK(g_robot_status.emergency_stop &&
          (g_robot_status.protection_current_reasons & PROTECT_STOP_REQUEST));
    tune_enable = 1;
    r.tick();
    CHECK(!r.arm.status().enabled_mask && !r.command.enabled);
    CHECK(!tune_enable && g_robot_tune_status.blocked);

    // A failed explicit enable cannot turn into a delayed start when feedback recovers.
    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_LEFT_SPEED;
    r.tick();
    wheels.ready = 0;
    tune_enable = 1;
    r.tick();
    CHECK(g_robot_tune_status.blocked && !g_robot_tune_status.ready && !r.command.enabled);
    wheels.ready = 1;
    r.wait(4);
    CHECK(g_robot_tune_status.blocked && !r.command.enabled && !tune_enable);

    // The open-current deadline reaches the shared output command and public
    // inputs; recovery requires resetting the tuning selection.
    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_LEFT_CURRENT;
    r.tick();
    tune_enable = 1;
    r.tick();
    tune_current_ma = 40;
    r.tick();
    CHECK(r.command.enabled && r.command.tune_value == 40);
    r.wait(125);
    CHECK(g_robot_tune_status.blocked && g_robot_tune_status.reason == TuneCurrentTimeout);
    CHECK(!r.command.enabled && r.command.tune_value == 0 && !tune_enable && tune_current_ma == 0);

    // A confirmed tipped IMU remains a global stop, including while tuning.
    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_RIGHT_SPEED;
    r.tick();
    tune_enable = 1;
    r.tick();
    tune_speed_rpm = 50;
    r.tick();
    CHECK(r.command.enabled);
    imu.tipped = 1;
    r.tick();
    CHECK(!r.command.enabled && !r.arm.status().enabled_mask && !tune_enable);
    CHECK(g_robot_status.emergency_stop &&
          (g_robot_status.protection_current_reasons & PROTECT_TIPPED));
    imu.tipped = 0;
    r.wait(4);
    CHECK(!r.command.enabled && g_robot_tune_status.blocked);

    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_RIGHT_SPEED;
    r.tick();
    tune_enable = 1;
    r.tick();
    tune_speed_rpm = 50;
    r.tick();
    CHECK(r.command.enabled);
    r.remote_online = false;
    r.tick();
    CHECK(!r.command.enabled && !tune_enable && g_robot_status.emergency_stop);
    r.remote_online = true;
    r.wait(80);
    CHECK(!r.command.enabled && !tune_enable && g_robot_tune_status.blocked);

    // Post-step arm faults override a wheel command before the next owner cycle.
    robot_tune_mode = 0;
    r.tick();
    robot_tune_mode = ROBOT_TUNE_RIGHT_SPEED;
    r.tick();
    tune_enable = 1;
    r.tick();
    tune_speed_rpm = 50;
    r.tick();
    CHECK(r.command.enabled && r.command.tune_value == 50);
    r.arm.transport_fault();
    Robot_ReportArmState(r.arm.status());
    CHECK(Robot_GetChassisCommand(&r.command) && !r.command.enabled && r.command.tune_value == 0);
    CHECK(!g_robot_tune_status.ready && g_robot_tune_status.blocked &&
          g_robot_status.emergency_stop);
    CHECK(revocations > 0);
    return 0;
}
} // namespace

extern "C" int Chassis_GetStatus(ChassisStatus *out) {
    if (!out) return 0;
    *out = wheels;
    return out->ready;
}
extern "C" int INS_GetStatus(InsStatus *out) {
    if (!out) return 0;
    *out = imu;
    return out->ready;
}
extern "C" void INS_RevokeCalibrationPermission(uint32_t) { ++revocations; }
extern "C" uint32_t Vehicle_GetProtectionReasons(uint32_t) { return vehicle_reasons; }
extern "C" void test_entry() {
    test_failed_line = run();
    test_done = 1;
    __asm volatile("bkpt #0");
    for (;;) {}
}
