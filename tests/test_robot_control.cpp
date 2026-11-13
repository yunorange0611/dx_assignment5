#include "robot_control.h"
#include "robot_control_internal.h"
#include "task_protect.h"
#include "Remote/operator_control.h"
#include "Chassis/chassis_control.h"
#include "Vehicle/robot_tuning.h"
#include "chassis_task.h"
#include "dr16_bsp.h"
#include "vehicle_state_task.h"
#include "task_ins.h"
#include <limits>
#include <cmath>
using namespace engineer;
extern "C" volatile unsigned test_checks;
extern "C" uint32_t test_hal_tick;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
namespace {
uint32_t vehicle_reasons = 0;
unsigned vehicle_reads = 0, calibration_revocations = 0;
ChassisStatus test_wheels{};
InsStatus test_imu{};

bool feed_all(ArmControl &a, uint32_t now) {
    bool ok = true;
    for (unsigned j = 0; j < kJointCount; ++j) {
        if (!(kArmConfig.joint_mask & (1u << j)))
            continue;
        if (j == BigPitch || j == SmallPitch) {
            const uint8_t d[8] = {uint8_t(0x10 + j), 0x80, 0, 0x80, 0x08, 0, 25, 25};
            ok = a.receive(1, a.feedback_id(j), d, 8, now) && ok;
        } else {
            const uint8_t d[8] = {3, 232, 0, 0, 0, 0, 25, 0};
            ok = a.receive(1, a.feedback_id(j), d, 8, now) && ok;
        }
    }
    return ok;
}
void fresh_inputs(uint32_t now) {
    test_hal_tick = now;
    g_dr16_status.valid = 1;
    g_dr16_status.consecutive_frames = 3;
    g_dr16_status.received_ms = now;
    for (auto &ch : g_dr16_status.frame.channel)
        ch = 0;
    g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
    g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
    test_wheels = {};
    test_wheels.updated_ms = now;
    test_wheels.ready = test_wheels.configuration_valid = test_wheels.hardware_confirmed = 1;
    test_wheels.wheels_stationary = 1;
    for (unsigned i = 0; i < 2; ++i) {
        test_wheels.feedback_ms[i] = now;
        test_wheels.feedback_frames[i] = 1;
    }
    test_imu = {};
    test_imu.updated_ms = test_imu.received_ms = now;
    test_imu.ready = test_imu.valid = 1;
}
} // namespace
extern "C" uint32_t Vehicle_GetProtectionReasons(uint32_t now) {
    ++vehicle_reads;
    return now == test_hal_tick ? vehicle_reasons : PROTECT_VEHICLE_STATE;
}
extern "C" void INS_RevokeCalibrationPermission(uint32_t) {
    ++calibration_revocations;
}
extern "C" int INS_GetStatus(InsStatus *out) {
    if (!out)
        return 0;
    *out = test_imu;
    return out->ready;
}
extern "C" int Chassis_GetStatus(ChassisStatus *out) {
    if (!out)
        return 0;
    *out = test_wheels;
    return out->ready;
}
int test_robot_control() {
    // Legacy manual-zero cases use -1; the fixed-zero case overrides it explicitly.
    auto cfg = kArmConfig;
    cfg.joint[Yaw].fixed_zero_encoder = -1;
    // The legacy whole-vehicle bridge tests below expect the old command gate.
    kChassisIndependent = false;
    static_assert(sizeof(ChassisCommand) < sizeof(RobotControlStatus));
    static_assert(unsigned(RobotMode::Standby) == ROBOT_STANDBY);
    static_assert(unsigned(RobotMode::Drive) == ROBOT_DRIVE);
    static_assert(unsigned(DriveGear::Fast) == ROBOT_GEAR_FAST);
    ChassisCommand command{};
    CHECK(!Robot_GetChassisCommand(nullptr));
    CHECK(!Robot_GetChassisCommand(&command));
    CHECK(!command.enabled && command.stop && command.brake && command.reset);
    RobotControlStatus s{};
    CHECK(!Robot_GetControlStatus(nullptr));
    CHECK(!Robot_GetControlStatus(&s));
    CHECK(!Robot_SetInterlocks(nullptr));
    RobotInterlocks locks{};
    locks.chassis_stopped = 1;
    locks.arm_allows_drive = 1;
    locks.arm_allows_fast = 1;
    locks.drive_yaw_allowed = 1;
    locks.drive_yaw_min_deg = -20;
    locks.drive_yaw_max_deg = 20;
    locks.drive_yaw_max_speed_deg_s = 2;
    CHECK(Robot_SetInterlocks(&locks));
    g_dr16_status.valid = 1;
    g_dr16_status.consecutive_frames = 3;
    g_dr16_status.received_ms = 1234;
    g_dr16_status.frame.switches[0] = DR16_SWITCH_UP;
    g_dr16_status.frame.switches[1] = DR16_SWITCH_MID;
    CHECK(Robot_RequestControl());
    ArmControl arm(cfg);
    Robot_Execute(arm, 1234, false);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.updated_ms == 1234 && s.ready && s.remote_online);
    CHECK(s.joint_mask == kArmConfig.joint_mask && g_robot_status.joint_mask == s.joint_mask);
    CHECK(!s.active && s.protection_disabled && s.emergency_stop && s.stop_chassis);
    CHECK(s.protection_current_reasons & PROTECT_CONFIG);
    CHECK(!(s.protection_current_reasons & PROTECT_REMOTE));
    CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    CHECK(!(s.protection_current_reasons & (PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED |
                                            PROTECT_VEHICLE_STATE | PROTECT_POWER)));
    CHECK(s.longitudinal_brake && s.reset_chassis_motion);
    CHECK(!s.fast_requested && s.drive_gear == ROBOT_GEAR_SLOW);
    CHECK(s.longitudinal_speed_request == 0 && s.turn_rate_request == 0 &&
          arm.status().enabled_mask == 0);
    CHECK(Robot_GetChassisCommand(&command));
    CHECK(command.valid && !command.enabled && command.stop && command.brake && command.reset);
    CHECK(command.updated_ms == s.updated_ms && command.epoch == s.motion_epoch &&
          command.stop_epoch == s.stop_epoch);
    // Mapping observations remain useful while configuration/STOP deny all output.
    g_dr16_status.frame.channel[0] = -660;
    g_dr16_status.frame.channel[1] = 330;
    g_dr16_status.frame.channel[2] = -39;
    g_dr16_status.frame.channel[3] = 660;
    g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
    g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_UP;
    Robot_Execute(arm, 1234, true);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.remote_axis[0] == 1 && s.remote_axis[1] == 0 &&
          std::fabs(s.remote_axis[2] - (.5f - .06f) / .94f) < .0001f && s.remote_axis[3] == -1);
    CHECK(s.remote_switch[0] == DR16_SWITCH_MID && s.remote_switch[1] == DR16_SWITCH_UP);
    CHECK(!s.active && !arm.status().enabled_mask && Robot_GetChassisCommand(&command) &&
          !command.enabled && command.speed_request == 0 && command.turn_request == 0);
    for (unsigned ch = 0; ch < 5; ++ch)
        g_dr16_status.frame.channel[ch] = 0;
    g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
    g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
    // Diagnostic storage cannot turn into a control request, including its source time.
    const auto saved_diagnostics = g_robot_status;
    g_robot_status.active = 1;
    g_robot_status.emergency_stop = 0;
    g_robot_status.longitudinal_speed_request = 1;
    g_robot_status.turn_rate_request = -1;
    g_robot_status.updated_ms = 99999;
    CHECK(Robot_GetChassisCommand(&command));
    CHECK(!command.enabled && command.speed_request == 0 && command.turn_request == 0 &&
          command.updated_ms == 1234);
    g_robot_status = saved_diagnostics;
    for (unsigned bad = 0; bad < 8; ++bad) {
        auto invalid = locks;
        if (bad == 0)
            invalid.chassis_stopped = 2;
        if (bad == 1)
            invalid.arm_allows_drive = 2;
        if (bad == 2)
            invalid.arm_allows_fast = 2;
        if (bad == 3)
            invalid.drive_yaw_allowed = 2;
        if (bad == 4)
            invalid.drive_yaw_min_deg = std::numeric_limits<float>::quiet_NaN();
        if (bad == 5)
            invalid.drive_yaw_max_deg = 400;
        if (bad == 6)
            invalid.drive_yaw_max_speed_deg_s = 0;
        if (bad == 7)
            invalid.drive_yaw_min_deg = 21;
        CHECK(!Robot_SetInterlocks(&invalid));
    }
    Robot_Execute(arm, 1234, false);
    CHECK(Robot_GetControlStatus(&s));
    if (kArmConfig.joint_mask == kFourAxisJoints)
        CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    else
        CHECK(s.protection_current_reasons &
              PROTECT_INTERLOCK); // 整车配置无效输入必须立刻撤掉旧许可。
    locks.drive_yaw_allowed = 0;
    locks.drive_yaw_min_deg = 0;
    locks.drive_yaw_max_deg = 0;
    locks.drive_yaw_max_speed_deg_s = 0;
    CHECK(Robot_SetInterlocks(&locks));
    const uint32_t previous_stop_epoch = command.stop_epoch;
    Robot_StopControl();
    Robot_Execute(arm, 1234, false);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
    CHECK(!s.active && s.emergency_stop && s.turn_rate_request == 0);
    CHECK(Robot_GetChassisCommand(&command));
    CHECK(command.stop_epoch != previous_stop_epoch && command.stop_epoch == s.stop_epoch &&
          !command.enabled && command.speed_request == 0 && command.turn_request == 0);
    Status fault{};
    fault.faults = FeedbackTimeout;
    Robot_ReportArmState(fault);
    CHECK(Robot_GetControlStatus(&s));
    CHECK((s.protection_latched_reasons & PROTECT_ARM_FAULT) && s.protection_disabled);
    CHECK(s.stop_chassis && s.emergency_stop && s.reset_chassis_motion);
    CHECK(Robot_GetChassisCommand(&command));
    CHECK(!command.enabled && command.stop && command.brake && command.reset &&
          command.speed_request == 0 && command.turn_request == 0 && command.updated_ms == 1234);
    Robot_ReportArmState(Status{});
    CHECK(Robot_GetChassisCommand(&command) && !command.enabled); // Healthy report cannot restart.
    // Stable debugger command symbol consumes each command once and preserves default disable.
    for (unsigned j = 0; j < kJointCount; ++j)
        if (kArmConfig.joint_mask & (1u << j)) {
            if (j == BigPitch || j == SmallPitch) {
                const uint8_t d[8] = {uint8_t(0x10 + j), 0x80, 0, 0x80, 0x08, 0, 25, 25};
                CHECK(arm.receive(1, arm.feedback_id(j), d, 8, 1234));
            } else {
                const uint8_t d[8] = {3, 232, 0, 0, 0, 0, 25, 0};
                CHECK(arm.receive(1, arm.feedback_id(j), d, 8, 1234));
            }
        }
    g_robot_debug_command = ROBOT_DEBUG_CAPTURE_ZERO;
    Robot_Execute(arm, 1234, false);
    CHECK(g_robot_debug_command == ROBOT_DEBUG_IDLE && arm.status().last_result == Result::Ok);
    CHECK(arm.status().enabled_mask == 0);
    for (unsigned j = 0; j < kJointCount; ++j)
        CHECK(arm.status().joint[j].calibrated == bool(kArmConfig.joint_mask & (1u << j)));
    arm.transport_fault();
    CHECK(arm.status().faults & TransportError);
    g_robot_debug_command = ROBOT_DEBUG_CLEAR_FAULTS;
    Robot_Execute(arm, 1234, false);
    CHECK(g_robot_debug_command == ROBOT_DEBUG_IDLE && arm.status().last_result == Result::Ok);
    CHECK(arm.status().faults == 0 && arm.status().enabled_mask == 0);
    g_robot_debug_command = ROBOT_DEBUG_REQUEST_CONTROL;
    Robot_Execute(arm, 1234, false);
    CHECK(g_robot_debug_command == ROBOT_DEBUG_IDLE && arm.status().enabled_mask == 0);
    g_robot_debug_command = ROBOT_DEBUG_STOP;
    Robot_Execute(arm, 1234, false);
    CHECK(g_robot_debug_command == ROBOT_DEBUG_IDLE);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH
    {
        const uint32_t vehicle_mask =
            PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED | PROTECT_VEHICLE_STATE | PROTECT_POWER;
        const uint32_t causes[] = {PROTECT_CHASSIS, PROTECT_IMU, PROTECT_TIPPED,
                                   PROTECT_VEHICLE_STATE, PROTECT_POWER};
        for (uint32_t cause : causes) {
            vehicle_reasons = cause;
            Robot_Execute(arm, 1234, false);
            CHECK(Robot_GetControlStatus(&s));
            CHECK((s.protection_current_reasons & vehicle_mask) == cause);
            CHECK((s.protection_latched_reasons & cause) && s.protection_disabled);
            CHECK(s.emergency_stop && s.stop_chassis && !s.active &&
                  arm.status().enabled_mask == 0);
        }
        vehicle_reasons = vehicle_mask;
        Robot_Execute(arm, 1234, false);
        CHECK(Robot_GetControlStatus(&s));
        CHECK((s.protection_current_reasons & vehicle_mask) == vehicle_mask);
        vehicle_reasons = 0;
        Robot_Execute(arm, 1234, false);
        CHECK(Robot_GetControlStatus(&s));
        CHECK(!(s.protection_current_reasons & vehicle_mask));
        CHECK(s.protection_disabled && !s.active); // Healthy source alone cannot restart.
        CHECK(s.protection_latched_reasons & PROTECT_VEHICLE_STATE);
        CHECK(calibration_revocations == 0); // Default confirmations never reach enabling.
        CHECK(vehicle_reads > 0);
    }
#else
    CHECK(vehicle_reads == 0); // Teammate's four-axis bench remains independent.
#endif
    if (kArmConfig.joint_mask == kAllJoints) {
        // Default confirmations stay false. A mode change clears the target once;
        // a zero target waits for a fresh nonzero value and preserves the hold.
        auto local = kArmConfig;
        local.hardware_confirmed = true;
        local.pitch.mapping_confirmed = true;
        ArmControl held(local);
        uint32_t now = 1238;
        fresh_inputs(now);
        CHECK(feed_all(held, now));
        CHECK(held.capture_zero(kFourAxisJoints, now) == Result::Ok);
        CHECK(held.enable(kFourAxisJoints, now) == Result::Ok);
        float hold_target[kJointCount] = {0, 3, -1, 2, 1};
        CHECK(held.set_targets(kFourAxisJoints, hold_target, now) == Result::Ok);
        robot_tune_mode = ROBOT_TUNE_LEFT_CURRENT;
        tune_current_ma = 40;
        tune_speed_rpm = 50;
        tune_yaw_speed_deg_s = 1;
        tune_yaw_target_deg = 2;
        Robot_Execute(held, now, false);
        CHECK(g_robot_tune_status.mode == ROBOT_TUNE_LEFT_CURRENT &&
              !g_robot_tune_status.ready);
        CHECK(tune_current_ma == 0 && tune_speed_rpm == 0 && tune_yaw_speed_deg_s == 0 &&
              tune_yaw_target_deg == held.status().joint[Yaw].position_deg);
        CHECK(held.status().enabled_mask == kFourAxisJoints);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.tune_value == 0);
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(held, now));
        Robot_Execute(held, now, false);
        CHECK(!g_robot_tune_status.ready && !g_robot_tune_status.blocked);
        const auto entry_epoch = g_robot_tune_status.epoch;
        for (unsigned n = 0; n < 80; ++n) {
            now += 4;
            fresh_inputs(now);
            CHECK(feed_all(held, now));
            Robot_Execute(held, now, false);
            CHECK(!g_robot_tune_status.ready && !g_robot_tune_status.blocked &&
                  g_robot_tune_status.epoch == entry_epoch);
            CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.tune_value == 0);
        }
        for (unsigned j = BigPitch; j < kJointCount; ++j)
            CHECK(held.status().joint[j].target_deg == hold_target[j]);
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(held, now));
        robot_tune_mode = ROBOT_TUNE_RIGHT_SPEED;
        tune_current_ma = 80;
        tune_speed_rpm = 90;
        tune_yaw_speed_deg_s = 2;
        Robot_Execute(held, now, false);
        CHECK(g_robot_tune_status.epoch != entry_epoch && !g_robot_tune_status.ready);
        CHECK(tune_current_ma == 0 && tune_speed_rpm == 0 && tune_yaw_speed_deg_s == 0);
        CHECK(held.status().enabled_mask == kFourAxisJoints);
        for (unsigned j = BigPitch; j < kJointCount; ++j)
            CHECK(held.status().joint[j].target_deg == hold_target[j]);

        // A mode change and global STOP in the same cycle cannot enable either
        // the newly selected object or any previously held mechanical arm axis.
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(held, now));
        robot_tune_mode = ROBOT_TUNE_YAW_SPEED;
        tune_yaw_speed_deg_s = 1;
        Robot_StopControl();
        Robot_Execute(held, now, false);
        CHECK(!held.status().enabled_mask && !g_robot_tune_status.ready &&
              g_robot_tune_status.blocked);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.tune_value == 0);
        CHECK(g_robot_status.emergency_stop &&
              (g_robot_status.protection_current_reasons & PROTECT_STOP_REQUEST));
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(held, now));
        Robot_Execute(held, now, false);
        CHECK(!held.status().enabled_mask && !g_robot_tune_status.ready);

        // In a Yaw tuning selection, the zero command calibrates Yaw only.
        ArmControl yaw_only(cfg);
        const uint8_t feedback[8] = {3, 232, 0, 0, 0, 0, 25, 0};
        now += 4;
        fresh_inputs(now);
        CHECK(yaw_only.receive(1, yaw_only.feedback_id(Yaw), feedback, 8, now));
        g_robot_debug_command = ROBOT_DEBUG_CAPTURE_ZERO;
        Robot_Execute(yaw_only, now, false);
        CHECK(g_robot_debug_command == ROBOT_DEBUG_IDLE && yaw_only.status().joint[Yaw].calibrated);
        for (unsigned j = BigPitch; j < kJointCount; ++j)
            CHECK(!yaw_only.status().joint[j].calibrated);
        CHECK(!yaw_only.status().enabled_mask);
        now += 4;
        fresh_inputs(now);
        CHECK(yaw_only.receive(1, yaw_only.feedback_id(Yaw), feedback, 8, now));
        robot_tune_mode = ROBOT_TUNE_NORMAL;
        Robot_Execute(yaw_only, now, false);
        CHECK(g_robot_tune_status.mode == ROBOT_TUNE_NORMAL && !g_robot_tune_status.ready &&
              !g_robot_tune_status.blocked);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.tune_mode == 0);
        CHECK(!yaw_only.status().enabled_mask);
        now += 4;
        fresh_inputs(now);
        robot_tune_mode = ROBOT_TUNE_LEFT_SPEED;
        g_dr16_status.frame.channel[kOperatorConfig.left_y_channel] = -660;
        g_dr16_status.frame.channel[kOperatorConfig.left_x_channel] = 660;
        g_dr16_status.frame.channel[kOperatorConfig.right_y_channel] = 39;
        g_dr16_status.frame.channel[kOperatorConfig.right_x_channel] = -330;
        g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_DOWN;
        Robot_Execute(yaw_only, now, true);
        CHECK(Robot_GetControlStatus(&s));
        CHECK(s.remote_axis[0] == -1 && s.remote_axis[1] == 1 && s.remote_axis[2] == 0 &&
              std::fabs(s.remote_axis[3] + (.5f - .06f) / .94f) < .0001f);
        CHECK(s.remote_switch[0] == DR16_SWITCH_DOWN && s.remote_switch[1] == DR16_SWITCH_DOWN);
        CHECK(!s.active && !yaw_only.status().enabled_mask &&
              Robot_GetChassisCommand(&command) && !command.enabled && command.tune_value == 0);
        robot_tune_mode = 0;
        test_hal_tick = 1234;
    }
    if (kArmConfig.joint_mask == kAllJoints) {
        // Independent chassis path: 0档 + left switch up must drive without the
        // arm being active; STOP latches; an arm fault must not clear the command.
        kChassisIndependent = true;
        robot_tune_mode = 0;
        uint32_t now = 1240;
        fresh_inputs(now);
        g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        g_dr16_status.frame.channel[kOperatorConfig.left_y_channel] = 660;
        ArmControl independent_arm(kArmConfig);
        CHECK(feed_all(independent_arm, now));
        auto run = [&](uint32_t t, bool left_up, int left_y) {
            fresh_inputs(t);
            if (left_up)
                g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            g_dr16_status.frame.channel[kOperatorConfig.left_y_channel] = int16_t(left_y);
            Robot_Execute(independent_arm, t, false);
            Robot_GetChassisCommand(&command);
            Robot_GetControlStatus(&s);
        };
        Robot_RequestControl(); // Clear any STOP latch left by earlier blocks.
        for (unsigned i = 0; i <= 76; ++i) { // 304ms neutral arming window.
            now += 4;
            run(now, true, 0);
        }
        CHECK(Robot_GetControlStatus(&s) && s.independent_armed);
        CHECK(Robot_GetChassisCommand(&command));
        CHECK(command.independent && command.enabled && command.speed_request == 0);
        now += 4;
        run(now, true, 660);
        CHECK(command.enabled && command.speed_request > .9f && command.turn_request == 0);
        CHECK(Robot_GetControlStatus(&s));
        CHECK(!s.emergency_stop && !s.stop_chassis);
        CHECK(s.longitudinal_speed_request > .9f);
        ChassisControl direct(kChassisConfig);
        WheelFeedback direct_wheels[2]{};
        for (auto &wheel : direct_wheels) {
            wheel.received_ms = now;
            wheel.frames = 1;
            wheel.temperature_c = 25;
        }
        direct.step(now, direct_wheels, command);
        CHECK(direct.status().enabled && direct.status().target_rpm[0] > 0);
        CHECK(direct.status().candidate_ma[0] > 0);

        // An arm fault is arm-side only; the independent chassis command stays.
        engineer::Status fault{};
        fault.faults = engineer::FeedbackTimeout;
        Robot_ReportArmState(fault);
        CHECK(Robot_GetChassisCommand(&command) && command.independent && command.enabled);
        CHECK(Robot_GetControlStatus(&s));
        CHECK(!s.emergency_stop && !s.stop_chassis);

        // Explicit STOP latches and must not auto-resume from the held stick.
        Robot_StopControl();
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.stop);
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.stop);

        // Command 4 clears the latch, but a held stick must not re-arm.
        Robot_RequestControl();
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled);
        for (unsigned i = 0; i <= 76; ++i) {
            now += 4;
            run(now, true, 0);
        }
        CHECK(command.enabled && command.independent);
        now += 4;
        run(now, true, 660);
        CHECK(command.enabled && command.speed_request > .9f);

        // Tip-over stops and latches; a held stick cannot re-arm after clearing.
        vehicle_reasons = PROTECT_TIPPED;
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled && command.stop);
        CHECK(Robot_GetControlStatus(&s) && s.independent_stop_latched);
        vehicle_reasons = 0;
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled);
        Robot_RequestControl();
        now += 4;
        run(now, true, 660);
        CHECK(Robot_GetChassisCommand(&command) && !command.enabled);
        for (unsigned i = 0; i <= 76; ++i) {
            now += 4;
            run(now, true, 0);
        }
        CHECK(command.enabled && command.independent);

        // Consume the request and leave a clean latch state for later tests.
        Robot_RequestControl();
        now += 4;
        fresh_inputs(now);
        Robot_Execute(independent_arm, now, false);
    }
    if (kArmConfig.joint_mask == kAllJoints) {
        // Independent chassis arming does not require the right switch at MID.
        kChassisIndependent = true;
        robot_tune_mode = 0;
        uint32_t now = 1500;
        auto local = cfg;
        ArmControl chassis_arm(local);
        auto step = [&](int right_switch) {
            now += 4;
            fresh_inputs(now);
            g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            g_dr16_status.frame.switches[kOperatorConfig.right_switch_index] = right_switch;
            Robot_Execute(chassis_arm, now, false);
            Robot_GetControlStatus(&s);
        };
        for (unsigned i = 0; i <= 76; ++i)
            step(DR16_SWITCH_UP);
        CHECK(s.independent_armed);
        CHECK(Robot_GetChassisCommand(&command) && command.enabled && command.independent);
        // Leave the independent chassis disarmed for the following Yaw test.
        now += 4;
        fresh_inputs(now);
        g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        Robot_Execute(chassis_arm, now, false);
    }
    if (kArmConfig.joint_mask == kAllJoints) {
        // Power stop requests braking with the chassis command enabled, but the
        // independent drive stays disarmed and requires neutral re-arm.
        kChassisIndependent = true;
        robot_tune_mode = 0;
        uint32_t now = 1700;
        auto local = cfg;
        ArmControl chassis_arm(local);
        auto step = [&](bool power_stop) {
            now += 4;
            fresh_inputs(now);
            test_wheels.power_confirmed = 1;
            test_wheels.power_session_allowed = 1;
            test_wheels.power_stop_requested = power_stop;
            g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            Robot_Execute(chassis_arm, now, false);
            Robot_GetControlStatus(&s);
            Robot_GetChassisCommand(&command);
        };
        for (unsigned i = 0; i <= 76; ++i)
            step(false);
        CHECK(s.independent_armed);
        step(true);
        CHECK(!s.independent_armed);
        CHECK(command.enabled && command.independent);
        CHECK(command.stop && command.brake);
        CHECK(command.speed_request == 0 && command.turn_request == 0);
        now += 4;
        fresh_inputs(now);
        g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        Robot_Execute(chassis_arm, now, false);
    }
    if (kArmConfig.joint_mask == kAllJoints) {
        // Yaw direct: left horizontal moves Yaw even when only Yaw is enabled.
        kChassisIndependent = true;
        robot_tune_mode = 0;
        uint32_t now = 1600;
        auto local = cfg;
        local.hardware_confirmed = true;
        local.pitch.mapping_confirmed = true;
        local.joint[Yaw].fixed_zero_encoder = -1; // Manual capture in this block.
        ArmControl yaw_direct(local);
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(yaw_direct, now));
        robot_tune_mode = ROBOT_TUNE_YAW_POSITION;
        g_robot_debug_command = ROBOT_DEBUG_CAPTURE_ZERO;
        Robot_Execute(yaw_direct, now, false);
        CHECK(yaw_direct.status().joint[Yaw].calibrated);
        robot_tune_mode = ROBOT_TUNE_NORMAL;
        now += 4;
        fresh_inputs(now);
        CHECK(feed_all(yaw_direct, now));
        g_robot_debug_command = ROBOT_DEBUG_ENABLE_YAW;
        Robot_Execute(yaw_direct, now, false);
        CHECK(yaw_direct.status().joint[Yaw].enabled);
        Robot_RequestControl();
        auto yaw_run = [&](uint32_t t, int left_x) {
            fresh_inputs(t);
            feed_all(yaw_direct, t);
            g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            g_dr16_status.frame.channel[kOperatorConfig.left_x_channel] = int16_t(left_x);
            Robot_Execute(yaw_direct, t, false);
            Robot_GetControlStatus(&s);
        };
        for (unsigned i = 0; i <= 76; ++i) {
            now += 4;
            yaw_run(now, 0);
        }
        CHECK(s.independent_armed && s.yaw_direct_active);
        now += 4;
        yaw_run(now, 660);
        CHECK(s.yaw_direct_active);
        CHECK(s.yaw_direct_target_deg < 0); // Current direct-Yaw sign is -stick.
        CHECK(yaw_direct.status().joint[Yaw].target_deg < 0);

        // Fixed-zero yaw joins the drive switch without command 5 or whole-arm confirmation.
        local.hardware_confirmed = false;
        local.joint[Yaw].fixed_zero_encoder = 1000;
        ArmControl automatic(local);
        auto auto_run = [&](int left_switch, int left_x = 0) {
            now += 4;
            fresh_inputs(now);
            feed_all(automatic, now);
            g_dr16_status.frame.switches[kOperatorConfig.left_switch_index] = left_switch;
            g_dr16_status.frame.channel[kOperatorConfig.left_x_channel] = int16_t(left_x);
            Robot_Execute(automatic, now, false);
            Robot_GetControlStatus(&s);
        };
        auto_run(DR16_SWITCH_DOWN);
        for (unsigned i = 0; i <= 76; ++i)
            auto_run(DR16_SWITCH_UP);
        CHECK(s.independent_armed && s.yaw_direct_active);
        CHECK(automatic.status().joint[Yaw].enabled);
        CHECK(automatic.status().enabled_mask == (1u << Yaw));
        auto_run(DR16_SWITCH_UP, 660);
        CHECK(automatic.status().joint[Yaw].target_deg < 0);
        auto_run(DR16_SWITCH_DOWN);
        CHECK(!automatic.status().joint[Yaw].enabled && !s.yaw_direct_active);
    }
    if (kArmConfig.joint_mask == kAllJoints) {
    uint32_t now = 2000;
    { // Command 5 enables Yaw only; whole-arm hardware confirmation is not required.
        auto local = cfg;
        local.hardware_confirmed = false;
        ArmControl yaw_cmd(local);
        const uint8_t yaw_feedback[8] = {3, 232, 0, 0, 0, 0, 25, 0};
        now += 4;
        fresh_inputs(now);
        CHECK(yaw_cmd.receive(1, yaw_cmd.feedback_id(Yaw), yaw_feedback, 8, now));
        robot_tune_mode = ROBOT_TUNE_YAW_POSITION;
        g_robot_debug_command = ROBOT_DEBUG_CAPTURE_ZERO;
        Robot_Execute(yaw_cmd, now, false);
        CHECK(yaw_cmd.status().joint[Yaw].calibrated);
        robot_tune_mode = ROBOT_TUNE_NORMAL;
        g_robot_debug_command = ROBOT_DEBUG_ENABLE_YAW;
        Robot_Execute(yaw_cmd, now, false);
        CHECK(yaw_cmd.status().joint[Yaw].enabled);
        CHECK(yaw_cmd.status().enabled_mask == (1u << Yaw));
        g_robot_debug_command = ROBOT_DEBUG_DISABLE_YAW;
        Robot_Execute(yaw_cmd, now, false);
        CHECK(!yaw_cmd.status().joint[Yaw].enabled);
    }
    { // Mode 6 becomes ready with only Yaw CAN healthy; chassis/other axes ignored.
        auto local = cfg;
        local.hardware_confirmed = false;
        ArmControl yaw_tune(local);
        const uint8_t yaw_feedback[8] = {3, 232, 0, 0, 0, 0, 25, 0};
        now += 4;
        fresh_inputs(now);
        CHECK(yaw_tune.receive(1, yaw_tune.feedback_id(Yaw), yaw_feedback, 8, now));
        robot_tune_mode = ROBOT_TUNE_YAW_POSITION;
        g_robot_debug_command = ROBOT_DEBUG_CAPTURE_ZERO;
        Robot_Execute(yaw_tune, now, false);
        CHECK(yaw_tune.status().joint[Yaw].calibrated);
        robot_tune_mode = ROBOT_TUNE_NORMAL;
        Robot_Execute(yaw_tune, now, false);
        robot_tune_mode = ROBOT_TUNE_YAW_POSITION;
        tune_yaw_target_deg = 3;
        for (unsigned i = 0; i < 80; ++i) {
            now += 4;
            fresh_inputs(now);
            test_wheels = {}; // Deliberately unavailable; Yaw mode must ignore it.
            CHECK(yaw_tune.receive(1, yaw_tune.feedback_id(Yaw), yaw_feedback, 8, now));
            Robot_Execute(yaw_tune, now, false);
        }
        CHECK(g_robot_tune_status.ready);
        CHECK(yaw_tune.status().joint[Yaw].enabled);
        CHECK(yaw_tune.status().enabled_mask == (1u << Yaw));
        robot_tune_mode = ROBOT_TUNE_NORMAL;
        g_robot_debug_command = ROBOT_DEBUG_DISABLE_YAW;
        Robot_Execute(yaw_tune, now, false);
        CHECK(!yaw_tune.status().joint[Yaw].enabled);
    }
    }
    kChassisIndependent = true;
    return 0;
}
