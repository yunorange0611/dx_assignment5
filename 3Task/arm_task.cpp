#include "arm_task.h"
#include "Arm/arm_control.h"
#include "CAN_bsp.h"
#include "dr16_bsp.h"
#include "robot_control_internal.h"
#include "robot_control.h"
#include "task_ins.h"
#include "bsp_dwt.h"
#include <cstring>
#include <optional>

extern "C" {
ArmStatus g_arm_status{};
volatile uint32_t g_arm_stack_free_bytes = 0;
volatile uint32_t g_stack_overflow_latched = 0;
char g_stack_overflow_task[16]{};
YawFeedbackDiag g_yaw_feedback_diag{};
volatile uint32_t g_yaw_capture_command = 0;
YawRawCapture g_yaw_raw_capture{};
volatile ArmTuneRequest g_arm_tune_request[ARM_JOINT_COUNT]{};
ArmTuneStatus g_arm_tune_status[ARM_JOINT_COUNT]{};
#if ARM_MOTOR_JOG_TEST
volatile int32_t g_arm_jog_command = 0;
#endif
}

namespace {
using namespace engineer;
enum Kind : uint8_t { Zero, Enable, Clear, Target };
struct Command {
    Kind kind;
    uint8_t mask;
    float q[kJointCount];
    uint32_t submitted_ms;
};
struct Received {
    uint8_t bus, len;
    uint16_t id;
    uint32_t ms;
    uint8_t data[8];
};
osMessageQueueId_t commands = nullptr, feedback = nullptr;
volatile bool stop_requested = false;
volatile uint32_t dropped_rx = 0;
#if ARM_MOTOR_JOG_TEST
bool jog_permitted(uint32_t now) {
    DR16Status remote{};
    DR16_GetStatus(&remote);
    const auto &c = kOperatorConfig;
    if (!remote.valid || remote.consecutive_frames < 3 ||
        uint32_t(now - remote.received_ms) > c.remote_timeout_ms ||
        c.left_switch_index >= 2 || c.right_switch_index >= 2 ||
        remote.frame.switches[c.left_switch_index] != DR16_SWITCH_MID ||
        remote.frame.switches[c.right_switch_index] != DR16_SWITCH_MID)
        return false;
    const float neutral = c.deadband * 660.0f;
    for (unsigned i = 0; i < 4; ++i)
        if (remote.frame.channel[i] < -neutral || remote.frame.channel[i] > neutral)
            return false;
    return true;
}
#else
void apply_tune_requests(ArmControl &arm, uint32_t now) {
    // Retry pending FreeMASTER edits at 50 Hz.
    static uint32_t last_poll_ms = 0;
    if (uint32_t(now - last_poll_ms) < 20)
        return;
    last_poll_ms = now;

    for (unsigned joint = BigPitch; joint <= Gripper; ++joint) {
        const auto &request = g_arm_tune_request[joint];
        auto &status = g_arm_tune_status[joint];
        const JointTuning tuning{request.position_kp, request.velocity_kp, request.velocity_ki,
                                 request.effort_limit, request.torque_bias_nm,
                                 request.gravity_gain_nm, request.gravity_offset_deg,
                                 request.up_ff_nm, request.down_ff_nm};
        if (tuning.position_kp == status.position_kp && tuning.velocity_kp == status.velocity_kp &&
            tuning.velocity_ki == status.velocity_ki && tuning.effort_limit == status.effort_limit &&
            tuning.torque_bias_nm == status.torque_bias_nm &&
            tuning.gravity_gain_nm == status.gravity_gain_nm &&
            tuning.gravity_offset_deg == status.gravity_offset_deg &&
            tuning.up_ff_nm == status.up_ff_nm && tuning.down_ff_nm == status.down_ff_nm) {
            status.result = uint32_t(Result::Ok);
            continue;
        }
        const auto result = Robot_ApplyArmTuning(arm, joint, tuning, now);
        auto next = status;
        next.result = uint32_t(result);
        if (result == Result::Ok)
            next = {uint32_t(result), tuning.position_kp, tuning.velocity_kp,
                    tuning.velocity_ki, tuning.effort_limit, tuning.torque_bias_nm,
                    tuning.gravity_gain_nm, tuning.gravity_offset_deg,
                    tuning.up_ff_nm, tuning.down_ff_nm};
        taskENTER_CRITICAL();
        status = next;
        taskEXIT_CRITICAL();
    }
}
#endif
// RX对象在任务栈上常驻；回调只复制到队列，不在ISR里做闭环/浮点解算。
void receive_callback(can_device_receive *dev, uint8_t *data) {
    if (!feedback)
        return;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(kArmConfig.joint_mask & (1u << i)))
            continue;
        const auto &c = kArmConfig.joint[i];
        auto *handle = c.bus == 1 ? &hcan1 : &hcan2;
        uint16_t id = c.motor == Motor::Dm4310Mit
                          ? c.feedback_id
                          : uint16_t((c.motor == Motor::M2006 ? 0x200 : 0x204) + c.motor_id);
        if (!dev->matches(handle, id))
            continue;
        Received f{};
        f.bus = c.bus;
        f.id = id;
        f.len = dev->rx_dlc;
        f.ms = HAL_GetTick();
        std::memcpy(f.data, data, 8);
        if (osMessageQueuePut(feedback, &f, 0, 0) != osOK)
            ++dropped_rx;
        break;
    }
}
int submit(Kind kind, uint8_t mask, const float *q = nullptr) {
    if (!commands)
        return 0;
    Command c{};
    c.kind = kind;
    c.mask = mask;
    c.submitted_ms = HAL_GetTick();
    if (q)
        std::memcpy(c.q, q, sizeof(c.q));
    return osMessageQueuePut(commands, &c, 0, 0) == osOK;
}
// Single-owner observer, never feeds any value back into ArmControl.
void diagnose_yaw(const ArmControl &arm, const Received &f, bool accepted) {
    if (f.bus != kArmConfig.joint[Yaw].bus || f.id != arm.feedback_id(Yaw))
        return;
    // Capture the queue's original raw bytes and receive timestamp together.
    // Once frozen, no task writes samples until another explicit arm command.
    const uint32_t command = g_yaw_capture_command;
    if (command) {
        g_yaw_capture_command = 0;
        if (command == 1 && !arm.status().enabled_mask) {
            g_yaw_raw_capture.count = 0;
            ++g_yaw_raw_capture.capture_seq;
            g_yaw_raw_capture.state = 1;
        } else {
            g_yaw_raw_capture.state = 3;
        }
    }
    if (g_yaw_raw_capture.state == 1) {
        if (arm.status().enabled_mask) {
            g_yaw_raw_capture.state = 3;
        } else {
            auto &sample = g_yaw_raw_capture.samples[g_yaw_raw_capture.count];
            sample.received_ms = f.ms;
            sample.can_id = f.id; sample.bus = f.bus; sample.dlc = f.len;
            std::memcpy(sample.data, f.data, 8);
            if (++g_yaw_raw_capture.count == 500)
                g_yaw_raw_capture.state = 2;
        }
    }
    static bool have = false;
    static uint32_t last_ms = 0, start_ms = 0, samples = 0, zeros = 0;
    static int last_encoder = 0;
    static int64_t ticks = 0, rpm_sum = 0;
    static int rpm_min = 0, rpm_max = 0;
    auto &d = g_yaw_feedback_diag;
    d.updated_ms = f.ms;
    d.bus = f.bus; d.can_id = f.id; d.dlc = f.len;
    std::memcpy(d.raw_data, f.data, 8);
    if (!accepted || f.len != 8) {
        ++d.invalid_frames; have = false; d.window_valid = 0; return;
    }
    d.raw_encoder = (uint16_t(f.data[0]) << 8) | f.data[1];
    d.raw_rpm = int16_t((uint16_t(f.data[2]) << 8) | f.data[3]);
    d.raw_effort = int16_t((uint16_t(f.data[4]) << 8) | f.data[5]);
    ++d.frames;
    if (!d.raw_rpm) ++d.zero_rpm_frames;
    d.parsed_speed_deg_s = arm.status().joint[Yaw].velocity_deg_s;
    d.parsed_position_deg = arm.status().joint[Yaw].position_deg;
    JointTuning tuning{};
    if (arm.get_tuning(Yaw, tuning)) {
        d.velocity_kp = tuning.velocity_kp; d.velocity_ki = tuning.velocity_ki;
        d.position_kp = tuning.position_kp;
    }
    const uint32_t gap = f.ms - last_ms;
    if (have && gap > d.max_gap_ms) d.max_gap_ms = gap;
    if (have && gap > kArmConfig.dji_unwrap_gap_ms) {
        ++d.gap_resets; have = false; d.window_valid = 0;
    }
    if (!have) {
        have = true; last_ms = start_ms = f.ms; last_encoder = d.raw_encoder;
        samples = zeros = 0; ticks = rpm_sum = 0;
        rpm_min = rpm_max = d.raw_rpm;
        return;
    }
    int delta = int(d.raw_encoder) - last_encoder;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    ticks += delta;
    rpm_sum += d.raw_rpm; ++samples;
    if (!d.raw_rpm) ++zeros;
    if (d.raw_rpm < rpm_min) rpm_min = d.raw_rpm;
    if (d.raw_rpm > rpm_max) rpm_max = d.raw_rpm;
    last_encoder = d.raw_encoder; last_ms = f.ms;
    const uint32_t elapsed = f.ms - start_ms;
    if (elapsed >= 500 && samples) {
        const auto &c = kArmConfig.joint[Yaw];
        const float scale = float(c.direction) / c.feedback_per_joint;
        d.encoder_speed_deg_s = float(ticks) * (360.0f / 8192.0f) * 1000 / elapsed * scale;
        d.rpm_mean_deg_s = float(rpm_sum) / samples * 6 * scale;
        const float lo = rpm_min * 6 * scale, hi = rpm_max * 6 * scale;
        d.rpm_min_deg_s = lo < hi ? lo : hi; d.rpm_max_deg_s = lo > hi ? lo : hi;
        d.zero_rpm_fraction = float(zeros) / samples;
        d.window_ms = elapsed; d.window_frames = samples;
        d.window_valid = 1; ++d.window_seq;
        start_ms = f.ms; samples = zeros = 0; ticks = rpm_sum = 0;
        rpm_min = rpm_max = d.raw_rpm;
    }
}

void publish(const ArmControl &arm, uint32_t now) {
    ArmStatus next{};
    const auto &s = arm.status();
    next.faults = s.faults;
    next.enabled_mask = s.enabled_mask;
    next.updated_ms = now;
    next.yaw_limit_version = arm.yaw_limits().version;
    next.last_result = uint8_t(s.last_result);
    next.yaw_friction_ff_applied = arm.yaw_friction_feedforward_applied();
    next.ready = 1;
    next.configuration_valid = s.configuration_valid;
    next.rx_dropped = dropped_rx;
    for (unsigned b = 0; b < 2; ++b)
        next.tx_errors[b] = can_tx_error_count[b];
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &j = s.joint[i];
        auto &d = next.joint[i];
        d.position_deg = j.position_deg;
        d.velocity_deg_s = j.velocity_deg_s;
        d.target_deg = j.target_deg;
        d.reference_deg = j.reference_deg;
        d.effort = j.effort;
        d.drive_position_deg = j.drive_position_deg;
        d.drive_velocity_deg_s = j.drive_velocity_deg_s;
        d.drive_target_deg = j.drive_target_deg;
        d.drive_reference_deg = j.drive_reference_deg;
        d.drive_reference_velocity_deg_s = j.drive_reference_velocity_deg_s;
        d.feedback_effort = j.motor_effort_feedback;
        d.temperature_c = j.temperature_c;
        d.velocity_target_deg_s = j.velocity_target_deg_s;
        d.feedforward_effort = j.feedforward_effort;
        d.motion_feedforward_effort = j.motion_feedforward_effort;
        d.motion_ff_direction = j.motion_ff_direction;
        d.received = j.received;
        d.feedback_age_ms = now - j.last_feedback_ms;
        d.faults = j.faults;
        d.online = j.online;
        d.calibrated = j.calibrated;
        d.enabled = j.enabled;
        d.motor_state = j.motor_state;
        d.raw_encoder = j.raw_encoder;
        d.fixed_zero_encoder = j.fixed_zero_encoder;
        d.homing = j.homing;
        d.can_velocity_deg_s = j.can_velocity_deg_s;
        d.encoder_velocity_deg_s = j.encoder_velocity_deg_s;
        d.velocity_window_ms = j.velocity_window_ms;
    }
    const YawPidSnapshot yp = arm.yaw_pid_snapshot();
    auto &yd = g_yaw_feedback_diag;
    yd.yaw_pid_dt_s = yp.dt_s;
    yd.yaw_pos_kp = yp.position_kp;
    yd.yaw_pos_ki = yp.position_ki;
    yd.yaw_pos_kd = yp.position_kd;
    yd.yaw_pos_error = yp.position_error;
    yd.yaw_pos_max_output = yp.position_max_output;
    yd.yaw_pos_max_i_error = yp.position_max_i_error;
    yd.yaw_pos_max_sigma = yp.position_max_sigma;
    yd.yaw_pos_p_term = yp.position_p;
    yd.yaw_pos_i_term = yp.position_i;
    yd.yaw_pos_d_term = yp.position_d;
    yd.yaw_pos_sigma = yp.position_sigma;
    yd.yaw_pos_output = yp.position_output;
    yd.yaw_spd_kp = yp.velocity_kp;
    yd.yaw_spd_ki = yp.velocity_ki;
    yd.yaw_spd_kd = yp.velocity_kd;
    yd.yaw_spd_error = yp.velocity_error;
    yd.yaw_spd_max_output = yp.velocity_max_output;
    yd.yaw_spd_max_i_error = yp.velocity_max_i_error;
    yd.yaw_spd_max_sigma = yp.velocity_max_sigma;
    yd.yaw_spd_p_term = yp.velocity_p;
    yd.yaw_spd_i_term = yp.velocity_i;
    yd.yaw_spd_d_term = yp.velocity_d;
    yd.yaw_spd_sigma = yp.velocity_sigma;
    yd.yaw_spd_prelimit = yp.velocity_prelimit;
    yd.yaw_spd_output = yp.velocity_output;
    taskENTER_CRITICAL();
    g_arm_status = next;
    taskEXIT_CRITICAL();
}
} // namespace

extern "C" int Arm_CaptureZero(uint8_t mask) {
    return submit(Zero, mask);
}
extern "C" int Arm_Enable(uint8_t mask) {
    return submit(Enable, mask);
}
extern "C" int Arm_ClearFaults() {
    return submit(Clear, 0);
}
extern "C" int Arm_SetTargets(uint8_t mask, const float *q) {
    return q ? submit(Target, mask, q) : 0;
}
extern "C" int Arm_Disable() {
    stop_requested = true;
    return 1;
}
extern "C" int Arm_GetStatus(ArmStatus *s) {
    if (!s)
        return 0;
    taskENTER_CRITICAL();
    *s = g_arm_status;
    taskEXIT_CRITICAL();
    return s->ready;
}

extern "C" void arm_control_task(void *argument) {
    DR16_Start();
    commands = osMessageQueueNew(16, sizeof(Command), nullptr);
    feedback = osMessageQueueNew(64, sizeof(Received), nullptr);
    if (!commands || !feedback || osKernelGetTickFreq() != 1000)
        Error_Handler();
    if (!BSP_DWT_Init(HAL_RCC_GetHCLKFreq()))
        Error_Handler();
    static ArmControl arm(kArmConfig); // Single owner; large history must not live on task stack.
#if !ARM_MOTOR_JOG_TEST
    for (unsigned joint = BigPitch; joint <= Gripper; ++joint) {
        JointTuning initial{};
        if (!arm.get_tuning(joint, initial))
            continue;
        auto &request = g_arm_tune_request[joint];
        request.position_kp = initial.position_kp;
        request.velocity_kp = initial.velocity_kp;
        request.velocity_ki = initial.velocity_ki;
        request.effort_limit = initial.effort_limit;
        request.torque_bias_nm = initial.torque_bias_nm;
        request.gravity_gain_nm = initial.gravity_gain_nm;
        request.gravity_offset_deg = initial.gravity_offset_deg;
        request.up_ff_nm = initial.up_ff_nm;
        request.down_ff_nm = initial.down_ff_nm;
        g_arm_tune_status[joint] = {0, initial.position_kp, initial.velocity_kp,
                                   initial.velocity_ki, initial.effort_limit, initial.torque_bias_nm,
                                   initial.gravity_gain_nm, initial.gravity_offset_deg,
                                   initial.up_ff_nm, initial.down_ff_nm};
    }
    uint8_t owned_buses = 0;
    const uint32_t period = kArmConfig.control_period_ms;
    if (!arm.status().configuration_valid) {
        publish(arm, HAL_GetTick());
        for (;;) {
            DR16_Service(); // Receiving/recovery must also work with invalid motor config.
            osDelay(4); // 错误配置禁止注册过滤器/发送电机帧
        }
    }
    for (unsigned i = 0; i < kJointCount; ++i)
        if (kArmConfig.joint_mask & (1u << i))
            owned_buses |= 1u << (kArmConfig.joint[i].bus - 1);
    // 构造时注册硬件过滤器。无电机反馈也不会阻塞初始化。
    std::optional<can_device_receive> rx[kJointCount]; // 就地构造，无堆分配；台架不注册yaw
    for (unsigned i = 0; i < kJointCount; ++i)
        if (kArmConfig.joint_mask & (1u << i))
            rx[i].emplace(kArmConfig.joint[i].bus == 1 ? &hcan1 : &hcan2, arm.feedback_id(i),
                          receive_callback);
    uint32_t previous_errors[2] = {0, 0};
    uint32_t previous_drops = 0;
    uint32_t tick = osKernelGetTickCount();
    for (;;) {
        const uint8_t previous_enabled = arm.status().enabled_mask;
        DR16_Service();
        Received f;
        // 消费先前的反馈，保持其原始接收时间，不能用消费时间冒充新反馈。
        for (unsigned n = 0; n < 64 && osMessageQueueGet(feedback, &f, nullptr, 0) == osOK; ++n) {
            const bool accepted = arm.receive(f.bus, f.id, f.data, f.len, f.ms);
            diagnose_yaw(arm, f, accepted);
        }
        uint32_t now = HAL_GetTick();
        for (unsigned b = 0; b < 2; ++b) {
            if ((owned_buses & (1u << b)) && can_tx_error_count[b] != previous_errors[b])
                arm.transport_fault();
            previous_errors[b] = can_tx_error_count[b];
        }
        if (dropped_rx != previous_drops)
            arm.transport_fault();
        previous_drops = dropped_rx;
        if (((owned_buses & 1) && (hcan1.Instance->ESR & CAN_ESR_BOFF)) ||
            ((owned_buses & 2) && (hcan2.Instance->ESR & CAN_ESR_BOFF)))
            arm.transport_fault();
        bool user_stopped = false;
        if (stop_requested) {
            taskENTER_CRITICAL();
            stop_requested = false;
            taskEXIT_CRITICAL();
            arm.disable();
            osMessageQueueReset(commands);
            user_stopped = true;
        } else {
            Command c;
            // 一轮只处理启动时已有命令，避免生产者持续入队饿死控制周期。
            for (unsigned n = 0; n < 16 && osMessageQueueGet(commands, &c, nullptr, 0) == osOK;
                 ++n) {
                now = HAL_GetTick();
                if (uint32_t(now - c.submitted_ms) > 50)
                    continue;
                switch (c.kind) {
                case Zero:
                    arm.capture_zero(c.mask, now);
                    break;
                case Enable:
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
                    INS_RevokeCalibrationPermission(now);
#endif
                    arm.enable(c.mask, now);
                    break;
                case Clear:
                    arm.clear_faults();
                    break;
                case Target:
                    arm.set_targets(c.mask, c.q, c.submitted_ms);
                    break;
                }
            }
        }
        now = HAL_GetTick();
        const auto previous_tune_epoch = g_robot_tune_status.epoch;
        const bool previous_tune_ready = g_robot_tune_status.ready;
        Robot_Execute(arm, now, user_stopped);
        if (previous_tune_epoch != g_robot_tune_status.epoch ||
            (previous_tune_ready && !g_robot_tune_status.ready))
            // Drop old motion frames, then republish every held axis below.
            // This cancels stale Yaw commands without changing arm hold targets.
            can_cancel_pending(owned_buses);
        now = HAL_GetTick();
        static uint32_t last_yaw_cycles = 0;
        static bool have_yaw_cycles = false;
        const uint32_t yaw_cycles = BSP_DWT_Cycles();
        const float yaw_dt_s = have_yaw_cycles
            ? BSP_DWT_ElapsedSeconds(yaw_cycles, last_yaw_cycles)
            : (kArmConfig.control_period_ms ? float(kArmConfig.control_period_ms) * 0.001f
                                            : 0.004f);
        last_yaw_cycles = yaw_cycles;
        have_yaw_cycles = true;
        arm.set_yaw_control_dt(yaw_dt_s);
        Frame frames[kJointCount]{};
        apply_tune_requests(arm, now);
        const unsigned count = arm.step(now, frames);
#endif
        if (previous_enabled & ~arm.status().enabled_mask)
            can_cancel_pending(owned_buses);
        for (unsigned i = 0; i < count; ++i) {
            auto &f = frames[i];
            can_device_transmit tx(f.bus == 1 ? &hcan1 : &hcan2, f.id, 8, f.data);
            if (tx.can_send_msg() != osOK) {
                arm.transport_fault();
                can_cancel_pending(owned_buses);
                break;
            }
        }
        Robot_ReportArmState(arm.status());
        publish(arm, now);
        // FreeRTOS returns the historical minimum free stack in words.
        static uint32_t stack_check_ms = 0;
        if (uint32_t(now - stack_check_ms) >= 250) {
            g_arm_stack_free_bytes = uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
            stack_check_ms = now;
        }
        // 迟到后从当前tick重新排程，禁止连续补跑同一毫秒的控制环。
        const uint32_t current = osKernelGetTickCount();
        tick = int32_t(current - tick) > 0 ? current + period : tick + period;
        osDelayUntil(tick);
    }
}

// Stack corruption is terminal: do not try to run motor logic on a damaged stack.
// External power isolation / drive-side command timeout remains the safety backstop.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *name) {
    g_stack_overflow_latched = 1;
    for (unsigned i = 0; i < 15; ++i) {
        g_stack_overflow_task[i] = name ? name[i] : 0;
        if (!g_stack_overflow_task[i]) break;
    }
    g_stack_overflow_task[15] = 0;
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}
