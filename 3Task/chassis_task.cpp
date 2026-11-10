#include "chassis_task.h"
#include "Chassis/chassis_control.h"
#include "Chassis/chassis_bench.h"
#include "Power/power_meter.h"
#include "Power/chassis_power.h"
#include "robot_control.h"
#include "arm_task.h"
#include "task_ins.h"
#include "Remote/operator_control.h"
#include "dr16_bsp.h"
#include "CAN_bsp.h"
#include <cmath>
#include <cstring>

#ifndef VEHICLE_SAFETY_ENABLED
#define VEHICLE_SAFETY_ENABLED 0
#endif

extern "C" {
ChassisStatus g_chassis_status{};
volatile ChassisDebugRequest g_chassis_debug_request{};
ChassisDebugStatus g_chassis_debug_status{};
}

namespace {
using namespace engineer;

#if !ARM_FOUR_AXIS_BENCH || CHASSIS_SUSPENDED_BENCH
struct Rx {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
    uint32_t ms;
};

osMessageQueueId_t rx_queue = nullptr;
volatile uint32_t drops = 0;

// CAN 回调只按值入队；解析、闭环和发送全部由本任务完成。
void on_rx(can_device_receive *device, uint8_t *data) {
    Rx frame{};
    frame.ms = HAL_GetTick();
    frame.len = device->rx_dlc;
    if (device->matches(&hcan2, 0x31)) {
        frame.id = 0x31;
    } else if (device->matches(&hcan2, 0x200 + kChassisConfig.motor_id[0])) {
        frame.id = 0x200 + kChassisConfig.motor_id[0];
    } else {
        frame.id = 0x200 + kChassisConfig.motor_id[1];
    }
    std::memcpy(frame.data, data, sizeof(frame.data));
    if (rx_queue && osMessageQueuePut(rx_queue, &frame, 0, 0) != osOK) {
        ++drops;
    }
}

// 本文件私有的运行上下文。设备与控制核心各只有一个实例，CAN2 只有本任务发送。
struct ChassisRuntime {
    ChassisMotor motors[2];
    PowerMeter power;
    ChassisControl control{kChassisConfig};
    ChassisPower energy;
    WheelFeedback wheels[2]{};
    ChassisCommand command{};
    ChassisDebugRequest debug_request{};
    float final_ma[2]{}; // 轮组正向坐标，mA；组帧前已经过所有限制。
    uint32_t now_ms = 0;
    uint32_t tx_errors = 0;
    uint32_t previous_drops = 0;
    uint32_t tick = 0;
    uint32_t previous_stop_epoch = 0;
    bool transport_error = false;
    bool explicit_stop = false;
    bool bench_active = false;
    bool previous_enabled = false;
#if CHASSIS_SUSPENDED_BENCH
    ChassisBenchGate bench_gate;
#else
    uint32_t debug_seq = 0;
    bool local_stop_latched = false;
#endif
};

void publish_invalid_configuration(const ChassisRuntime &runtime) {
    ChassisStatus status{};
    status.ready = 1;
    status.updated_ms = HAL_GetTick();
    status.faults = ChassisConfigFault;
    status.hardware_confirmed = kChassisConfig.hardware_confirmed;
    status.power_configuration_valid = runtime.energy.status().configuration_valid;
    status.power_confirmed = runtime.energy.status().confirmed;
    taskENTER_CRITICAL();
    g_chassis_status = status;
    taskEXIT_CRITICAL();
}

// 输入：带 CAN 源时间的队列帧。输出：两轮/功率计反馈与本周期总线异常。
void receive_feedback(ChassisRuntime &runtime) {
    Rx frame;
    for (unsigned count = 0; count < 64 && osMessageQueueGet(rx_queue, &frame, nullptr, 0) == osOK;
         ++count) {
        if (frame.id == 0x31) {
            runtime.power.receive(frame.data, frame.len, frame.ms);
        } else {
            for (unsigned i = 0; i < 2; ++i) {
                if (frame.id == 0x200 + kChassisConfig.motor_id[i]) {
                    runtime.motors[i].receive(frame.data, frame.len, frame.ms);
                }
            }
        }
    }
    runtime.now_ms = HAL_GetTick();
    runtime.transport_error = runtime.tx_errors != can_tx_error_count[1] ||
                              drops != runtime.previous_drops ||
                              (hcan2.Instance->ESR & CAN_ESR_BOFF);
    if (runtime.transport_error) {
        runtime.control.trip(ChassisTransportFault);
        runtime.bench_active = false;
        can_cancel_pending(2);
    }
    runtime.tx_errors = can_tx_error_count[1];
    runtime.previous_drops = drops;
    for (unsigned i = 0; i < 2; ++i) {
        runtime.wheels[i] = runtime.motors[i].status();
    }
}

ChassisDebugRequest read_debug_request() {
    ChassisDebugRequest request{};
    taskENTER_CRITICAL();
    request.mode = g_chassis_debug_request.mode;
    request.wheel = g_chassis_debug_request.wheel;
    request.duration_ms = g_chassis_debug_request.duration_ms;
    request.value = g_chassis_debug_request.value;
    request.suspended_confirm = g_chassis_debug_request.suspended_confirm;
    request.request_seq = g_chassis_debug_request.request_seq;
    taskEXIT_CRITICAL();
    return request;
}

#if CHASSIS_SUSPENDED_BENCH
// 任务只读取快照并转换为纯输入，遥控/轮反馈判定留在 chassis_bench。
void read_bench_command(ChassisRuntime &runtime) {
    DR16Status remote{};
    DR16_GetStatus(&remote);
    ArmStatus arm{};
    Arm_GetStatus(&arm);

    ChassisBenchRemoteInput input{};
    input.mapping_confirmed = kOperatorConfig.mapping_confirmed;
    input.valid = remote.valid;
    input.received_ms = remote.received_ms;
    input.consecutive_frames = remote.consecutive_frames;
    for (unsigned i = 0; i < 5; ++i) {
        input.channel[i] = remote.frame.channel[i];
    }
    input.switches_in_test_position =
        remote.frame.switches[kOperatorConfig.left_switch_index] == DR16_SWITCH_UP &&
        remote.frame.switches[kOperatorConfig.right_switch_index] == DR16_SWITCH_MID;
    input.owner_idle = arm.ready && arm.enabled_mask == 0;
    input.robot_ready = runtime.command.valid;
    input.robot_updated_ms = runtime.command.updated_ms;

    const auto feedback =
        chassis_bench_feedback_health(runtime.now_ms, runtime.wheels, kChassisConfig);
    ChassisBenchHealth health{};
    health.hardware_confirmed = kChassisConfig.hardware_confirmed;
    health.mapping_confirmed = kOperatorConfig.mapping_confirmed;
    health.remote_ok = chassis_bench_remote_ready(runtime.now_ms, input);
    health.owner_idle = input.owner_idle;
    health.feedback_fresh = feedback.fresh;
    health.temperature_ok = feedback.temperature_ok;
    health.wheels_stationary = feedback.stationary;
    health.transport_ok = !runtime.transport_error;
    health.timing_ok =
        !g_chassis_status.ready ||
        uint32_t(runtime.now_ms - g_chassis_status.updated_ms) <= kChassisConfig.period_ms + 2;
    health.controller_faulted = runtime.control.status().faults != 0;
    health.controller_enabled = runtime.control.status().enabled;
    health.global_stop = runtime.explicit_stop;
    health.stop_epoch = runtime.command.stop_epoch;

    const auto &request = runtime.debug_request;
    const ChassisBenchRequest pulse{request.mode,
                                    request.wheel,
                                    request.duration_ms,
                                    request.value,
                                    request.suspended_confirm,
                                    request.request_seq};
    const auto &bench = runtime.bench_gate.step(runtime.now_ms, pulse, health);
    runtime.bench_active = bench.active;
    if (bench.clear_requested) {
        runtime.control.trip(0);
        runtime.control.clear_faults();
    }
    g_chassis_debug_status = {bench.consumed_seq, uint32_t(bench.result), bench.until_ms};
    runtime.command = {};
    runtime.command.valid = true;
    runtime.command.updated_ms = runtime.now_ms;
    runtime.command.enabled = runtime.bench_active;
    runtime.command.stop = true;
    runtime.command.brake = true;
    runtime.command.reset = false;
}
#else
void read_local_stop_request(ChassisRuntime &runtime) {
    const bool owner_disabled = !runtime.command.enabled;
    const auto &request = runtime.debug_request;
    if (request.request_seq != runtime.debug_seq) {
        runtime.debug_seq = request.request_seq;
        g_chassis_debug_status = {runtime.debug_seq, uint32_t(ChassisBenchResult::InvalidRequest),
                                  0};
        if (request.mode == CHASSIS_DEBUG_STOP || request.mode == CHASSIS_DEBUG_CLEAR) {
            runtime.local_stop_latched = true;
            runtime.command.enabled = false;
            runtime.control.trip(0);
            can_cancel_pending(2);
            Robot_StopControl();
            if (request.mode == CHASSIS_DEBUG_CLEAR) {
                const bool wheels_cleared = runtime.control.clear_faults();
                const bool power_cleared =
                    runtime.energy.clear(runtime.now_ms, runtime.power.status(), true);
                g_chassis_debug_status.result =
                    uint32_t(wheels_cleared && power_cleared ? ChassisBenchResult::ClearRequested
                                                             : ChassisBenchResult::NotReady);
            } else {
                g_chassis_debug_status.result = uint32_t(ChassisBenchResult::Stopped);
            }
        }
    }
    if (runtime.local_stop_latched) {
        runtime.command.enabled = false;
        if (owner_disabled) {
            runtime.local_stop_latched = false;
        }
    }
}
#endif

// 输入：唯一整车 owner 的速度请求[-1,1]、模式及停控事件；调试请求只消费一次。
void read_command(ChassisRuntime &runtime) {
    Robot_GetChassisCommand(&runtime.command);
    runtime.explicit_stop = runtime.command.stop_epoch != runtime.previous_stop_epoch;
    runtime.previous_stop_epoch = runtime.command.stop_epoch;
    if (runtime.explicit_stop) {
        runtime.bench_active = false;
        can_cancel_pending(2);
    }

    runtime.debug_request = read_debug_request();
#if CHASSIS_SUSPENDED_BENCH
    read_bench_command(runtime);
#else
    read_local_stop_request(runtime);
#endif
}

// 原差速/斜坡/轮速 PI 核心不变；输出为所有功率/安全约束之后的轮电流 mA。
void control_and_limit(ChassisRuntime &runtime) {
    runtime.energy.step(runtime.now_ms, runtime.power.status(), runtime.control.status().enabled,
                        runtime.control.status().wheels_stationary);
    auto &command = runtime.command;
#if !CHASSIS_SUSPENDED_BENCH
    const auto &energy = runtime.energy.status();
    command.enabled &= energy.session_allowed;
    if (energy.stop_requested) {
        command.stop = true;
        command.brake = true;
        command.speed_request = 0;
        command.turn_request = 0;
    } else {
        command.speed_request *= energy.speed_scale;
        command.turn_request *= energy.speed_scale;
    }
#endif
#if VEHICLE_SAFETY_ENABLED && !CHASSIS_SUSPENDED_BENCH && !YAW_SUSPENDED_BENCH
    // 先撤销旧的静态校准许可，再允许本周期首次出力。
    if (command.enabled && !runtime.control.status().enabled) {
        INS_RevokeCalibrationPermission(runtime.now_ms);
    }
#endif
    runtime.control.step(runtime.now_ms, runtime.wheels, command);
    runtime.final_ma[0] = runtime.control.status().candidate_ma[0];
    runtime.final_ma[1] = runtime.control.status().candidate_ma[1];
#if CHASSIS_SUSPENDED_BENCH
    const auto &bench = runtime.bench_gate.status();
    if (runtime.control.status().enabled && runtime.bench_active) {
        if (bench.mode == CHASSIS_DEBUG_CURRENT) {
            runtime.control.direct_output();
            runtime.final_ma[0] = 0;
            runtime.final_ma[1] = 0;
            runtime.final_ma[bench.wheel] =
                std::fmax(-kChassisConfig.max_current_ma,
                          std::fmin(kChassisConfig.max_current_ma, bench.value));
        } else {
            runtime.control.step_debug_speed(bench.wheel, bench.value);
            runtime.final_ma[0] = runtime.control.status().candidate_ma[0];
            runtime.final_ma[1] = runtime.control.status().candidate_ma[1];
        }
    }
#else
    runtime.energy.limit(runtime.control.status().candidate_ma,
                         runtime.control.status().measured_rpm, runtime.final_ma);
#endif
    if (!runtime.control.status().enabled) {
        runtime.final_ma[0] = 0;
        runtime.final_ma[1] = 0;
    }
}

// CAN2 唯一发送入口：设备层编码，最终量化电流用于原 PI 抗积分饱和。
void submit_output(ChassisRuntime &runtime) {
    if (runtime.previous_enabled && !runtime.control.status().enabled) {
        can_cancel_pending(2);
    }
    uint8_t data[8];
    float submitted_ma[2]{};
    ChassisMotor::pack_current(runtime.final_ma[0], runtime.final_ma[1], kChassisConfig.motor_id,
                               kChassisConfig.direction, data, submitted_ma);
    can_device_transmit message(&hcan2, 0x200, 8, data);
    const bool submitted = message.can_send_msg() == osOK;
    runtime.control.apply(submitted_ma, submitted);
    if (!submitted) {
        runtime.bench_active = false;
        can_cancel_pending(2);
    }
    runtime.previous_enabled = runtime.control.status().enabled;
}

// 一次发布完整快照；current_ma 是已接受入队的量化值，不是电机执行确认。
void publish_status(const ChassisRuntime &runtime) {
    const auto &control = runtime.control.status();
    const auto &power = runtime.power.status();
    const auto &energy = runtime.energy.status();
    ChassisStatus status{};
    status.ready = 1;
    status.updated_ms = runtime.now_ms;
    status.configuration_valid = control.configuration_valid;
    status.hardware_confirmed = kChassisConfig.hardware_confirmed;
    status.power_configuration_valid = energy.configuration_valid;
    status.power_confirmed = energy.confirmed;
    status.enabled = control.enabled;
    status.wheels_stationary = control.wheels_stationary;
    status.bench_active = runtime.bench_active && control.enabled;
    status.faults = control.faults;
    status.rx_dropped = drops;
    status.tx_errors = can_tx_error_count[1];
    status.power_session_allowed = energy.session_allowed;
    status.power_stop_requested = energy.stop_requested;
    status.power_inhibited = energy.inhibited;
    status.power_phase = uint8_t(energy.phase);
    status.power_reason = uint8_t(energy.reason);
    status.v_m_s = control.v_m_s;
    status.omega_rad_s = control.omega_rad_s;
    status.power_w = power.power_w;
    status.energy_j = power.energy_j;
    status.power_received_ms = power.received_ms;
    status.power_frames = power.frames;
    for (unsigned i = 0; i < 2; ++i) {
        status.measured_rpm[i] = control.measured_rpm[i];
        status.target_rpm[i] = control.target_rpm[i];
        status.current_ma[i] = control.applied_ma[i];
        status.feedback_ms[i] = runtime.wheels[i].received_ms;
        status.feedback_frames[i] = runtime.wheels[i].frames;
        status.temperature_c[i] = runtime.wheels[i].temperature_c;
    }
    taskENTER_CRITICAL();
    g_chassis_status = status;
    taskEXIT_CRITICAL();
}

void wait_next_cycle(ChassisRuntime &runtime) {
    const uint32_t current = osKernelGetTickCount();
    if (int32_t(current - runtime.tick) > 0) {
        runtime.tick = current + kChassisConfig.period_ms;
    } else {
        runtime.tick += kChassisConfig.period_ms;
    }
    osDelayUntil(runtime.tick);
}
#endif
} // namespace

extern "C" int Chassis_GetStatus(ChassisStatus *out) {
    if (!out) {
        return 0;
    }
    taskENTER_CRITICAL();
    *out = g_chassis_status;
    taskEXIT_CRITICAL();
    return out->ready;
}

extern "C" void chassis_task(void *) {
#if ARM_FOUR_AXIS_BENCH && !CHASSIS_SUSPENDED_BENCH
    // 日常四轴台架不注册 CAN2 电机，也不发送底盘输出。
    for (;;) {
        osDelay(100);
    }
#else
    rx_queue = osMessageQueueNew(64, sizeof(Rx), nullptr);
    if (!rx_queue || osKernelGetTickFreq() != 1000) {
        Error_Handler();
    }
    ChassisRuntime runtime;
    if (!runtime.control.status().configuration_valid) {
        publish_invalid_configuration(runtime);
        for (;;) {
            osDelay(100);
        }
    }
    can_device_receive left(&hcan2, 0x200 + kChassisConfig.motor_id[0], on_rx);
    can_device_receive right(&hcan2, 0x200 + kChassisConfig.motor_id[1], on_rx);
    can_device_receive meter(&hcan2, 0x31, on_rx);
    runtime.tx_errors = can_tx_error_count[1];
    runtime.tick = osKernelGetTickCount();

    // 参照旧整车任务：收反馈 -> 读请求 -> 原核心控制/限幅 -> 统一发送 -> 发布。
    for (;;) {
        receive_feedback(runtime);
        read_command(runtime);
        control_and_limit(runtime);
        submit_output(runtime);
        publish_status(runtime);
        wait_next_cycle(runtime);
    }
#endif
}
