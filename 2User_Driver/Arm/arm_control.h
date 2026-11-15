#pragma once
#include <cstddef>
#include <cstdint>
#include "Arm/yaw_pid.h"

// 上层使用关节端deg/deg/s：BigPitch=DM1独立关节角，SmallPitch=DM2独立关节角。
// 两轴运动已解耦，不再使用q肘=a2-a1或a2=q肩+q肘换算；电机闭环和effort直接使用各自驱动角。
namespace engineer {
constexpr unsigned kJointCount = 5;
constexpr uint8_t kAllJoints = 0x1f;
constexpr uint8_t kFourAxisJoints = 0x1e; // 大/小pitch、roll、夹爪；无yaw
enum Joint : uint8_t { Yaw, BigPitch, SmallPitch, Roll, Gripper };
// 兼容旧调用方的组合mask；两pitch已解耦，不再要求成对校零/使能。
constexpr uint8_t kPitchPair = (1u << BigPitch) | (1u << SmallPitch);
constexpr uint8_t kParkedArmGroups[] = {1u << BigPitch, 1u << SmallPitch, 1u << Roll,
                                        1u << Gripper};
constexpr int32_t kJogCapturePitchZero = 5, kJogClearFaults = 8, kJogStop = 9;
enum class Motor : uint8_t { Gm6020Voltage, M2006, Dm4310Mit, Gm6020Current };
enum Fault : uint32_t {
    NoFault = 0,
    FeedbackTimeout = 1u << 0,
    CommandTimeout = 1u << 1,
    OverTemperature = 1u << 2,
    MotorError = 1u << 3,
    PositionLimit = 1u << 4,
    PositionLost = 1u << 5,
    TransportError = 1u << 6,
    TimingError = 1u << 7,
    EnableTimeout = 1u << 8
};
constexpr uint32_t kSharedArmFaults = TransportError | TimingError;
enum class Result : uint8_t {
    Ok,
    InvalidArgument,
    ConfigUnconfirmed,
    Busy,
    NotOnline,
    NotCalibrated,
    FaultLatched,
    Moving,
    OutOfRange,
    InterlockOpen
};

struct JointConfig {
    Motor motor;
    uint8_t bus;              // 1=CAN1, 2=CAN2
    uint16_t motor_id;        // DJI: 拨码/电调ID; DM: ESC_ID
    uint16_t feedback_id;     // DM: MST_ID; DJI由型号和motor_id推导
    float feedback_per_joint; // 反馈轴角/驱动坐标角；pitch为绝对连杆驱动角，夹爪为32T轮角
    int8_t direction;         // 驱动坐标正向，+1/-1同时作用于测量和输出
    float min_deg, max_deg;   // 公共关节q范围；第二DM驱动范围另见Config.pitch
    float max_speed_deg_s;
    float max_accel_deg_s2;
    float max_effort;                    // 6020电流模式/C610: mA; DM: Nm; 6020电压模式: 原码
    float position_kp;                   // 驱动坐标位置误差deg -> 驱动速度deg/s
    float velocity_kp, velocity_ki;      // 驱动速度PI -> max_effort对应单位，积分乘实际dt
    float p_max, v_max, t_max;           // DM协议映射范围，必须与上位机读数一致
    float max_temperature_c;             // C610不提供温度，此项不适用于M2006
    float limit_margin_deg = 2.0f;       // 反馈越界锁存余量，同样按此轴角度单位
    float stationary_speed_deg_s = 2.0f; // 手动校零允许的驱动速度阈值
    float initial_effort_limit = -1.0f;  // -1 uses max_effort; same units as max_effort.
    // Pitch DM drives only. Angle is the calibrated drive angle plus this offset.
    float torque_bias_nm = 0, gravity_gain_nm = 0, gravity_offset_deg = 0;
    // Nonnegative motion-assist magnitudes. up_direction maps physical up to drive angle.
    float up_ff_nm = 0, down_ff_nm = 0;
    int8_t up_direction = 1;
    int16_t fixed_zero_encoder = -1;
    bool encoder_velocity = false; // Yaw only; false restores CAN rpm feedback. // DJI单圈固定零点0..8191；-1表示未标定
};
// 已弃用：两pitch解耦后不再使用第二驱动a2=q肩+q肘范围、两反馈时差和成对确认。
// 字段暂时保留以兼容现有配置初始化和调试工具，后续可整体删除。
struct PitchConfig {
    bool mapping_confirmed;
    float second_drive_min_deg, second_drive_max_deg;
    float second_drive_max_speed_deg_s, second_drive_max_accel_deg_s2;
    uint32_t max_feedback_skew_ms;
};
struct Config {
    JointConfig joint[kJointCount];
    bool hardware_confirmed; // 接线、传动、零位、限位确认后才改true
    uint32_t control_period_ms;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    uint32_t dji_unwrap_gap_ms;
    PitchConfig pitch;
    uint8_t joint_mask = kAllJoints; // 编译配置只选整车或四轴台架
};
extern const Config kArmConfig;

struct Frame {
    uint8_t bus;
    uint16_t id;
    uint8_t data[8];
};
struct YawRuntimeLimits {
    float min_deg = 0, max_deg = 0, max_speed_deg_s = 0;
    float braking_accel_deg_s2 = 0, braking_margin_deg = 0, max_target_lead_deg = 0;
    uint32_t version = 0;
};
struct JointStatus {
    // effort: 电流模式6020/C610为mA，DM为Nm；6020电压模式为原码。
    float position_deg, velocity_deg_s, target_deg, reference_deg, effort;
    float drive_position_deg, drive_velocity_deg_s, drive_target_deg, drive_reference_deg;
    float drive_reference_velocity_deg_s;
    float motor_effort_feedback; // DJI反馈原码 / DM Nm；不是夹持力
    float temperature_c;
    float velocity_target_deg_s;
    float feedforward_effort; // Pitch bias + cosine term before the combined output limit, Nm.
    uint32_t received, last_feedback_ms, faults;
    bool online, calibrated, enabled;
    uint8_t motor_state; // DM: 0=失能,1=使能,8~F=故障
    uint16_t raw_encoder; // DJI当前原始单圈编码器，0..8191
    int16_t fixed_zero_encoder; // 生效配置，-1表示未填写
    float can_velocity_deg_s, encoder_velocity_deg_s;
    uint32_t velocity_window_ms;
    bool homing; // 固定零点轴开控回零中
    float motion_feedforward_effort;
    int8_t motion_ff_direction; // +1 up, -1 down, 0 holding; selected from desired speed.
};
struct Status {
    JointStatus joint[kJointCount];
    uint8_t enabled_mask;
    uint32_t faults;
    Result last_result;
    bool configuration_valid;
};

struct JointTuning {
    float position_kp, velocity_kp, velocity_ki, effort_limit;
    float torque_bias_nm = 0, gravity_gain_nm = 0, gravity_offset_deg = 0;
    float up_ff_nm = 0, down_ff_nm = 0;
};

// Read-only diagnostics for the Yaw-only PID pair.
struct YawPidSnapshot {
    float dt_s = 0;
    float position_kp = 0, position_ki = 0, position_kd = 0, position_error = 0;
    float position_max_output = 0, position_max_i_error = 0, position_max_sigma = 0;
    float position_p = 0, position_i = 0, position_d = 0;
    float position_sigma = 0, position_output = 0;
    float velocity_kp = 0, velocity_ki = 0, velocity_kd = 0, velocity_error = 0;
    float velocity_max_output = 0, velocity_max_i_error = 0, velocity_max_sigma = 0;
    float velocity_p = 0, velocity_i = 0, velocity_d = 0;
    float velocity_sigma = 0, velocity_prelimit = 0, velocity_output = 0;
};

// 纯控制逻辑，无HAL依赖；由一个任务拥有，禁止从ISR直接调用。
class ArmControl {
  public:
    explicit ArmControl(const Config &config);
    bool receive(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t len, uint32_t now_ms);
    Result capture_zero(uint8_t mask, uint32_t now_ms);
    // Explicitly add the requested axes; already-enabled axes keep their state.
    Result enable_checked(uint8_t mask, uint32_t now, bool require_confirmation);
    Result enable(uint8_t mask, uint32_t now_ms, bool independent_yaw = false);
    Result capture_parked_zero(uint32_t now_ms);
    Result enable_parked_tuning(uint32_t now_ms);
    void disable();
    // Owner-only mode changes. Neither operation enables an axis; pitch stays paired.
    Result hold(uint8_t mask, uint32_t now_ms);
    Result disable_joints(uint8_t mask);
    Result clear_faults();
    Result set_targets(uint8_t mask, const float positions[kJointCount], uint32_t now_ms);
    void transport_fault();
    Result set_effort_limit(unsigned joint, float limit);
    // Disabled-only RAM tuning. Limits never exceed the startup output cap.
    Result set_tuning(unsigned joint, const JointTuning &tuning, uint32_t now_ms);
    // Normal runtime tuning requires settled drives; parked tuning accepts moving,
    // healthy axes. The task owner checks remote input before calling this method.
    Result set_tuning_runtime(unsigned joint, const JointTuning &tuning, uint32_t now_ms);
    bool get_tuning(unsigned joint, JointTuning &tuning) const;
    Result set_yaw_limits(const YawRuntimeLimits &limits, uint32_t now_ms);
    Result set_yaw_envelope(float min_deg, float max_deg, float speed_deg_s);
    Result set_yaw_test_speed(float speed_deg_s, uint32_t now_ms);
    void clear_yaw_test();
    void set_yaw_tuning_limits(bool enabled) { yaw_tuning_unlimited_ = !enabled; }
    // Voltage-mode Yaw only. Added in the direction of a non-zero speed target
    // to break static friction; zero disables. Invalid writes keep the old value.
    void set_yaw_friction_feedforward(float raw);
    float yaw_friction_feedforward_applied() const { return yaw_friction_ff_applied_; }
    // Actual Yaw PID timestep measured by the task layer. Invalid values are ignored.
    void set_yaw_control_dt(float dt_s);
    float yaw_control_dt_s() const { return yaw_control_dt_s_; }
    YawPidSnapshot yaw_pid_snapshot() const;
    // Called only after the tuning owner has enabled and validated Yaw.
    void bypass_yaw_homing_for_tuning() { status_.joint[Yaw].homing = false; }
    const YawRuntimeLimits &yaw_limits() const {
        return yaw_limits_;
    }
    // 输出最多5帧：DJI按bus+组ID合帧，DM每台一帧。调用者提供>=5个槽位。
    unsigned step_jog(uint32_t now_ms, int32_t request, bool permit, Frame out[kJointCount]);
    unsigned step(uint32_t now_ms, Frame out[kJointCount]);
    const Status &status() const {
        return status_;
    }
    uint16_t feedback_id(unsigned joint) const;

  private:
    struct Runtime {
        uint16_t last_encoder;
        int64_t encoder_sum;
        float motor_position_deg, zero_motor_deg, integral, reference_velocity, effort_limit;
        uint32_t last_command_ms, enable_ms;
        bool have_feedback, dm_was_enabled;
    } runtime_[kJointCount]{};
    struct VelocitySample { uint32_t ms; int64_t ticks; } yaw_velocity_history_[64]{};
    uint32_t yaw_velocity_head_ = 0, yaw_velocity_count_ = 0;
    int64_t yaw_velocity_ticks_ = 0;
    uint16_t yaw_velocity_last_encoder_ = 0;
    float update_yaw_velocity(uint16_t encoder, uint32_t now, bool reset);
    struct RollZeroStillness {
        int64_t min_encoder, max_encoder;
        uint32_t since_ms;
        bool tracking;
    } roll_zero_stillness_{};
    unsigned output_frames(Frame out[kJointCount]) const;
    unsigned jog_axis_ = BigPitch;
    int8_t jog_direction_ = 0;
    float jog_start_deg_[kJointCount]{};
    Result start_jog(unsigned joint, int8_t direction, uint32_t now_ms);
    void run_jog(uint32_t now_ms);
    Config config_;
    const float startup_effort_cap_[kJointCount];
    float tuning_effort_limit_[kJointCount]{};
    Status status_{};
    uint32_t last_step_ms_ = 0;
    bool have_step_ = false, parked_tuning_ = false;
    YawRuntimeLimits yaw_limits_{};
    bool yaw_tuning_unlimited_ = false;
    float yaw_motion_min_ = 0, yaw_motion_max_ = 0, yaw_motion_speed_ = 0, yaw_test_speed_ = 0;
    float yaw_friction_ff_raw_ = 0;
    float yaw_friction_ff_applied_ = 0;
    bool yaw_dt_valid_ = true;
    float yaw_control_dt_s_ = 0.004f;
    YawPid yaw_position_pid_{}, yaw_velocity_pid_{};
    void reset_yaw_pid();
    float yaw_position_ff_ = 0;
    bool yaw_position_ff_active_ = false;
    float yaw_speed_ff_ = 0;
    bool yaw_test_active_ = false;
    float limit_yaw_speed(float requested, float dt) const;
    void refresh(uint32_t now_ms);
    void update_joint_feedback();
    void update_drive_references();
    bool targets_in_range(const float q[kJointCount]) const;
    float drive_min(unsigned joint) const;
    float drive_max(unsigned joint) const;
    float drive_speed_limit(unsigned joint) const;
    void trip(uint32_t fault, unsigned joint);
    Result set_tuning_checked(unsigned joint, const JointTuning &tuning, uint32_t now_ms,
                             bool runtime);
    bool valid_config() const;
    bool selected(unsigned joint) const {
        return (config_.joint_mask & (1u << joint)) != 0;
    }
    uint16_t command_id(unsigned joint) const;
};
// MIT纯力矩模式: kp=kd=0，通过外部位置/速度闭环限幅后给t_ff。
void pack_dm_torque(float torque, float p_max, float v_max, float t_max, uint8_t data[8]);
} // namespace engineer
