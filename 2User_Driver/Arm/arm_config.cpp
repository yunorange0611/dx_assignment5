#include "Arm/arm_control.h"
#include "Arm/yaw_config.h"

// C610 IDs: feedback ID and command slot are derived by ArmControl.
#define ARM_ROLL_C610_ID 3
#define ARM_GRIPPER_C610_ID 4
static_assert(ARM_ROLL_C610_ID >= 1 && ARM_ROLL_C610_ID <= 8 &&
              ARM_GRIPPER_C610_ID >= 1 && ARM_GRIPPER_C610_ID <= 8 &&
              ARM_ROLL_C610_ID != ARM_GRIPPER_C610_ID,
              "C610 IDs must be distinct values from 1 to 8");

#ifndef ARM_FOUR_AXIS_BENCH
#define ARM_FOUR_AXIS_BENCH 0 // 日常单整车固件；四轴配置只用于兼容逻辑测试
#endif

namespace engineer {
constexpr float kPitchMinDeg = -2.0f;
constexpr float kPitchLimitDeg = 150.0f;
constexpr float kSmallPitchLimitDeg = 70.0f;
constexpr int8_t kGripperDirection = 1; // 闭合参考为0，张开时编码器累计值和关节角增大。
// 2026-11-14 临时静平衡标定值；装齐六颗螺丝并重校零后仍需复测。
// BigP: 三点固定 offset=90° 拟合，最大残差 0.039 Nm。
// SmallP: 四点拟合，排除 25.75° 的 1.80 Nm 异常点，最大残差 0.031 Nm。
constexpr float kBigPitchTorqueBiasNm = -0.228f;
constexpr float kBigPitchGravityGainNm = 0.437f;
constexpr float kBigPitchGravityOffsetDeg = 90.0f;
constexpr float kSmallPitchTorqueBiasNm = 1.276f;
constexpr float kSmallPitchGravityGainNm = 0.248f;
constexpr float kSmallPitchGravityOffsetDeg = -48.33f;

// 仅为台架起步配置。以下角度不是从CAD测得的机械限位，不能直接带载运行。
// M2006反馈在转子端: 36 * 外部减速比；DM反馈在内部减速器输出端，勿再次乘10。
// CAD：yaw为37T:37T同步带；两DM为同轴安装的连杆驱动；roll暂按联轴器直连。
// 夹爪M2006经24T->32T驱动轮，角度统一为32T轮转角：总比36*32/24=48。
// 夹爪闭合参考为0、张开为正，45deg行程暂定，速度/加速度沿用旧轴侧参数乘0.75。
// 速度PI增益除以0.75，位置Kp和电流限幅不变。所有数值仍需台架实测。
const Config kArmConfig = {
    {// type, bus, id, mst, ratio, dir, min, max, speed, accel,
     // effort, posKp, velKp, velKi, PMAX, VMAX, TMAX, temp
     kYawConfig, // 底盘/yaw负责人维护yaw_config.h；运行时仍由本任务统一发送。
                 // 以下四轴由机械臂负责人维护，控制量分别为Nm、Nm、mA、mA。
     {Motor::Dm4310Mit, 1, 1, 0x11, 1.0f, 1, kPitchMinDeg, kPitchLimitDeg, 30, 30,
      6.5f, 2.0f, 0.012f, 0.0f,         // 力矩软上限、位置 Kp、速度 Kp、速度 Ki
      3.14159f, 30.0f, 10.0f,           // PMAX、VMAX、TMAX，与达妙上位机一致
      65.0f, 2.0f, 2.0f, 6.5f,         // 温度上限、越界余量、校零速度阈值、启动限矩
      kBigPitchTorqueBiasNm, kBigPitchGravityGainNm, kBigPitchGravityOffsetDeg,
      0.85f, 0.20f, -1},               // 用户暂用上/下行补偿Nm；大P抬升时驱动角减小
     {Motor::Dm4310Mit, 1, 2, 0x12, 1.0f, 1, -kSmallPitchLimitDeg, kSmallPitchLimitDeg, 30, 30,
      6.5f, 2.0f, 0.01f, 0.0f, 3.14159f, 30.0f, 10.0f, 65,
      2.0f, 2.0f, 6.5f,               // 越界余量、校零速度阈值、启动限矩
      kSmallPitchTorqueBiasNm, kSmallPitchGravityGainNm, kSmallPitchGravityOffsetDeg,
      0.0f, 0.0f, 1},                  // 上/下行补偿初值Nm；小P抬升时驱动角增大
     {Motor::M2006, 1, ARM_ROLL_C610_ID, 0, 36.0f, 1, -90, 90, 30, 30, 3000, 10.0f, 45.0f, 8.0f, 0, 0, 0, 0},
     {Motor::M2006, 1, ARM_GRIPPER_C610_ID, 0, 48.0f, kGripperDirection, 0, 45, 3.75f, 11.25f, 500, 2.0f, 80.0f / 3.0f,
      20.0f / 3.0f, 0, 0, 0, 0,     1.5f, 1.5f}},
    false, // TODO: 根据实物确认后改true；默认仍收反馈、发送零输出/失能帧
    4,     // 五电机共用CAN1：250Hz闭环，避免1kHz指令+反馈超过总线容量
    50,    // 反馈失联保护
    250,   // 遥控协调层每4ms刷新目标；连续不刷新则停机
    5,     // DJI丢帧后多圈位置不再可信，必须重新校零
    {
        false,   // 已弃用：两pitch解耦，不再做联合映射确认
        -kPitchLimitDeg, kPitchLimitDeg, // 已弃用：不再使用a2=q肩+q肘组合范围
        5, 15,   // 已弃用：第二DM改用joint[SmallPitch]的独立速度/加速度
        8        //已弃用：不再要求两DM反馈时间差
    },
    ARM_FOUR_AXIS_BENCH ? kFourAxisJoints : kAllJoints};
} // namespace engineer
