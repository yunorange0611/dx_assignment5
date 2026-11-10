# 两轮底盘：参数与运行时调参

CAN2左轮ID1、右轮ID2，反馈0x201/0x202，输出0x200第一/二槽；CAN1由原机械臂任务管理。日常构建 `Debug/Release`，同一整车固件通过 `robot_tune_mode` 和 `tune_enable` 调参。

正常控制链为 `Robot_GetChassisCommand → ChassisControl → ChassisPower → ChassisMotor组帧 → CAN2`。摇杆直接映射目标速度再限加减速，`speed_request/turn_request`为[-1,1]；前进和逆时针为正，倒车不反转转向，不做Yaw跟随。

## 修改入口

源码参数在 `2User_Driver/Chassis/chassis_config.h`。实测确认默认关闭，只确认已经核对的项目。

| 内容 | 字段 |
|---|---|
| 电调ID、正方向 | `motor_id[2]`、`direction[2]` |
| 有效轮径、轮距、总齿比 | `diameter_m`、`track_m`、`gear_ratio` |
| 慢速/转向上限 | `max_speed_m_s`、`max_turn_rad_s` |
| 加减速/转向斜坡 | `acceleration_m_s2`、`deceleration_m_s2`、`angular_accel_rad_s2` |
| 轮速PI/运行限流 | `velocity_kp`、`velocity_ki`、`max_current_ma` |
| 超时、轮静止确认 | `feedback_timeout_ms`、`command_timeout_ms`、`stationary_rpm`、`stationary_ms` |

FreeMASTER日常直接修改 `tune_wheel_kp`、`tune_wheel_ki`、`tune_wheel_current_limit_ma`，在测试停止、目标归零和新鲜静止反馈满足时由原owner校验生效；查看 `g_chassis_tune_status`、`g_chassis_status.tune_result`，不改只读状态。测试限流还受300mA及编译上限约束。参数只存RAM，确定后写回源码。底盘没有独立快速档参数，快速请求不会使车变快；功率参数在 `Power/chassis_power.h`。

## 同一固件内测轮

| robot_tune_mode | 测试 | 目标 |
|---|---|---|
| 1 / 2 | 左 / 右轮电流 | `tune_current_ma`，mA |
| 3 / 4 | 左 / 右轮速度PI | `tune_speed_rpm`，3508转子rpm |

两轮悬空并固定车体。需要抬臂时先在0档正常夹取，抬起后左拨杆下进入待机、右拨杆中、五通道回中并等实际静止。已使能轴继续保持，未使能轴不因选轮调参而自动使能。

每次按 **enable=0 → 等轮子停稳 → 选择mode → 确认电流/速度目标为0 → enable=1 → 写小目标**。切mode自动关闭 `tune_enable` 并清原目标/PI历史；只给选中轮输出，另一轮槽位0。先电流确认正负方向，再Ki=0调速度P，最后少量加I。速度单位是转子rpm，不是轮轴rpm。

`tune_enable=0`停止轮测试而不撤机械臂保持。模式1/2非零电流最多500ms，不能靠改幅值或符号延长。失联、反馈/温度/时序异常、发送失败、全局关控和本地故障均能撤销测试；健康恢复不自动续跑。回0档清本次调参锁，再按需清本地硬故障、重新选档和使能。模式5/6调Yaw时两轮禁止输出。

功率未实测确认时仅允许上述受限悬空调轮，不代表可以落地行驶；一旦功率配置确认，数据超时、低能量和禁止运行继续约束调轮。本地CAN2停止/清错不直接撤销CAN1机械臂保持；正常行驶限功率制动见 [README_VEHICLE.md](README_VEHICLE.md)。

## 退出、清错与观察

先enable=0、等实际停稳，再mode=0。0档不是总关控；机械臂继续保持，正常遥控需停稳回中后写 `g_robot_debug_command=4` 明确恢复。全局关控始终用命令1或 `Robot_StopControl()`。

`g_chassis_debug_request`仅保留本地维护：mode0停止、mode1清错，最后递增request_seq；旧mode2/3、wheel/duration/confirm不再是日常出力入口。清错不自动使能，需核对硬故障和功率锁存。日常选轮/给目标无需填这组请求。

观察 `g_chassis_status` 的反馈源时间、measured_rpm、target_rpm、current_ma、faults、tune_mode/tune_blocked及功率状态。current_ma是队列接受的最终量化值，不是电机执行确认；PI按最终提交值抗饱和。轮静止确认不能检测抱死滑移。

每批步骤和异常处理以原 [00_分批调参步骤.pdf](fm调参/00_分批调参步骤.pdf)、[01_排障与变量说明.pdf](fm调参/01_排障与变量说明.pdf) 为准；三个FreeMASTER工程使用同一Debug固件ELF。软件验证不能替代实物方向、停车距离与负载验收。
