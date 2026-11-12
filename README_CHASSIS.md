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
| 加减速/转向斜坡 | `acceleration_m_s2`、`deceleration_m_s2`、`angular_accel_rad_s2`、`angular_decel_rad_s2` |
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


### 2026-11-11 底盘调参放行简化
单轮调参(mode 1–4)不再依赖mapping_confirmed、hardware_confirmed、机械臂故障/校零及IMU就绪；这些条件仍用于原正常整车路径。保留遥控在线、明确关控、任务时序、轮反馈/温度、CAN故障、参数合法性、限流和电流脉冲超时。先架空调轮，不视为整车放行。mode只选对象，tune_enable控制执行；切对象仍清零一次。


### 2026-11-11 目标直接执行
robot_tune_mode仅选择对象/控制层。tune_enable为旧FM兼容符号，不再控制执行。先选mode（切档清目标一次），再写电流/速度目标，非零即执行，归零停止。电流500ms超时仍保留；超时后需把目标写0，待状态解除，再给新目标。mode6为yaw位置保持，进入时目标捕获当前角度，不用enable。全局关控、失联、设备反馈与输出限制不变。


2026-11-11：tune_enable全局符号已彻底删除，mode选对象后直接给目标。电流档仍保留500ms超时，超时会清目标；速度档无该脉冲超时。FM必须使用新ELF刷新绑定，不可沿用删除符号后的旧表。


2026-11-11：按实际轮子识别交换左右映射：左轮CAN2 ID2、方向-1；右轮CAN2 ID1、方向+1。mode1/3选择实际左轮，mode2/4选择实际右轮。


2026-11-11实测修正：原配置索引0(ID2)实际为右轮，索引1(ID1)实际为左轮。现改左ID1/方向+1、右ID2/方向-1，接收及输出均按同一motor_id数组映射。单轮调参PI与限流合法参数直接在线生效，不再要求目标归零或静止，也不插入失能周期；积分限制到新电流界限。调参电流硬上限仍300mA，速度目标仍限±300转子rpm。


2026-11-11：按用户要求，M3508/C620每轮输出上限改为10000mA（20A控制量程一半，非两轮总和）。单轮电流目标范围与默认调参电流上限同步10000mA，移除300mA调参钳位。速度目标仍±300转子rpm，电流脉冲500ms、功率保护/故障处理不变。上限不等于实际输出，速度PI仍按误差计算电流。


2026-11-11：去掉FM速度和轮速PI的人工输入范围，浮点采用直接输入及6位小数显示；固件单轮速度上限由300改为现有转子上限9000rpm（ChassisConfig.max_rotor_rpm）。PI仍要求有限、P>0、I>=0，固件保留原增益合法范围及10A输出上限，不允许NaN/Inf。9000为边界而非调试起步建议。


轮速PI实测记录：Kp=3.4、Ki=1.3，已同步正常控制默认值与FM调参初值。移除RobotTuning失败后必须零目标解锁的历史锁存；故障仍禁止输出并由owner清目标，条件恢复后直接提交新目标即可。电流500ms超时仍保留。


### 2026-11-11 0档底盘直驱（交接用）
- `ChassisConfig.h` 的 `kChassisIndependent=true` 时：`robot_tune_mode=0`、左拨杆上、遥控在线，底盘直接使用共用解析值 `remote_axis[0]`（左竖）和 `remote_axis[3]`（右横）驱动；不再要求机械臂 active、Yaw 半幅、IMU 或功率计。
- 左拨杆中/下、松杆、遥控失联、命令1：停止/制动。
- 遥控解析、通道、方向、死区仍共用；机械臂/Yaw 代码不参与底盘输出。
- `ChassisCommand.independent` 是内部标记；`g_chassis_status.independent` 为只读状态。FreeMASTER 的“0档底盘直驱｜左拨杆上”Group 用于观察。
- 以后整车联调时把 `kChassisIndependent` 改回 false，恢复 `Robot_GetChassisCommand()` 的整车许可路径；底盘控制算法和 CAN2 发送入口不变。

### 行驶斜坡在线调参
FreeMASTER「行驶加减速与转向」组的五项 `tune_chassis_*` 在正常底盘控制周期生效：行驶加速/减速、转向角加速度/角减速度、最大转向角速度。只接受有限正数，非法输入保留上一生效值。默认值来自 `chassis_config.h`，在线修改不保存到 Flash，不影响单轮测试、限流或停止保护。

转向松杆和回中时按 `angular_decel_rad_s2` 收参考角速度；同时 `ChassisControl::step` 在无行驶/转向请求（`stop`，或 `brake` 且转向请求为0）时每个周期清零两轮速度环积分，避免积分在静摩擦下冻结、松杆后仍持续驱动一侧轮子。该清理不作用于纯转向中的 `brake`（转向请求非0时保留积分）。

### 功率限制（代码已接入，默认未启用）
`ChassisPowerConfig.confirmed` 默认 false，只有改源码重编为 true 后才参与控制。启用后：
- 滤波后的 `power_w`（`power_filter_ms`，默认300ms）超过 `budget_w`（默认45W）时，按比例降低**速度目标**，不砍驱动电流；`max_current_ma`/`drive_current_ma` 只做电机/C620安全底。
- 缓冲能量低于 `warning_energy_j`（默认300J）开始降速，低于 `stop_energy_j`（默认80J）受控停车，恢复需 `restart_energy_j`（默认350J）。
- `braking_current_ma` 默认8000，避免下坡/功率停车制动太弱。
- FM「功率限制（当前未启用）」组的 `tune_chassis_power_budget_w`、`tune_chassis_power_filter_ms` 只改RAM参数；`confirmed` 必须改源码重编。
- `g_chassis_status.power_speed_scale` 是功率/能量限速比例，`power_filtered_w` 是滤波后功率，用于现场判断是否在降速。
- 完整参数表、调参步骤和现场记录表见 [功率限制调参说明.md](fm调参/功率限制调参说明.md)。

### 独立底盘翻车保护（2026-11-12）
- 独立底盘仍不要求 IMU 安装/阈值确认，也不要求机械臂/Yaw/功率确认；但 `PROTECT_TIPPED` 会撤销独立底盘命令并置 `independent_stop_latched`。
- 翻车判定来自 IMU `tipped`；只有 IMU `valid`（已校准、无故障）时才更新。IMU 未校准时 `tipped` 不更新，独立底盘仍可调；先做静态标定。
- FM「翻车保护｜独立底盘」组可在线调 `tune_attitude_trip_deg`、`tune_attitude_release_deg`、`tune_attitude_trip_ms`、`tune_attitude_release_ms`；只存 RAM，断电恢复源码默认 55°/40°/150ms/500ms。
- 触发后 `independent_stop_latched=1`，用命令4或 `Robot_RequestControl()` 清除；命令1只是关控，不会清这个锁存。清除前即使倾角恢复、拨杆仍在上也不会自动恢复。
- IMU 的 `installation_confirmed` / `thresholds_confirmed` 仍是 false；比赛前必须实测方向、阈值和解除角，不能只靠默认 55°/40°。
