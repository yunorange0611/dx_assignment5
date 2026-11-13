# Yaw固定车头零点（当前入口）

失能并对正车头后，在FreeMASTER「第6批 Yaw线束中位与软限位」分组读取 `g_arm_status.joint[0].raw_encoder`（0..8191）。将该整数填入 `2User_Driver/Arm/yaw_config.h` 的 `kYawZeroEncoder`，重新编译、烧录并刷新FM符号。默认-1表示尚未填写，保留原人工校零；不把0当作已测零位。

填写后首次反馈自动建立相对固定零点的坐标，`fixed_zero_encoder` 显示生效配置，`calibrated=1`；命令2不会覆盖固定Yaw零点。其他四轴仍需原校零。断电不得额外绕整圈，固定零点方案适用于零点附近小于±180°的实际行程；拆装/跳齿后重新标定。

0档用命令5仅使能Yaw（6仅失能）；或原整车开控入口。成功使能后 `homing=1`，参考角从当前位置按yaw_config.h速度/加速度向0运行，到零点附近后接受位置/遥控目标。mode5速度环调试显式接管回零。未开控时不出力，现有开控确认、反馈和关控条件保留；命令写入不代表使能成功，看last_result/enabled。独立遥控需左上+右中+回中解锁，不能用上档代替关控。

## 独立底盘左横直控

当前 `kChassisIndependent=true` 时，0档解锁后（`independent_armed=1`、左拨杆上）左横直接积分 Yaw 目标：`yaw_direct_target_deg` 从当前角开始，按 `max_speed_deg_s` 增长，限制使用 `arm.yaw_limits()` 的线束软限位；未配置限位时退回 `kYawConfig` 的 -90..90°/10deg/s。该路径不使用正常行驶姿态包络，不能称为碰撞保护。

Yaw 必须在线、已校零、已使能且无故障，直控才生效；固定零点轴首次开控会先回零（`homing=1`），回零期间左横不推进目标，回零完成后再接受直控。未使能时底盘照常驱动，Yaw 不动。只使能 Yaw 用 `g_robot_debug_command=5`，只失能用6，不影响其他关节。激活时目标从当前角开始；松开左横保持当前目标，不回中。关控、翻车、遥控失联、功率停车或离开上档会立即撤销直控，Yaw 按统一策略失能（通常不主动回中）；恢复要重新回中解锁并使能Yaw。

观察 `g_robot_status.yaw_direct_active`、`yaw_direct_target_deg`，以及 `g_arm_status.joint[0]` 的 `raw_encoder/fixed_zero_encoder/homing/position/target/velocity/effort`。

静态软限位在yaw_config.h的min_deg/max_deg；在线对称半幅为tune_yaw_half_range_deg，仍要求kYawSafetyConfirmed和有效反馈/校零。限位未确认默认不放行，不改机械臂其他参数。

以下为历史说明，涉及人工Yaw校零及tune_enable的操作已由以上入口替代：

# Yaw限位与运行时调参

CAN1 GM6020电流模式，ID1，反馈0x205，输出0x1FE第一槽。Yaw由原 `ArmControl / Robot_Execute / arm_task` 闭环并发送，没有第二个驱动或线程。与底盘共用 `Debug/Release` 整车固件，mode5/6选内环或双环，不换固件。

C板在底盘，Yaw相对角和速度来自6020编码器/rpm，不能用底盘IMU代替。没有滑环，断电后人工整理线束并对齐可重复的自然中位，再显式校零；任意上电姿态不当零点，软件不能识别断电期间额外扭转的整圈。

## 修改入口

| 内容 | 入口 |
|---|---|
| 模式、ID、比例、方向、静态范围与确认 | `2User_Driver/Arm/yaw_config.h` |
| 制动余量/减速度、目标领先约束 | 同文件Yaw限位参数 |
| 在线线束半幅 | `tune_yaw_half_range_deg` |
| 在线位置P、速度PI | `tune_yaw_position_kp`、`tune_yaw_velocity_kp`、`tune_yaw_velocity_ki` |
| 测试输出上限 | `tune_yaw_current_limit_ma`，mA |
| 速度/位置目标 | `tune_yaw_speed_deg_s` / `tune_yaw_target_deg` |
| 独立底盘直控使能/失能 | `g_robot_debug_command=5` 只使能Yaw；`=6` 只失能Yaw |

`kYawSafetyConfirmed`等确认默认false。约1:1传动、方向、实际软限位、可靠制动与电流须逐项实测，不能把±90deg占位当安全行程。在线参数只存RAM，确定后写回源码。

## 线束中位和半幅

全局关控并确认失能，整理线束到人工中位并保持静止。命令2在mode5/6只校Yaw，在0档针对全部五轴；两pitch如参与必须联合校零。检查 `g_arm_status.last_result`，不能只看命令已清0。

直接写 `tune_yaw_half_range_deg`，范围为[-half,+half]；原owner在数值变化时检查后应用，日常不用填限位序号。要求关控、反馈新鲜已校零且静止，范围合法且不超过已确认静态上限，实际角/目标/参考均被包含并留足制动余量。看 `g_yaw_limit_status.result/ready/active_half_range_deg`。拒绝后不迟到执行，修正原因后需再次明确修改输入；首次有效限位前正常Yaw/整车放行禁止。

生效版本同时用于遥控目标、姿态包络与内层限位；任务间用 `Yaw_GetLimitStatus()`，`ArmStatus.updated_ms`和`yaw_limit_version`用于核查发布时间与核心版本。不写只读状态强行放行。

## 速度内环与位置双环

| robot_tune_mode | 功能 | 目标 |
|---|---|---|
| 5 | 跳过位置外环，调速度PI | `tune_yaw_speed_deg_s` |
| 6 | 位置P＋速度PI | `tune_yaw_target_deg` |

底盘停止、机构有支撑；调参不再由固件强制拨杆位置，但上车应保持右拨杆中、五通道回中并等实际静止。按 **速度0/位置对齐当前角 → 切mode → 确认目标0或对齐当前角 → 写小目标**；没有单独 enable，非零速度或位置目标即执行，写0停止。切档会清历史并重新对齐位置目标。速度测试最大5deg/s，测试电流最大150mA（默认100mA）且受编译/已生效上限约束；目标须在生效线束软限位内。

改参数先把速度写0或位置对齐当前角、等稳，再改P/PI/限流，查看 `g_robot_tune_status` 参数结果和实际值。真实角边界、目标领先、电流和反馈保护继续生效。速度环稳定后在同固件切6档调位置P。

把速度/位置目标写0或回0档停止Yaw调参运动，已使能Yaw按现有策略回到当前位置保持，其他已使能轴保持；切档不会自动使能其他四轴。全局关控/失联/故障按核定策略失能，零输出不是机械锁止。退出先目标0、停稳、mode=0，再正常模式停稳回中后命令4恢复遥控。

## 限位与输出边界

核心按真实角/速度、保守减速度和提前余量限制向外速度，保留朝内制动；越界故障余量与提前制动余量不同。行驶包络同时约束目标、参考和内层速度；新包络排除实际角/目标/参考时请求停车，不强拉回边界。未确认轮静止前不放宽已有较窄包络。

PI按最终安全/电流限幅后的提交值抗饱和，发送失败由原owner锁存并取消旧帧。限速不等于已验证制动距离，入队不等于电机执行。带载前双方确认Yaw撤力后的整臂/线束行为。

逐批操作见原 [00_分批调参步骤.pdf](fm调参/00_分批调参步骤.pdf)，异常查 [01_排障与变量说明.pdf](fm调参/01_排障与变量说明.pdf)。旧独立Yaw固件和短脉冲请求不再是当前入口。
