# Yaw 运行限位与独立调参

第二批在原 `ArmControl / Robot_Execute / arm_task` 内增强 GM6020 Yaw。CAN1、ID5、反馈0x209、电流命令0x2FE第一槽保持不变，没有第二个电机实例、闭环线程或发送者。日常四轴 `Debug/Release/BenchDebug` 完全不启用Yaw；专用调参构建为 `YawBenchDebug`。

当前请求/状态声明在 `3Task/yaw_service.h`，任务适配实现在 `yaw_service.cpp`，仍由原 `Robot_Execute` 调用；`robot_control.h` 兼容包含这些声明，原调试符号保留。纯脉冲门控在 `Arm/yaw_bench.h`。三层与调用顺序见 README_ARCHITECTURE.md。

## 先确认的参数

`yaw_config.h` 中 `kYawSafetyConfirmed=false` 保持默认。±90°、约1:1、方向、PI、电流及制动参数仍是待测占位；只有人工核对电机模式、线束自然中位、两侧安全行程和停止距离后才能有依据地确认。C板在底盘，Yaw相对角和速度来自6020编码器/rpm，不来自底盘IMU。

线束参考沿用显式人工校零，保存在RAM；重启/重烧后重新整理线束、对齐实物中位标记再校零。不要把任意启动姿态当零，也不能用软件识别断电期间额外扭转的整圈。

## 在线半幅的请求与生效

沿用现有可用的变量读写工具，按本固件ELF操作。先写 `g_yaw_limit_request.half_range_deg`，再递增 `request_seq`。对称区间为 `[-half,+half]`。只在现有owner内处理请求，要求全部轴关控、Yaw反馈新鲜且已校零、静止，参数有限/合法、不超过经确认的静态上限；实际角、现有目标和参考都必须被新范围容纳，并留出提前制动余量。

`g_yaw_limit_status` 是只读结果：`consumed_seq` 表示已消费请求，`active_seq` 表示实际生效版本，`result` 对应 `engineer::Result`，`ready` 表示已成功应用。被拒绝的请求不会等条件恢复后迟到生效，必须重新递增序号。缩小范围排除当前位置/目标/参考时保留旧限制，关控后人工安全回中再申请，不自动冲回新边界。参数不默认掉电保存。

有效版本同周期交给 `ArmOperator` 目标推进、`Robot_SetInterlocks` 包络检查和 `ArmControl` 内层。跨任务读取用 `Yaw_GetLimitStatus()`；`ArmStatus.updated_ms` 为发布源时间，`yaw_limit_version` 显示内核版本。首次成功应用之前，正常整车Yaw/整车开控保持禁止。

## 实际角度制动与动态限制

根据真实相对角、真实角速度、保守制动减速度及余量计算向外速度上限；真实角越接近边界，底层速度目标越小。预留一个控制周期的运动距离。朝内制动电流保留；越界故障余量 `limit_margin_deg` 与提前制动余量不是同一个参数。

行驶姿态包络继续由真实互锁提供，但现在同时限制内层速度目标和参考轨迹。新包络排除实际角/目标/参考时拒绝应用、请求停车处理，不自动跳到边界。目标和参考相对实际角的领先量有上限，外部直接调用目标API也不能绕过。

切到Grab/Standby或丢失Drive许可时，未确认驱动轮静止前保持已有较窄包络；静止后才按模式条件放宽，避免停车途中突然释放Yaw转角。驱动轮静止不等于整车绝对无滑移。

速度PI抗饱和使用本周期最终安全/电流限幅之后的值；发送失败仍通过原owner锁存故障并取消旧CAN帧。入队成功不等于已在实物执行。

## YawBenchDebug 受限调参

此构建使用同一五轴内核，但只允许单独使能Yaw；底盘任务、INS和整车放行任务不运行，其他四轴保持失能。必须支撑机构；Yaw撤力会自由转动，零电流不是机械锁止。正常整车和日常四轴不消费Yaw调参出力请求。

1. 按实物核对配置后重新构建；默认确认标记不为真时不会出力。
2. 关控、手动对齐线束中位、静止反馈在线，写原 `g_robot_debug_command=2`。在此专用构建中仅校零Yaw；读 `g_arm_status.last_result`。
3. 写入保守在线半幅请求，确认 `active_seq/ready`，不能直接使用未测的±90°。
4. 保持四杆回中、左拨杆上、右拨杆中，遥控数据新鲜，其他轴失能。
5. 写 `g_yaw_debug_request.mode=1`、`speed_deg_s`、`duration_ms`、`suspended_confirm=0x59415753`，最后递增 `request_seq`。速度绝对值最多5°/s，持续最多500ms，电流上限为配置上限与150mA中较小值。该入口仍经过真实角度边界、反馈/时序/故障保护，只跳过位置外环以便调速度PI。
6. 观察 `g_yaw_debug_status`：result0停止，1接受，2参数非法，3未就绪/忙，4已撤销/到期。请求进行中追加请求会取消旧脉冲，不能靠不断写序号续时。等待实际静止后再申请下一次。
7. `mode=0` 并递增序号可停止；显式 `Robot_StopControl()`、原调试关控命令1、遥控失联/偏杆/换拨杆、时序/反馈错误和到期也会撤销。恢复健康不续旧请求。

观测Yaw关节 `position_deg/target_deg/reference_deg/velocity_deg_s/velocity_target_deg_s/effort`、发布时戳、限制版本、锁存原因。先速度PI，后回普通整车构建调位置P；不要在正常固件里另开测试发送任务。软件测试不代替方向、制动距离、线束、负载和整臂撤力行为实测。
