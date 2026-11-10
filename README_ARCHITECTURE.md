# 当前三层结构与阅读入口

本轮按project4_1中“任务只组织流程、设备负责报文、控制模块负责计算”的方式重构；保留这台工程车的单板、两轮差速、无滑环GM6020和机械臂协作要求。project4的大型云台任务、旧车两板通信和调参值不迁入。日常四轴和两种悬空固件继续隔离，确认标记仍默认false。

## 三层分别放什么

| 层 | 文件/目录 | 职责与依赖 |
|---|---|---|
| 3Task任务与应用协调 | arm_task、robot_control、yaw_service、chassis_task、task_ins、vehicle_state_task、task_protect、dm_id_config_task | 读写快照、消费请求、RTOS周期、保护协调、调用业务核心和硬件服务。跨任务只走公开接口。yaw_service是原arm_task内的适配函数，没有新建线程。 |
| 2User_Driver控制与设备协议 | Arm、Remote、Chassis、Power、Imu、Vehicle | 参数、报文解析/编码、运动学、轨迹与PI、门控状态机、姿态融合和纯安全判断。不包含任务头，不访问任务全局或HAL/RTOS。 |
| 1Bsp板级服务 | CAN_bsp、dr16_bsp、dr16_protocol、bsp_imu、bsp_dwt | 中断、硬件采样、按值收发、接收回调、源时间。DR16解码随接收协议归入本层，不再让BSP反向包含上层。Core与HAL负责芯片初始化。 |

有状态的业务模型由对应owner持有，不靠跨层extern访问内部变量。只读诊断全局保留原名字以便现有调试工具读取；写请求有独立入口，不能直接改状态快照出力。

## 从project4和project4_1借鉴了什么

| 参考入口 | 采用的组织方法 | 本工程落点 |
|---|---|---|
| project4_1的3Task/Src/task_chassis.c | 任务顺着“取指令、取反馈、保护、控制、统一组帧、状态”读完；数学与报文分别交给模块 | chassis_task的六步线性循环，控制在ChassisControl，电流编码在ChassisMotor |
| project4_1的motor_3508与chassis_control | 电机反馈/命令编码和整车运动学分开维护 | C620方向/量化归设备模型；两轮差速、斜坡和PI归控制模型 |
| project4的task_rc、drv_rc和gimbal_control | 接收、操作意图和闭环分开；调参有明确入口 | DR16板级解码、ArmOperator统一模式、ArmControl唯一五轴闭环，原owner内受限YawBench |
| project4的can_diag与两工程INS组织 | 保留总线/电机新鲜度诊断、独立姿态发布，输出前核查健康 | 既有CAN计数/源时间和各模块快照继续使用；Vehicle聚合健康，INS只收校准许可 |

本次借鉴职责划分，保留当前团队的C++业务模型与公开API，不做无益的C语言重写或目录改名。旧车四轮全向运动学、跨板0x310/0x311通信、DM大Yaw协议和自动撞限位找零不适合当前车，不迁入；PI数值仍按当前机构独立实测。project4的大云台任务也不作为新的合并目标。

## 顺着功能读代码

### 底盘正常控制

`chassis_task`的循环顺序为：

```text
receive_feedback       取CAN2队列帧，保留源时间，检查本地总线异常
read_command           直接取ChassisCommand，正常固件只接受停止或清错调试请求
control_and_limit      同一个差速/斜坡/轮速PI，再执行功率和最终安全限制
submit_output          chassis_motor统一编码，返回量化实际值，提交CAN并反馈PI
publish_status         发布真实反馈、许可、故障和队列接受的量化提交值
wait_next_cycle        阻塞到下周期，迟到时不连续补跑
```

这些私有函数及运行上下文留在chassis_task.cpp中。ChassisControl、ChassisPower和ChassisMotor仍各持有自己需要的状态，没有为缩短任务而新增五个调度文件。

协调入口直接发布现有 `ChassisCommand`，底盘通过 `Robot_GetChassisCommand()` 一次取得请求，不再经历 `RobotControlStatus → ChassisCommand` 的搬运。请求只含原运行/停车/复位、速度/转向、时间和必要事件；整车快照保留诊断及INS标定状态来源。`Robot_ReportArmState`在同一临界区撤销底盘请求，保留发送后发现故障的同周期覆盖。恒零的yaw_follow和历史API版本宏已删；这轮不增加动作枚举、不移动健康线程或再拆模块。

先看`Chassis/chassis_config.h`，再看`chassis_control.*`理解差速、斜坡、PI；电调报文/方向/量化看`chassis_motor.*`，功率协议看`Power/power_meter.*`，能量停车与制动限流看`Power/chassis_power.*`。悬空请求的纯门控在`chassis_bench.*`，任务只适配输入与发送。

### 遥控、机械臂和Yaw

```text
DR16接收/解码
    → Robot_Execute读取一次快照
    → yaw_service处理在线半幅并同步生效版本
    → TaskProtect检查配置、失联、整车保护
    → ArmOperator产生模式与目标
    → yaw_service按模式应用动态包络
    → ArmControl执行五轴核心
    → arm_task统一CAN1发送与状态发布
```

YawBenchDebug在处理在线半幅后进入独立的 `Yaw_ExecuteBench` 分支，由该分支调用保护和纯脉冲门控，不运行普通遥控模式计算；最后仍回到同一ArmControl和arm_task发送。

`robot_control.*`维护整车请求、停止优先、模式协调和通用发布。Yaw专属的在线请求、结果发布、半幅版本同步、包络适配与YawBench流程收拢到`3Task/yaw_service.*`。公开的g_yaw_*符号和请求序号语义保留。

`Arm/yaw_bench.h`只负责纯脉冲门控，替代原含义模糊的yaw_runtime.h；实际半幅/边界/双环仍由ArmControl负责。Yaw没有第二个驱动、CAN发送方或独立RTOS线程。四轴台架保持原收发屏蔽。

### 真实互锁与IMU校准许可

`vehicle_state_task`读取Arm、Chassis、INS快照，转换为VehicleInput和真实配置中的时限；`Vehicle/vehicle_state`只接收所需的时间、有效性和姿态量并返回VehicleState。它不认识ArmStatus、ChassisStatus、InsStatus或RobotInterlocks。任务再把结果发布为整车健康和互锁。

INS只负责采样、姿态与数据发布，接收带时效的校准许可，不再读底盘、机械臂和Robot状态。整车聚合层根据原先的三源新鲜、全部失能条件持续给许可；最老源已经消耗的时间不会在转交时重置。

每个出力owner在尝试使能前立即撤销许可，防止20ms聚合周期内沿用旧许可。撤销后，只有全部来源在撤销时刻之后更新且仍失能的快照才能重新授权；默认、陈旧、同毫秒旧状态均不放行。标定请求仍一次消费，运动/坏帧会打断连续静态窗口。

## 本次消除的反向依赖和冗余

- Vehicle不再包含四个任务头；只用自身精简输入输出，任务负责协议适配。
- CAN BSP不再include配ID任务，也不再用DM_ID_CONFIG_MODE选择业务。配ID固件在任务初始化时注册CAN1总线回调，正常固件沿用设备ID分发；帧检查、每次ISR最多3帧、按值队列和取消代号保留。
- 两路CAN的相同过滤器组装收敛到一个私有函数，发送/错误/代号机制仍为每总线独立。
- INS不再包含Robot/Arm/Chassis任务头。使能前的校准撤销由协调任务主动通知。
- C620量化反解从任务移到chassis_motor；台架条件判断移回纯门控。控制层不承担RTOS或CAN发送。
- 旧Motor_Driver和pid_driver保留学习参考，但从所有当前固件的显式源码清单排除。旧DJI服务的弱函数/RTOS残留声明及ioc任务条目一并清理，不再产生两套可误用的编译路线。
- CMake按BSP/业务/任务列明源文件，不再递归把整个目录加入编译。新增源文件必须明确加入对应清单。
- 业务头统一按`Arm/...`、`Chassis/...`、`Power/...`等模块路径引用；BSP/任务头按公开文件名引用。只暴露1Bsp、2User_Driver和3Task三个根目录，不再混用跨层`../`或给每个业务子目录添加搜索路径。
- 展开压缩的一行多语句，统一4空格缩进和100列格式；各控制PI保留原语义，不强行抽取量纲与抗饱和不同的公共PID框架。

## 构建与架构检查

`tests/check_architecture.py`会检查BSP/业务向任务层的反向include、跨层../写法与旧驱动误入构建；同时只提供业务和BSP根目录，让所有活动业务源和每个公开业务头单独通过编译，无法暗中依赖HAL、RTOS或任务头。

```powershell
python tests/check_architecture.py
python tests/run_tests.py
python tests/run_tests.py --vehicle
python tests/run_tests.py --yaw-bench
cmake --preset VehicleDebug
cmake --build --preset VehicleDebug
```

全部固件仍从同一工程配置构建，默认四轴、整车、底盘悬空、Yaw悬空和独立配ID入口不变。单板内部直接共享底盘请求类型，不再维护历史0x0203版本宏；诊断符号继续保留，加载对应新ELF读取。底盘方向/PI、Yaw线束中位/限位、IMU安装、带载撤力、限功率停车距离和慢速整车协作仍是实物待验项目，代码重构不代替上车验收。

## 本轮实际验证记录（此前架构重构，属于请求收拢前历史）

| 检查 | 本次结果 |
|---|---|
| 默认四轴逻辑 | 46,036条断言通过 |
| 整车逻辑 | 46,064条断言通过 |
| 独立YawBench逻辑 | 46,018条断言通过 |
| 分层检查 | BSP/业务无向上反向include；13个业务公开头独立编译，业务源不提供Task/HAL/RTOS搜索路径也可编译 |
| Debug四轴 | FLASH 67,568 B，RAM 58,592 B；编译链接通过 |
| VehicleDebug | FLASH 93,336 B，RAM 58,976 B；编译链接通过 |
| VehicleRelease | FLASH 51,520 B，RAM 58,976 B；编译链接通过 |
| ChassisBenchDebug | FLASH 81,420 B，RAM 58,744 B；编译链接通过 |
| YawBenchDebug | FLASH 60,696 B，RAM 58,664 B；编译链接通过 |
| DmIdConfig | FLASH 40,484 B，RAM 57,080 B；编译链接通过 |

逻辑测试执行ARM机器码，输入为合成数据，HAL/RTOS为替身；测试ELF保留RWX段警告。旧DJI/pid已经退出当前固件，不再产生其原构造顺序/offsetof警告。此次未烧录，未把轮组、Yaw、IMU和整车实测项标成通过。
