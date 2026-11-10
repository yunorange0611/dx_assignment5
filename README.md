# RM Robosister 工程车

> 当前已在三批功能基础上完成三层架构重构，并将底盘控制请求与整车诊断快照分开。结构与调用关系先看 README_ARCHITECTURE；调参依据为 README_CHASSIS、README_YAW、README_VEHICLE。所有实测确认默认关闭，尚未烧录或实物验收。本轮只做本地Git提交，不推送。

单块 RoboMaster C 板（STM32F407IGH6）运行 FreeRTOS，使用 DR16 遥控协议 V2.1。两位电控同学共同维护同一份固件：机械臂负责人调试大/小 pitch、roll、夹爪；底盘/yaw 负责人调试两台 M3508 和大 yaw GM6020。

分批调参PDF、三个中文分组FreeMASTER工程及固件分组切换工具见 `fm调参/`；默认对应 `ChassisBenchDebug`。支持失能静止时申请在线PI/限流，以及同一底盘悬空固件内先抬臂、后待机保持测轮；实际负载、方向和停车仍须按步骤实测。

开始修改前，agent 先读 [AGENTS.md](AGENTS.md)。本文是项目入口；三层职责、实际调用顺序及旧驱动退出范围见 [README_ARCHITECTURE.md](README_ARCHITECTURE.md)。详细控制依据见 [README_ARM.md](README_ARM.md)，四轴台架操作、调试变量、整车遥控和底盘接口统一见 [README_REMOTE.md](README_REMOTE.md)。教学模板的原始说明完整保存在 [docs/TEMPLATE_README.md](docs/TEMPLATE_README.md)。

缺少DM的3Pin调试线时，可用独立 `DmIdConfig` 固件，通过C板CAN1逐台改两台4310的ID；上电不自动写参数，读取、RAM改ID、保存分别手动触发。接线、FreeMASTER最少变量、ELF位置和掉电复验见 [README_DM_ID.md](README_DM_ID.md)。配置完成须烧回正常四轴/整车固件。

## 分工与文件

| 负责人 | 内容 | 主要入口 |
|---|---|---|
| 机械臂负责人 | 大/小 pitch 两台 DM4310、roll 和夹爪两台 M2006 | `2User_Driver` 中四轴参数；`arm_control.cpp` 的传动与闭环 |
| 底盘/yaw 负责人 | 两台 M3508 的差速底盘与大 yaw GM6020 调试 | `Chassis/`、`Power/`、`Imu/`、`Vehicle/` 与 `Arm/yaw_config.h` |
| 共同维护 | DR16、模式解释、关控保护、CAN、互锁 | `2User_Driver`、`3Task`、`1Bsp` |

大 yaw 参数独立维护，运行时仍由 `ArmControl` 和 `arm_control_task` 统一闭环并发送 CAN1。一个电机只保留一个控制和发送来源；不要为 GM6020 再构造旧 `DJI_motor` 对象或再开一个发送任务。共享代码修改应说明对另一方的影响，尽量避免同时编辑同一文件。

## 当前完成范围

- CAN1：GM6020 电流模式、两台 DM4310 MIT 纯力矩、两台 M2006 电流控制，共五轴，250 Hz 位置 P + 速度 PI。
- 四轴台架：日常 `Debug/Release`（保留 `BenchDebug`）只管理两台 DM4310、roll 和夹爪 M2006，不依赖 yaw 或底盘互锁；左拨杆上关控、中夹取、下待机，健康的上→中动作加连续回中 300 ms 可请求一次开控。
- 机械臂：手动校零、限速/限加速度、pitch 联动换算、夹爪传动比 48、目标/反馈超时及故障停机。
- 遥控：行驶、夹取、待机。行驶左横调 yaw、右横请求底盘转向；夹取左竖肩、右竖肘、左横 roll、右横 yaw。右拨杆行驶上/中慢、下快；夹取上开、中保持、下闭。
- 保护：默认失能、关控优先、取消 CAN1 旧指令、恢复通信后必须回中并明确重新开控。正常待机持续刷新已开控关节的保持目标。
- CAN2：两台M3508/C620反馈、差速、速度映射加斜坡、轮速PI与最终提交值抗饱和、本地超时/发送故障停机，以及功率计解析和保守限幅/有时限停车。用户确认左轮ID1、右轮ID2；方向、有效几何、齿比、PI及停车距离待实测。
- Yaw：原CAN1 owner内统一生效的在线对称半幅、真实角度边界减速、目标领先限制及独立YawBenchDebug短脉冲调参；不新增第二个驱动或发送者。
- 整车：BMI088静态标定、安装映射、上向量倾覆检测、带源时间的真实机械臂/轮速/能量互锁和统一锁存关控。快速许可恒关；精细功率模型和满载性能留后续。

末端 XYZ/逆运动学、自动抓取、接触检测、恒夹持力、重力补偿和整臂碰撞检测尚未实现。

## 总线与接口约定

| 对象 | 总线 | 约定 ID / 报文 |
|---|---|---|
| 大 yaw GM6020 | CAN1 | 用户确认电机 ID 1；反馈 `0x205`；电流指令 `0x1FE` 第一槽 |
| 大/小 pitch DM4310 | CAN1 | 电机 ID 1/2；反馈 `0x11/0x12`；MIT 指令 `0x001/0x002` |
| roll / 夹爪 M2006 | CAN1 | C610 ID 1/2；反馈 `0x201/0x202`；共用 `0x200` 第一/二槽 |
| 两台 M3508/C620 | CAN2 | 用户确认左ID1/右ID2，反馈0x201/0x202，指令0x200第1/2槽，方向待实测 |
| 功率计 | CAN2 | 标准0x31，DLC4，signed int16小端×0.1W/J |

CAN1、CAN2 均为 1 Mbps。CAN1 同一指令 ID 不能由多个任务分别发送；两条总线可以独立使用同号 ID。底盘通过 `Robot_GetChassisCommand()` 直接取得现有 `ChassisCommand`，消费运行许可、停车/复位、速度/转向、源时间和停止事件；不再读取整车大快照再转换。`RobotControlStatus`保留诊断及INS校准状态来源。`vehicle_state_task`继续聚合真实底盘、机械臂和INS状态；互锁超过100 ms未更新会停止控制。底盘本地检查请求时效、`epoch/stop_epoch`及CAN2故障，四轴台架始终发布底盘失能请求。

功率计已接CAN2。50W/500J为规则边界，首版使用40W预算占位及低缓冲预留，推进和制动分别限流；这不是精细功率模型或严格瞬态功率保证。功率反馈超时/低能量而轮反馈健康时先有时限停车，轮/总线故障立即撤力；参数和距离须实测。

## 调试边界

`hardware_confirmed`、`pitch.mapping_confirmed`、遥控 `mapping_confirmed` 默认均为 false，两个模式都不会默认使能电机。日常 `Debug/Release` 和原有 `BenchDebug` 均固定为四轴（`joint_mask=0x1e`），不注册 yaw 反馈，不发送 `0x1FE`，不依赖底盘在线；整车五轴使用显式命名的 `VehicleDebug/VehicleRelease`（`0x1f`）。两者使用同一闭环、校零与关控保护。

单独调机械臂时直接构建、烧录 `Debug`。固件烧入一次后，每次上电都保持四轴模式，无需按板载按钮或重新选择模式；每次重启仍须校零并明确开控。源码参数有改动时才需要重新构建、烧录。切回整车前重新核对包含 yaw 在内的硬件确认和真实底盘互锁，不能把四轴确认直接当作整车确认。

底盘/yaw 负责人依据当前 `yaw_config.h` 配置入口独立整定，并核验已接入的真实停车/姿态许可；旧车PI数值不直接沿用。机械臂负责人确认四轴机械对应、零位和行程。各参数实测确认、重新编译烧录后，先支撑机构重新校零，再按所选配置开控；台架步骤见 [README_REMOTE.md](README_REMOTE.md)，底盘/Yaw受限请求见各自README。校零只保存在 RAM，重启后需要重做。

正常待机维持已开控关节的位置闭环；关控/失联使当前配置全部轴失能。负载保持策略本轮暂不改。roll ±90°、夹爪 0..45°仍是暂定软件范围，roll 实际范围未知、夹爪没有机械限位，不能把占位值当作实测安全行程。0.3 Nm pitch 上限、夹爪 150 mA 保持电流均未完成承重验证。

调试复用原快照，跨任务使用Get接口。原 `g_robot_debug_command` 负责关控、校零、清故障与开控；新增有序号的一次性底盘/Yaw受限请求和Yaw半幅请求，详见新增README。沿用用户现有可用读写方式，不修改其工具配置或通信栈。

## 构建和测试

需要 PATH 中有 STM32CubeCLT 的 `arm-none-eabi-*`、CMake、Ninja；逻辑测试另需 Python 3。

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset VehicleDebug
cmake --build --preset VehicleDebug
python -m pip install --target build/test_deps -r tests/requirements.txt
python tests/run_tests.py
python tests/run_tests.py --vehicle
python tests/run_tests.py --yaw-bench
python tests/check_architecture.py
```

`ARM_FOUR_AXIS_BENCH` 默认 ON；`Debug/Release/BenchDebug` 明确 ON，`VehicleDebug/VehicleRelease` 明确 OFF。四轴日常输出在 `build/Debug/`，整车在 `build/VehicleDebug/`：固件为 `engineer_arm.bin/.hex`，调试符号为 `dx_assignment5_freertos.elf`。原 `BenchDebug` 仍输出到 `build/BenchDebug/`。测试默认四轴，`--vehicle` 测整车，保留 `--bench` 作为显式四轴选项。烧录和调试必须使用同一配置的文件。`firmware_disabled/` 是旧本地固件副本，请从当前源码重新构建；构建输出、IDE 文件和本地测试依赖已由 `.gitignore` 排除。

2026-11-10 架构重构阶段历史复验（本轮请求收拢前）：四轴46,036、整车46,064、YawBench46,018条断言通过；Debug、VehicleDebug、VehicleRelease、ChassisBenchDebug、YawBenchDebug、DmIdConfig六种固件编译链接通过。VehicleDebug为FLASH 93,336 B/RAM 58,976 B，VehicleRelease为FLASH 51,520 B/RAM 58,976 B。架构检查通过，13个业务公开头独立编译，业务源不提供Task/HAL/RTOS搜索路径也可编译。旧DJI/pid不再进入构建；测试ELF的RWX段警告仍在。测试使用ARM机器码、合成输入与HAL/RTOS替身，未烧录/实物待验。

以下为重构前历史记录，不代表当前固件占用：2026-11-10 固定四轴默认配置复验时，`Debug`（四轴）与 `VehicleDebug`（整车）均构建通过，各使用 Flash 65,144 B、RAM 58,624 B；四轴 42,584 条、整车 42,585 条断言通过。软件测试不覆盖真实 UART/CAN、方向、负载保持或底盘制动。

同日新增独立DM ID配置模式后，三种Debug构建均通过；加入配置协议测试后，四轴44,002条、整车44,003条断言通过。原四轴/整车的Flash与RAM占用不变，配置固件Flash 40,020 B、RAM 56,616 B。完整操作见 [README_DM_ID.md](README_DM_ID.md)。

共享仓库：[dudu070603/robosister](https://github.com/dudu070603/robosister)。本轮用户明确要求每批本地Git提交，不push。协作时各自使用分支，合并前运行构建和相关测试，不提交本地构建产物或IDE缓存。
