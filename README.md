# RM Robosister 工程车

单块 RoboMaster C 板（STM32F407）运行 FreeRTOS。CAN1 为五轴机械臂（含 GM6020 yaw），CAN2 为两台 M3508 和功率计。机械臂同学维护两 pitch、roll、夹爪；你维护底盘和 yaw。每个电机只有一个闭环和 CAN 发送入口。

日常只使用一份整车固件：`Debug/Release` 是同一功能的调试/优化构建，均包含五轴、底盘、INS 和真实互锁。切换调参对象只改 FreeMASTER 变量，不换固件。`DmIdConfig` 只用于逐台修改 DM ID，仍是独立维护工具。

## 阅读入口

- 上车逐批操作：[00_分批调参步骤.pdf](fm调参/00_分批调参步骤.pdf)。
- 变量与故障：[01_排障与变量说明.pdf](fm调参/01_排障与变量说明.pdf)。
- 先抬臂再保持测轮：[02_机械臂抬起与保持.pdf](fm调参/02_机械臂抬起与保持.pdf)。
- 源码职责：[README_ARCHITECTURE.md](README_ARCHITECTURE.md)；参数：[README_CHASSIS.md](README_CHASSIS.md)、[README_YAW.md](README_YAW.md)。
- 遥控/关控：[README_REMOTE.md](README_REMOTE.md)；整车保护：[README_VEHICLE.md](README_VEHICLE.md)；机械依据：[README_ARM.md](README_ARM.md)。

`fm调参/`三个 `.pmpx` 是同一固件的不同观察页面，变量显示名使用源码符号、Group 使用中文。日常均加载 `build/Debug/dx_assignment5_freertos.elf`；板内程序必须与 ELF 同次构建。沿用现有通信方式，不增加 FMSTR 驱动。

## 一份固件内的调参档

| robot_tune_mode | 对象 | 目标变量 |
|---|---|---|
| 0 | 正常遥控，保留行驶/夹取/待机 | 遥控器 |
| 1 / 2 | 左 / 右轮电流测试 | `tune_current_ma`，mA |
| 3 / 4 | 左 / 右轮速度 PI | `tune_speed_rpm`，3508转子 rpm |
| 5 | yaw 速度内环 | `tune_yaw_speed_deg_s`，deg/s |
| 6 | yaw 位置双环 | `tune_yaw_target_deg`，deg |

`robot_tune_mode`选对象，`tune_enable`允许该对象测试；默认0，切档自动回0。顺序是：**enable=0 → 等实际停稳 → 切mode → 确认目标归零（位置档对齐当前角）→ enable=1 → 再写小目标**。调参时左拨杆下、右拨杆中、五通道回中。详细条件和每批观察量按原PDF执行。

`tune_enable=0`只停止调参对象，已抬起并使能的其他关节继续保持；切档不自动使能其他机械臂。`robot_tune_mode=0`不是总关控：退出调参后先保持，选择正常遥控模式、停稳回中，再写命令4明确恢复。全局关控用 `g_robot_debug_command=1` 或 `Robot_StopControl()`，遥控失联和故障保护始终有效。左拨杆上是行驶，不能沿用旧台架“左上关控”。

电流测试非零输出最多500ms，轮组测试上限300mA/300转子rpm，yaw速度测试上限5deg/s、测试限流上限150mA；这些上限不表示实物已安全整定。先低电流验证方向，再调闭环。故障恢复不自动续跑，先排除原因、按需清错，回0档后重新选择和使能。

## 总线与当前范围

| 对象 | 总线 | 电机ID / 反馈 / 输出 |
|---|---|---|
| GM6020 yaw电流模式 | CAN1 | ID1 / 0x205 / 0x1FE第一槽 |
| 大 / 小pitch DM4310 MIT | CAN1 | ID1、2 / 0x11、0x12 / 0x001、0x002 |
| roll / 夹爪M2006 | CAN1 | ID3、4 / 0x203、0x204 / 0x200第三、四槽 |
| 左 / 右M3508 | CAN2 | ID1、2 / 0x201、0x202 / 0x200第一、二槽 |
| 功率计 | CAN2 | 标准0x31、DLC4，小端有符号int16×0.1W/J |

两路均为1Mbps。yaw使用6020编码器和转速控制相对底盘角，底盘IMU用于姿态/翻车判断。无滑环，须人工对齐线束中位并设置已测软限位；校零和在线参数只存RAM。保留硬件、映射、Yaw、IMU、功率和姿态的默认确认门槛，不为让电机转而全部改真。

底盘已有差速、速度映射加斜坡、轮速PI、最终提交值抗饱和、本地失联和发送故障停机。Yaw已有位置P＋速度PI、真实角度边界制动和目标领先限制。正常模式使用真实互锁、IMU与保守功率停车；快速许可关闭，精细功率模型和满载性能留待后续。末端逆解、自动抓取、接触检测和重力补偿未实现。

## 构建和检查

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
python -m pip install --target build/test_deps -r tests/requirements.txt
python tests/run_tests.py
python tests/check_architecture.py
```

日常烧录 `build/Debug/engineer_vehicle.bin`（或同目录hex），加载同目录ELF；优化构建对应 `build/Release/`。测试默认整车，`--bench`仅用于兼容逻辑回归，不对应日常固件。DM改ID按 [README_DM_ID.md](README_DM_ID.md) 执行，结束后烧回整车。

编译和合成输入测试不能代替真实方向、线束、负载保持、翻车门控和停车距离验收。项目不自动烧录。旧构建目录和历史统计不作为当前固件依据。原教学模板保存在 `docs/TEMPLATE_README.md`；agent先读 [AGENTS.md](AGENTS.md)。本轮只做本地Git提交，不push到 [共享仓库](https://github.com/dudu070603/robosister)。
