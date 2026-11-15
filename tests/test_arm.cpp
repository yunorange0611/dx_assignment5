#include "Arm/arm_control.h"
#include "arm_feedback_fixture.h"
#include <cmath>
#include <cstring>
#include <limits>
using namespace engineer;

extern "C" {
volatile unsigned test_checks = 0, test_failed_line = 0, test_done = 0;
volatile unsigned test_can_failed_line = 0;
volatile unsigned test_operator_failed_line = 0;
volatile unsigned test_cad_failed_line = 0;
volatile unsigned test_protect_failed_line = 0;
volatile unsigned test_robot_failed_line = 0;
volatile unsigned test_four_axis_failed_line = 0;
volatile unsigned test_dm_config_failed_line = 0;
volatile unsigned test_chassis_failed_line = 0;
volatile unsigned test_yaw_failed_line = 0;
volatile unsigned test_vehicle_failed_line = 0;
volatile unsigned test_robot_tuning_failed_line = 0;
}
int test_can_tx();
int test_operator();
int test_cad_mapping();
int test_protect();
int test_robot_control();
int test_four_axis();
int test_dm_id_config();
int test_chassis();
int test_yaw();
int test_vehicle();
int test_robot_tuning();
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x)) {                                                                                \
            test_failed_line = __LINE__;                                                           \
            return;                                                                                \
        }                                                                                          \
    } while (0)
namespace {
Config bench() {
    Config c = kArmConfig;
    c.joint_mask = kAllJoints;
    c.hardware_confirmed = true;
    c.pitch.mapping_confirmed = true;
    // Bidirectional controller fixtures use an explicit synthetic travel range.
    c.joint[BigPitch].min_deg = c.joint[SmallPitch].min_deg = -150;
    c.joint[BigPitch].max_deg = c.joint[SmallPitch].max_deg = 150;
    // Synthetic PID/protocol cases use a load-free fixture. Production defaults
    // are checked separately against the measured holding-torque points.
    c.joint[BigPitch].torque_bias_nm = 0;
    c.joint[BigPitch].gravity_gain_nm = 0;
    c.joint[BigPitch].gravity_offset_deg = 0;
    c.joint[BigPitch].up_ff_nm = c.joint[BigPitch].down_ff_nm = 0;
    c.joint[SmallPitch].torque_bias_nm = 0;
    c.joint[SmallPitch].gravity_gain_nm = 0;
    c.joint[SmallPitch].gravity_offset_deg = 0;
    c.joint[SmallPitch].up_ff_nm = c.joint[SmallPitch].down_ff_nm = 0;
    return c;
}
void dji(ArmControl &a, unsigned joint, unsigned e, uint32_t ms, int rpm = 0, unsigned temp = 25) {
    uint8_t d[8] = {uint8_t(e >> 8), uint8_t(e), uint8_t(uint16_t(rpm) >> 8), uint8_t(rpm), 0, 0,
                    uint8_t(temp),   0};
    a.receive(1, a.feedback_id(joint), d, 8, ms);
}
void dm(ArmControl &a, unsigned joint, uint32_t ms, unsigned state = 0, unsigned temp = 25) {
    const unsigned id = joint == BigPitch ? 1 : 2;
    // 位置/速度/力矩都在中点附近。12位速度有量化误差，不等于精确0。
    uint8_t d[8] = {
        uint8_t((state << 4) | id), 0x80, 0x00, 0x80, 0x08, 0x00, uint8_t(temp), uint8_t(temp)};
    a.receive(1, a.feedback_id(joint), d, 8, ms);
}
void all(ArmControl &a, uint32_t ms, unsigned state = 0) {
    prime_roll_feedback(a, ms);
    dji(a, Yaw, 1000, ms);
    dji(a, Roll, 1000, ms);
    dji(a, Gripper, 1000, ms);
    dm(a, BigPitch, ms, state);
    dm(a, SmallPitch, ms, state);
}
void dm_at(ArmControl &a, unsigned joint, float degrees, uint32_t ms, unsigned velocity = 0x800) {
    const auto position = unsigned((degrees / 57.2957795131f + 3.14159f) * 65535 / 6.28318f + .5f);
    const unsigned id = joint == BigPitch ? 1 : 2;
    const uint8_t d[8] = {uint8_t(0x10 | id), uint8_t(position >> 8), uint8_t(position),
                         uint8_t(velocity >> 4), uint8_t((velocity << 4) | 8), 0, 25, 25};
    a.receive(1, a.feedback_id(joint), d, 8, ms);
}
const Frame *find(const Frame *frames, unsigned n, unsigned bus, unsigned id) {
    for (unsigned i = 0; i < n; ++i)
        if (frames[i].bus == bus && frames[i].id == id)
            return frames + i;
    return nullptr;
}
bool dji_zero(const Frame *f, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        if (f[i].id == 0x200 || f[i].id == 0x2ff || f[i].id == 0x1ff || f[i].id == 0x1fe ||
            f[i].id == 0x2fe)
            for (auto b : f[i].data)
                if (b)
                    return false;
    return true;
}
void run() {
    test_robot_tuning_failed_line = test_robot_tuning();
    CHECK(test_robot_tuning_failed_line == 0);
    test_chassis_failed_line = test_chassis();
    CHECK(test_chassis_failed_line == 0);
    test_yaw_failed_line = test_yaw();
    CHECK(test_yaw_failed_line == 0);
    test_vehicle_failed_line = test_vehicle();
    CHECK(test_vehicle_failed_line == 0);
    test_dm_config_failed_line = test_dm_id_config();
    CHECK(test_dm_config_failed_line == 0);
    test_protect_failed_line = test_protect();
    CHECK(test_protect_failed_line == 0);
    test_can_failed_line = test_can_tx();
    CHECK(test_can_failed_line == 0);
    test_robot_failed_line = test_robot_control();
    CHECK(test_robot_failed_line == 0);
    test_four_axis_failed_line = test_four_axis();
    CHECK(test_four_axis_failed_line == 0);
    test_operator_failed_line = test_operator();
    CHECK(test_operator_failed_line == 0);
    test_cad_failed_line = test_cad_mapping();
    CHECK(test_cad_failed_line == 0);
    Frame f[kJointCount]{};
    { // 缺配置/反馈/标定都不得输出运动指令；无电机不阻塞。
        auto defaults = kArmConfig;
        defaults.joint_mask = kAllJoints;
        ArmControl a(defaults);
        CHECK(a.enable(kAllJoints, 0) == Result::ConfigUnconfirmed);
        auto n = a.step(0, f);
        CHECK(n == 4);
        CHECK(dji_zero(f, n));
        for (unsigned i = 0; i < n; ++i)
            CHECK(f[i].bus == 1);
        CHECK(find(f, n, 1, 1)->data[7] == 0xfd);
        CHECK(a.feedback_id(Yaw) == 0x205);
        CHECK(find(f, n, 1, 0x1fe) != nullptr); // 电流模式yaw ID1占0x1fe第一槽
        CHECK(find(f, n, 1, 0x2fe) == nullptr);
        CHECK(find(f, n, 1, 0x2ff) == nullptr); // 当前配置不再发送6020电压帧
        CHECK(kArmConfig.joint[Yaw].motor == Motor::Gm6020Current);
        ArmControl b(bench());
        CHECK(b.enable(1, 0) == Result::NotOnline);
        all(b, 0);
        CHECK(b.enable(1, 0) == Result::NotCalibrated);
        CHECK(b.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(b.enable(kAllJoints, 0) == Result::Ok);
        CHECK(b.capture_zero(1, 0) == Result::Busy);
        n = b.step(0, f);
        CHECK(find(f, n, 1, 1)->data[7] == 0xfc); // 等DM使能确认
        all(b, 1, 1);
        n = b.step(1, f);
        CHECK((find(f, n, 1, 1)->data[3] & 0xf) == 0);                           // MIT kp=0
        CHECK(find(f, n, 1, 1)->data[4] == 0 && find(f, n, 1, 1)->data[5] == 0); // kp/kd=0
    }
    { // DJI过零解包和36:1换算；首次反馈不能凭空多一圈。
        ArmControl a(bench());
        prime_roll_feedback(a, 0, 8190);
        dji(a, Roll, 8190, 0);
        CHECK(a.capture_zero(1u << Roll, 0) == Result::Ok);
        dji(a, Roll, 2, 1);
        CHECK(std::fabs(a.status().joint[Roll].position_deg - 4 * 360.0f / 8192 / 36) < 0.0001f);
        dji(a, Roll, 8190, 2);
        CHECK(std::fabs(a.status().joint[Roll].position_deg) < 0.0001f);
        const auto feedback = a.feedback_id(Roll);
        uint8_t bad[8]{};
        CHECK(!a.receive(1, feedback, bad, 7, 3));
        CHECK(!a.receive(2, feedback, bad, 8, 3));
        bad[0] = 0xff;
        CHECK(!a.receive(1, feedback, bad, 8, 3));
        dji(a, Roll, 8190, 10);
        CHECK(!a.status().joint[Roll].calibrated);
    }
    { // Roll zero tolerates RPM spikes and encoder seam jitter after 200 ms.
        ArmControl a(bench());
        dji(a, Roll, 8190, 0, 52);
        dm(a, BigPitch, 0);
        dm(a, SmallPitch, 0);
        dji(a, Gripper, 1000, 0);
        CHECK(a.capture_parked_zero(0) == Result::Moving);
        CHECK(a.status().joint[BigPitch].calibrated);
        CHECK(a.status().joint[SmallPitch].calibrated);
        CHECK(!a.status().joint[Roll].calibrated);
        CHECK(a.status().joint[Gripper].calibrated);
        for (uint32_t t = 1; t < 200; ++t)
            dji(a, Roll, (8190 + (t % 2) * 20) % 8192, t, t % 2 ? 52 : -52);
        CHECK(a.capture_zero(1u << Roll, 199) == Result::Moving);
        dji(a, Roll, 8190, 200, 52);
        CHECK(a.capture_zero(1u << Roll, 200) == Result::Ok);
        CHECK(a.status().joint[Roll].calibrated);
        CHECK(std::fabs(a.status().joint[Roll].position_deg) < .0001f);
        CHECK(a.status().joint[Roll].drive_velocity_deg_s > 8);
    }
    { // Actual slow motion rejects zero even when reported RPM is zero.
        ArmControl a(bench());
        for (uint32_t t = 0; t <= 400; ++t)
            dji(a, Roll, 1000 + t, t);
        CHECK(a.capture_zero(1u << Roll, 400) == Result::Moving);
        for (uint32_t t = 401; t <= 600; ++t)
            dji(a, Roll, 1400, t);
        CHECK(a.capture_zero(1u << Roll, 600) == Result::Ok);
    }
    { // Returning to the starting angle cannot hide a large excursion.
        ArmControl a(bench());
        for (uint32_t t = 0; t <= 200; ++t)
            dji(a, Roll, t == 100 ? 1500 : 1000, t);
        CHECK(a.capture_zero(1u << Roll, 200) == Result::Moving);
        for (uint32_t t = 201; t <= 301; ++t)
            dji(a, Roll, 1000, t);
        CHECK(a.capture_zero(1u << Roll, 301) == Result::Ok);
    }
    { // Stale feedback resets the window through refresh and receive paths.
        ArmControl a(bench());
        for (uint32_t t = 0; t <= 200; ++t)
            dji(a, Roll, 1000, t);
        CHECK(a.capture_zero(1u << Roll, 200) == Result::Ok);
        CHECK(a.capture_zero(1u << Roll, 206) == Result::Moving);
        CHECK(!a.status().joint[Roll].calibrated);
        for (uint32_t t = 207; t <= 406; ++t)
            dji(a, Roll, 1000, t);
        CHECK(a.capture_zero(1u << Roll, 406) == Result::Moving);
        dji(a, Roll, 1000, 407);
        CHECK(a.capture_zero(1u << Roll, 407) == Result::Ok);
        dji(a, Roll, 1000, 414);
        CHECK(a.capture_zero(1u << Roll, 414) == Result::Moving);
    }
    { // Output-angle tolerance respects ratio/direction and tick wraparound.
        auto c = bench();
        c.joint[Roll].feedback_per_joint = 18;
        c.joint[Roll].direction = -1;
        ArmControl a(c);
        const uint32_t start = 0xffffff80u;
        for (uint32_t t = 0; t <= 200; ++t)
            dji(a, Roll, 1000 + (t % 2) * 40, start + t, 52);
        CHECK(a.capture_zero(1u << Roll, start + 200) == Result::Ok);
        dji(a, Roll, 1100, start + 201);
        CHECK(a.capture_zero(1u << Roll, start + 201) == Result::Moving);
    }
    { // 非法目标整包拒绝，不能部分更新；目标超时锁存停机。
        ArmControl a(bench());
        all(a, 0);
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(a.enable(1, 0) == Result::Ok);
        float q[5] = {10, 0, 0, 0, 0};
        CHECK(a.set_targets(1, q, 0) == Result::Ok);
        q[0] = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_targets(1, q, 0) == Result::InvalidArgument);
        CHECK(a.status().joint[Yaw].target_deg == 10);
        q[0] = 91;
        CHECK(a.set_targets(1, q, 0) == Result::OutOfRange);
        for (unsigned t = 0; t <= 251; ++t) {
            all(a, t);
            a.step(t, f);
        }
        CHECK(a.status().faults & CommandTimeout);
        CHECK(a.status().enabled_mask == 0);
        CHECK(dji_zero(f, 4));
        CHECK(a.enable(1, 251) == Result::FaultLatched);
        CHECK(a.clear_faults() == Result::Ok);
        CHECK(a.enable(1, 251) == Result::Ok);
    }
    { // 失联/多圈失准需要重新校零；通讯恢复不自动重启。
        ArmControl a(bench());
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(1, 0);
        a.step(0, f);
        a.step(1, f);
        a.step(2, f);
        a.step(3, f);
        a.step(4, f);
        a.step(5, f);
        a.step(6, f);
        CHECK(a.status().faults & PositionLost);
        CHECK(!a.status().joint[Yaw].calibrated);
        all(a, 7);
        a.step(7, f);
        CHECK(a.status().enabled_mask == 0);
        a.clear_faults();
        CHECK(a.enable(1, 7) == Result::NotCalibrated);
    }
    { // DM温度/驱动器错误、状态回退、使能确认超时都使整臂失能。
        ArmControl a(bench());
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(kAllJoints, 0);
        a.step(0, f);
        all(a, 1, 1);
        dm(a, BigPitch, 1, 1, 80);
        auto n = a.step(1, f);
        CHECK(a.status().faults & OverTemperature);
        CHECK(dji_zero(f, n));
        CHECK(find(f, n, 1, 1)->data[7] == 0xfd);
        ArmControl b(bench());
        all(b, 0);
        b.capture_zero(kAllJoints, 0);
        b.enable(6, 0);
        b.step(0, f);
        dm(b, BigPitch, 1, 1);
        b.step(1, f);
        dm(b, BigPitch, 2, 8);
        b.step(2, f);
        CHECK(b.status().faults & MotorError);
        ArmControl c(bench());
        all(c, 0);
        c.capture_zero(kAllJoints, 0);
        c.enable(6, 0);
        for (unsigned t = 0; t <= 201; ++t) {
            all(c, t);
            c.step(t, f);
        }
        CHECK(c.status().faults & EnableTimeout);
        ArmControl e(bench());
        all(e, 0);
        e.capture_zero(kAllJoints, 0);
        e.enable(6, 0);
        for (unsigned t = 0; t <= 51; ++t)
            e.step(t, f);
        CHECK(e.status().faults & FeedbackTimeout);
    }
    { // 真实控制输出：有符号打包、通道归属、斜坡、输出限幅和正反方向。
        Config cfg = bench();
        cfg.joint[Roll].direction = -1;
        ArmControl a(cfg);
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(1u << Roll, 0);
        float q[5] = {0, 0, 0, 90, 0};
        a.set_targets(1u << Roll, q, 0);
        float previous_v = 0, previous_q = 0;
        for (unsigned t = 0; t < 1000; ++t) {
            all(a, t * 4);
            a.set_targets(1u << Roll, q, t * 4);
            const unsigned n = a.step(t * 4, f);
            auto &s = a.status().joint[Roll];
            CHECK(a.status().faults == 0);
            CHECK(std::fabs(s.effort) <= 3000);
            const float v = (s.reference_deg - previous_q) / 0.004f;
            CHECK(v >= -0.0001f && v <= cfg.joint[Roll].max_speed_deg_s + 0.001f);
            CHECK(s.reference_deg <= cfg.joint[Roll].max_deg);
            // At the position limit, clipping can shorten the final position step.
            // Acceleration is constrained on the planner's reference velocity.
            CHECK(std::fabs(s.drive_reference_velocity_deg_s - previous_v) <=
                  cfg.joint[Roll].max_accel_deg_s2 * 0.004f + 0.0001f);
            previous_q = s.reference_deg;
            previous_v = s.drive_reference_velocity_deg_s;
            const unsigned id = cfg.joint[Roll].motor_id, byte = ((id - 1) % 4) * 2;
            const auto *current = find(f, n, 1, id <= 4 ? 0x200 : 0x1ff);
            CHECK(current != nullptr);
            const int raw = int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]);
            CHECK(raw == int16_t(-s.effort) && raw <= 0);
            for (unsigned j = 0; j < 8; ++j)
                if (j != byte && j != byte + 1) CHECK(current->data[j] == 0);
        }
        a.transport_fault();
        auto n = a.step(4000, f);
        CHECK(dji_zero(f, n));
        CHECK(a.status().faults & TransportError);
    }
    { // GM6020电流模式：真实闭环输出的mA->原码、ID1~7分组、槽位、符号与零输出。
        for (unsigned id = 1; id <= 7; ++id)
            for (int sign = -1; sign <= 1; sign += 2) {
                Config c = bench();
                c.joint[Yaw].motor_id = id;
                c.joint[Yaw].max_effort = 3000;
                c.joint[Yaw].velocity_kp = 1000000;
                ArmControl a(c);
                CHECK(a.status().configuration_valid);
                CHECK(a.feedback_id(Yaw) == 0x204 + id);
                all(a, 0);
                CHECK(a.capture_zero(1, 0) == Result::Ok);
                CHECK(a.enable(1, 0) == Result::Ok);
                float q[5] = {float(sign) * 20, 0, 0, 0, 0};
                CHECK(a.set_targets(1, q, 0) == Result::Ok);
                auto n = a.step(0, f);
                const auto *current = find(f, n, 1, id <= 4 ? 0x1fe : 0x2fe);
                CHECK(current != nullptr);
                CHECK(find(f, n, 1, 0x1ff) == nullptr && find(f, n, 1, 0x2ff) == nullptr);
                const unsigned byte = ((id - 1) % 4) * 2;
                const int raw =
                    int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]);
                CHECK(raw == sign * 16384);
                CHECK(a.status().joint[Yaw].effort == float(sign) * 3000);
                for (unsigned j = 0; j < 8; ++j)
                    if (j != byte && j != byte + 1)
                        CHECK(current->data[j] == 0);
                a.disable();
                all(a, 4);
                n = a.step(4, f);
                CHECK(dji_zero(f, n));
            }
        Config c = bench();
        c.joint[Yaw].velocity_kp = 1000000;
        c.joint[Yaw].direction = -1;
        ArmControl a(c);
        all(a, 0);
        a.capture_zero(1, 0);
        a.enable(1, 0);
        float q[5] = {20, 0, 0, 0, 0};
        a.set_targets(1, q, 0);
        auto n = a.step(0, f);
        const auto *current = find(f, n, 1, 0x1fe);
        CHECK(current != nullptr);
        // 默认上限300mA约对应1638原码；反向只影响报文符号，状态effort仍为关节正向mA。
        CHECK(a.status().joint[Yaw].effort == 300);
        CHECK(int16_t(uint16_t(current->data[0]) << 8 | current->data[1]) == -1638);
        dji(a, Yaw, 1000, 4, 0, 80);
        n = a.step(4, f);
        CHECK(a.status().faults & OverTemperature);
        CHECK(dji_zero(f, n));
        c = bench();
        c.joint[Yaw].max_effort = 3001;
        ArmControl bad_limit(c);
        CHECK(!bad_limit.status().configuration_valid);
        c = bench();
        c.joint[Yaw].motor_id = 8;
        ArmControl bad_id(c);
        CHECK(!bad_id.status().configuration_valid);
        // 显式旧电压类型仍单独走旧帧和原码，不能把3000mA转换误用于电压模式。
        c = bench();
        c.joint[Yaw].motor = Motor::Gm6020Voltage;
        c.joint[Yaw].max_effort = 2000;
        c.joint[Yaw].velocity_kp = 1000000;
        ArmControl voltage(c);
        all(voltage, 0);
        voltage.capture_zero(1, 0);
        voltage.enable(1, 0);
        voltage.set_targets(1, q, 0);
        n = voltage.step(0, f);
        const auto *v = find(f, n, 1, 0x1ff);
        CHECK(v != nullptr && find(f, n, 1, 0x1fe) == nullptr);
        CHECK(int16_t(uint16_t(v->data[0]) << 8 | v->data[1]) == 2000);
    }
    { // 协议边界和配置冲突检查。
        uint8_t d[8];
        pack_dm_torque(99, 12.5f, 30, 10, d);
        CHECK(d[6] == 0x0f && d[7] == 0xff);
        pack_dm_torque(-99, 12.5f, 30, 10, d);
        CHECK(d[6] == 0 && d[7] == 0);
        Config c = bench();
        c.joint[Gripper].motor_id = c.joint[Roll].motor_id;
        ArmControl a(c);
        CHECK(!a.status().configuration_valid);
        CHECK(a.step(0, f) == 0);
        c = bench();
        c.joint[BigPitch].feedback_id = 1;
        ArmControl b(c);
        CHECK(!b.status().configuration_valid);
        c = bench();
        c.joint[Yaw].feedback_per_joint = 0;
        ArmControl e(c);
        CHECK(!e.status().configuration_valid);
        ArmControl g(bench());
        all(g, 0);
        g.capture_zero(6, 0);
        g.enable(6, 0);
        g.step(0, f);
        all(g, 8);
        g.step(8, f);
        CHECK(g.status().faults & TimingError);
    }
    { // 动态夹爪电流上限立即生效，且不能提高到超过配置值。
        Config c = bench();
        c.joint[Gripper].velocity_kp = 100000;
        ArmControl a(c);
        all(a, 0);
        a.capture_zero(1u << Gripper, 0);
        a.enable(1u << Gripper, 0);
        float q[5] = {0, 0, 0, 0, 45};
        a.set_targets(1u << Gripper, q, 0);
        CHECK(a.set_effort_limit(Gripper, 150) == Result::Ok);
        auto n = a.step(0, f);
        CHECK(a.status().joint[Gripper].effort == 150);
        const unsigned id = c.joint[Gripper].motor_id, byte = ((id - 1) % 4) * 2;
        const auto *current = find(f, n, 1, id <= 4 ? 0x200 : 0x1ff);
        CHECK(current != nullptr);
        CHECK(int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]) == 150 * c.joint[Gripper].direction);
        for (unsigned j = 0; j < 8; ++j)
            if (j != byte && j != byte + 1) CHECK(current->data[j] == 0);
        CHECK(a.set_effort_limit(Gripper, 0) == Result::Ok);
        all(a, 4);
        a.step(4, f);
        CHECK(a.status().joint[Gripper].effort == 0);
        CHECK(a.set_effort_limit(Gripper, 501) == Result::InvalidArgument);
        CHECK(a.set_effort_limit(Gripper, -1) == Result::InvalidArgument);
        CHECK(a.set_effort_limit(5, 100) == Result::InvalidArgument);
    }
    { // 在线PID只在失能/反馈新鲜/已校零/静止时原子生效，不改变目标和零位。
        ArmControl a(bench());
        const JointTuning tuning{3, 40, 2, 500};
        JointTuning current{};
        CHECK(a.get_tuning(Roll, current));
        CHECK(!a.get_tuning(kJointCount, current));
        CHECK(a.set_tuning(Roll, tuning, 0) == Result::NotOnline);
        all(a, 0);
        CHECK(a.set_tuning(Roll, tuning, 0) == Result::NotCalibrated);
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        float q[5] = {0, 1, 2, 3, 4};
        CHECK(a.set_targets(kAllJoints, q, 0) == Result::Ok);
        dji(a, Roll, 1004, 1);
        const auto before = a.status();
        const auto limits = a.yaw_limits();
        CHECK(a.set_tuning(Roll, tuning, 1) == Result::Ok);
        CHECK(a.get_tuning(Roll, current));
        CHECK(current.position_kp == 3 && current.velocity_kp == 40 &&
              current.velocity_ki == 2 && current.effort_limit == 500);
        CHECK(a.status().enabled_mask == 0 && a.status().joint[Roll].calibrated);
        CHECK(a.status().joint[Roll].position_deg == before.joint[Roll].position_deg &&
              a.status().joint[Roll].target_deg == before.joint[Roll].target_deg &&
              a.status().joint[Roll].reference_deg == before.joint[Roll].reference_deg);
        CHECK(a.yaw_limits().min_deg == limits.min_deg &&
              a.yaw_limits().max_deg == limits.max_deg &&
              a.yaw_limits().version == limits.version);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const JointTuning invalid[] = {
            {nan, 40, 2, 500}, {3, nan, 2, 500}, {3, 40, nan, 500}, {3, 40, 2, nan},
            {0, 40, 2, 500}, {1001, 40, 2, 500}, {3, -1, 2, 500}, {3, 40, -1, 500},
            {3, 1000001, 2, 500}, {3, 40, 1000001, 500}, {3, 40, 2, -1}, {3, 40, 2, 3001}};
        for (const auto &bad : invalid) {
            CHECK(a.set_tuning(Roll, bad, 1) == Result::InvalidArgument);
            CHECK(a.get_tuning(Roll, current));
            CHECK(current.position_kp == 3 && current.velocity_kp == 40 &&
                  current.velocity_ki == 2 && current.effort_limit == 500);
        }
        CHECK(a.set_tuning(kJointCount, tuning, 1) == Result::InvalidArgument);
        dji(a, Roll, 1004, 2, 100);
        CHECK(a.set_tuning(Roll, tuning, 2) == Result::Moving);
        dji(a, Roll, 1004, 3);
        CHECK(a.set_tuning(Roll, tuning, 60) == Result::NotOnline);

        ArmControl busy(bench());
        all(busy, 0);
        CHECK(busy.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(busy.enable(1u << Yaw, 0) == Result::Ok);
        CHECK(busy.set_tuning(Roll, tuning, 0) == Result::Busy); // 任一轴使能都拒绝。
        busy.disable();
        busy.transport_fault();
        CHECK(busy.set_tuning(Roll, tuning, 0) == Result::FaultLatched);
        auto four = bench();
        four.joint_mask = kFourAxisJoints;
        ArmControl no_yaw(four);
        CHECK(!no_yaw.get_tuning(Yaw, current));
        CHECK(no_yaw.set_tuning(Yaw, {2, 20, 5, 100}, 0) == Result::InvalidArgument);
    }
    { // 两pitch解耦后，调一台只检查该轴自身的在线/校零/静止条件。
        ArmControl a(bench());
        JointTuning small_before{};
        CHECK(a.get_tuning(SmallPitch, small_before));
        const JointTuning tuning{3, .04f, .002f, .2f};
        dm(a, BigPitch, 0);
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::NotCalibrated);
        CHECK(a.capture_zero(1u << BigPitch, 0) == Result::Ok);
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::Ok);
        JointTuning small{};
        CHECK(a.get_tuning(SmallPitch, small));
        CHECK(small.position_kp == small_before.position_kp &&
              small.velocity_kp == small_before.velocity_kp &&
              small.velocity_ki == small_before.velocity_ki &&
              small.effort_limit == small_before.effort_limit);
        CHECK(a.status().enabled_mask == 0 && a.status().joint[BigPitch].calibrated &&
              !a.status().joint[SmallPitch].calibrated);
        dm_at(a, BigPitch, 0, 4, 0x7d0); // 大P自身在动，仍要求静止。
        CHECK(a.set_tuning(BigPitch, tuning, 4) == Result::Moving);
        dm(a, BigPitch, 8);
        CHECK(a.set_tuning(BigPitch, tuning, 8) == Result::Ok);
    }
    { // A lower startup limit is distinct from the permitted BigPitch tuning limit.
        auto config = bench();
        config.joint[BigPitch].max_effort = 3.0f;
        config.joint[BigPitch].initial_effort_limit = .3f;
        config.joint[BigPitch].velocity_kp = .03f;
        config.joint[SmallPitch].max_effort = .3f;
        config.joint[SmallPitch].initial_effort_limit = .3f;
        ArmControl a(config);
        JointTuning big{}, small{};
        CHECK(a.get_tuning(BigPitch, big) && big.effort_limit == .3f);
        CHECK(a.get_tuning(SmallPitch, small) && small.effort_limit == .3f);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        const float q[kJointCount] = {0, 10, 0, 0, 0};
        auto step_with_velocity_error = [&](uint32_t now) {
            // MIT velocity raw 0x7d0 reports about -40 deg/s at VMAX=30 rad/s.
            const uint8_t dm_fast[8] = {0x11, 0x80, 0, 0x7d, 0x08, 0, 25, 25};
            a.receive(1, a.feedback_id(BigPitch), dm_fast, 8, now);
            dm(a, SmallPitch, now, 1);
            return a.step(now, f);
        };
        CHECK(a.enable(kPitchPair, 0) == Result::Ok);
        CHECK(a.set_targets(kPitchPair, q, 0) == Result::Ok);
        step_with_velocity_error(0);
        CHECK(a.status().joint[BigPitch].effort == .3f);

        a.disable();
        dm(a, BigPitch, 4, 1);
        dm(a, SmallPitch, 4, 1);
        big.effort_limit = 3;
        CHECK(a.set_tuning(BigPitch, big, 4) == Result::Ok);
        big.effort_limit = 3.01f;
        CHECK(a.set_tuning(BigPitch, big, 4) == Result::InvalidArgument);
        small.effort_limit = 3;
        CHECK(a.set_tuning(SmallPitch, small, 4) == Result::InvalidArgument);
        CHECK(a.get_tuning(BigPitch, big) && big.effort_limit == 3);
        CHECK(a.get_tuning(SmallPitch, small) && small.effort_limit == .3f);
        CHECK(!a.status().enabled_mask);
        CHECK(a.enable(kPitchPair, 4) == Result::Ok);
        CHECK(a.set_targets(kPitchPair, q, 4) == Result::Ok);
        const auto n = step_with_velocity_error(4);
        const float effort = a.status().joint[BigPitch].effort;
        CHECK(!a.status().faults && effort > .3f && effort <= 3);
        const auto *drive = find(f, n, 1, 1);
        CHECK(drive != nullptr);
        const unsigned wire_torque = (unsigned(drive->data[6] & 0xf) << 8) | drive->data[7];
        CHECK(std::fabs((wire_torque * 20.0f / 4095.0f - 10.0f) - effort) < .005f);
    }
    { // 在线增益真正进入原闭环；运行限流不会被夹爪owner的保持请求覆盖。
        auto baseline_config = bench();
        baseline_config.joint[Roll].position_kp = 2;
        baseline_config.joint[Roll].velocity_kp = 20;
        baseline_config.joint[Roll].velocity_ki = 5;
        ArmControl baseline(baseline_config), tuned(baseline_config);
        all(baseline, 0);
        all(tuned, 0);
        CHECK(baseline.capture_zero(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.capture_zero(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.set_tuning(Roll, {2, 40, 5, 500}, 0) == Result::Ok);
        CHECK(baseline.enable(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.enable(1u << Roll, 0) == Result::Ok);
        float q[5] = {0, 0, 0, 1, 45};
        CHECK(baseline.set_targets(1u << Roll, q, 0) == Result::Ok);
        CHECK(tuned.set_targets(1u << Roll, q, 0) == Result::Ok);
        baseline.step(0, f);
        tuned.step(0, f);
        CHECK(tuned.status().joint[Roll].effort > baseline.status().joint[Roll].effort * 1.9f);

        auto c = bench();
        c.joint[Gripper].velocity_kp = 100000;
        ArmControl gripper(c);
        all(gripper, 0);
        CHECK(gripper.capture_zero(1u << Gripper, 0) == Result::Ok);
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 40}, 0) == Result::Ok);
        CHECK(gripper.set_effort_limit(Gripper, 150) == Result::Ok);
        CHECK(gripper.enable(1u << Gripper, 0) == Result::Ok);
        CHECK(gripper.set_targets(1u << Gripper, q, 0) == Result::Ok);
        gripper.step(0, f);
        CHECK(gripper.status().joint[Gripper].effort == 40);
        gripper.disable();
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 500}, 0) == Result::Ok);
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 501}, 0) == Result::InvalidArgument);
        ArmControl restarted(c);
        JointTuning original{};
        CHECK(restarted.get_tuning(Gripper, original) && original.effort_limit == 500);
    }
    { // Production defaults carry the provisional measured pitch feedforward.
        auto c = kArmConfig;
        CHECK(c.joint[Roll].position_kp == 10 && c.joint[Roll].velocity_kp == 45 &&
              c.joint[Roll].velocity_ki == 8);
        c.hardware_confirmed = true;
        ArmControl a(c);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        for (unsigned joint = BigPitch; joint <= SmallPitch; ++joint) {
            JointTuning tuning{};
            CHECK(a.get_tuning(joint, tuning));
            const float expected_bias = joint == BigPitch ? -0.228f : 1.276f;
            const float expected_gain = joint == BigPitch ? 0.437f : 0.248f;
            const float expected_offset = joint == BigPitch ? 90.0f : -48.33f;
            CHECK(std::fabs(tuning.torque_bias_nm - expected_bias) < .0001f);
            CHECK(std::fabs(tuning.gravity_gain_nm - expected_gain) < .0001f);
            CHECK(std::fabs(tuning.gravity_offset_deg - expected_offset) < .0001f);
            CHECK(tuning.effort_limit == 6.5f && c.joint[joint].max_effort == 6.5f);
            CHECK(tuning.up_ff_nm == (joint == BigPitch ? .85f : 0));
            CHECK(tuning.down_ff_nm == (joint == BigPitch ? .20f : 0));
            CHECK(tuning.velocity_kp == (joint == BigPitch ? .012f : .01f));
            CHECK(c.joint[joint].up_direction == (joint == BigPitch ? -1 : 1));
            tuning.velocity_kp = tuning.velocity_ki = 0;
            tuning.up_ff_nm = tuning.down_ff_nm = 0;
            CHECK(a.set_tuning(joint, tuning, 0) == Result::Ok);
        }
        CHECK(a.enable(kPitchPair, 0) == Result::Ok);
        dm_at(a, BigPitch, -1.0f, 4);
        dm_at(a, SmallPitch, 59.151257f, 4);
        a.step(4, f);
        const auto &b = a.status().joint[BigPitch];
        const auto &s = a.status().joint[SmallPitch];
        CHECK(std::fabs(b.feedforward_effort -
                        (-0.228f + 0.437f * std::cos((-1.0f + 90.0f) / 57.2957795131f))) <
              .001f);
        CHECK(std::fabs(s.feedforward_effort -
                        (1.276f + 0.248f * std::cos((59.151257f - 48.33f) / 57.2957795131f))) <
              .001f);
        CHECK(b.effort == b.feedforward_effort && s.effort == s.feedforward_effort);
    }
    { // Each feedforward uses its own absolute calibrated drive angle.
        auto c = bench();
        c.joint[BigPitch].direction = -1;
        ArmControl a(c);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(a.set_tuning(BigPitch, {2, 0, 0, 1, .2f, .8f, 15}, 0) == Result::Ok);
        CHECK(a.set_tuning(SmallPitch, {2, 0, 0, 1, -.1f, .6f, -20}, 0) == Result::Ok);
        CHECK(a.enable(kPitchPair, 0) == Result::Ok);
        dm_at(a, BigPitch, -60, 4);
        dm_at(a, SmallPitch, 30, 4);
        const unsigned n = a.step(4, f);
        const auto &b = a.status().joint[BigPitch];
        const auto &s = a.status().joint[SmallPitch];
        CHECK(std::fabs(b.drive_position_deg - 60) < .02f);
        CHECK(std::fabs(s.position_deg - 30) < .02f);
        CHECK(std::fabs(b.feedforward_effort - (.2f + .8f * std::cos(75 / 57.2957795131f))) < .001f);
        CHECK(std::fabs(s.feedforward_effort - (-.1f + .6f * std::cos(10 / 57.2957795131f))) < .001f);
        CHECK(b.effort == b.feedforward_effort && s.effort == s.feedforward_effort);
        const auto *wire = find(f, n, 1, 1);
        CHECK(wire != nullptr);
        const unsigned torque = (unsigned(wire->data[6] & 15) << 8) | wire->data[7];
        CHECK(std::fabs(torque * 20.0f / 4095 - 10 + b.effort) < .005f);
        a.disable();
        CHECK(a.status().joint[BigPitch].effort == 0 &&
              a.status().joint[SmallPitch].feedforward_effort == 0);
    }
    { // Ki=0 removes stored integral; an enabled pitch can be retuned while moving.
        auto c = bench();
        c.joint[BigPitch].velocity_kp = 0;
        c.joint[BigPitch].velocity_ki = 1;
        ArmControl a(c);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(a.enable_parked_tuning(0) == Result::Ok);
        a.step(0, f);
        CHECK(std::fabs(a.status().joint[BigPitch].effort) > .0001f);
        JointTuning b{}, small{};
        CHECK(a.get_tuning(BigPitch, b) && a.get_tuning(SmallPitch, small));
        b.velocity_ki = 0;
        CHECK(a.set_tuning_runtime(BigPitch, b, 0) == Result::Ok);
        dm(a, BigPitch, 4, 1);
        dm(a, SmallPitch, 4, 1);
        a.step(4, f);
        CHECK(a.status().joint[BigPitch].effort == 0);
        dm_at(a, BigPitch, 10, 8, 0x7d0);
        dm_at(a, SmallPitch, 15, 8);
        b.torque_bias_nm = .25f;
        b.effort_limit = .5f;
        CHECK(a.set_tuning_runtime(BigPitch, b, 8) == Result::Ok);
        a.step(8, f);
        CHECK(a.status().joint[BigPitch].effort == .25f);
        b.velocity_kp = .03f;
        CHECK(a.set_tuning_runtime(BigPitch, b, 8) == Result::Ok);
        CHECK(a.get_tuning(BigPitch, b) && b.torque_bias_nm == .25f && b.velocity_kp == .03f);
        b.velocity_kp = 0;
        b.torque_bias_nm = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_tuning_runtime(BigPitch, b, 8) == Result::InvalidArgument);
        JointTuning after_moving{};
        CHECK(a.get_tuning(BigPitch, after_moving) && after_moving.torque_bias_nm == .25f &&
              after_moving.velocity_kp == .03f);
        JointTuning after{};
        CHECK(a.get_tuning(SmallPitch, after) && after.velocity_kp == small.velocity_kp &&
              after.torque_bias_nm == small.torque_bias_nm);
        b.torque_bias_nm = 2;
        CHECK(a.set_tuning_runtime(BigPitch, b, 8) == Result::Ok);
        dm_at(a, BigPitch, 10, 12, 0x7d0);
        dm_at(a, SmallPitch, 15, 12);
        a.step(12, f);
        CHECK(a.status().joint[BigPitch].feedforward_effort == 2 &&
              a.status().joint[BigPitch].effort == .5f);
        dm_at(a, BigPitch, 151, 16);
        dm_at(a, SmallPitch, 151, 16);
        a.step(16, f);
        CHECK(a.status().joint[BigPitch].effort == 0); // Cannot push further beyond soft limit.
        dm(a, BigPitch, 20, 8);
        dm(a, SmallPitch, 20, 1);
        a.step(20, f);
        CHECK(a.status().joint[BigPitch].effort == 0 &&
              a.status().joint[BigPitch].feedforward_effort == 0 &&
              a.status().joint[SmallPitch].enabled);
    }
    { // Feedforward and PI share one cap; saturation must not accumulate velocity integral.
        auto c = bench();
        c.joint[BigPitch].velocity_kp = 0;
        c.joint[BigPitch].velocity_ki = 1;
        c.joint[BigPitch].torque_bias_nm = 2;
        c.joint[BigPitch].initial_effort_limit = 1;
        ArmControl a(c);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(a.enable_parked_tuning(0) == Result::Ok);
        for (uint32_t t = 0; t <= 16; t += 4) {
            dm_at(a, BigPitch, 0, t, 0x7d0);
            dm_at(a, SmallPitch, 0, t);
            a.step(t, f);
            CHECK(a.status().joint[BigPitch].effort == 1);
        }
        dm(a, BigPitch, 20, 1);
        dm(a, SmallPitch, 20, 1);
        JointTuning t{};
        CHECK(a.get_tuning(BigPitch, t));
        t.torque_bias_nm = 0;
        CHECK(a.set_tuning_runtime(BigPitch, t, 20) == Result::Ok);
        a.step(20, f);
        CHECK(std::fabs(a.status().joint[BigPitch].effort) < .01f);
    }
    { // Production pitch limits reject outward targets and permit recovery from each boundary.
        auto c = kArmConfig;
        c.hardware_confirmed = true;
        for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
            CHECK(c.joint[j].min_deg == (j == BigPitch ? -2 : -70));
            CHECK(c.joint[j].max_deg == (j == BigPitch ? 150 : 70));
            c.joint[j].velocity_kp = c.joint[j].velocity_ki = 0;
            c.joint[j].torque_bias_nm = -.5f;
            c.joint[j].gravity_gain_nm = 0;
            c.joint[j].up_ff_nm = c.joint[j].down_ff_nm = 0;
        }
        ArmControl outward(c);
        dm(outward, BigPitch, 0);
        dm(outward, SmallPitch, 0);
        CHECK(outward.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(outward.enable(kPitchPair, 0) == Result::Ok);
        float q[kJointCount] = {0, -2, -70, 0, 0};
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::Ok);
        q[BigPitch] = -2.01f;
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::OutOfRange);
        q[BigPitch] = -2;
        q[SmallPitch] = -70.01f;
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::OutOfRange);
        q[SmallPitch] = 70;
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::Ok);
        q[SmallPitch] = 70.01f;
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::OutOfRange);
        q[SmallPitch] = -70;
        CHECK(outward.set_targets(kPitchPair, q, 0) == Result::Ok);
        dm_at(outward, BigPitch, -2.02f, 4);
        dm_at(outward, SmallPitch, -70.02f, 4);
        outward.step(4, f);
        CHECK(outward.status().faults == 0);
        for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
            CHECK(outward.status().joint[j].target_deg == c.joint[j].min_deg);
            CHECK(outward.status().joint[j].reference_deg >= c.joint[j].min_deg);
            CHECK(outward.status().joint[j].effort == 0);
            c.joint[j].torque_bias_nm = .5f;
        }
        ArmControl inward(c);
        dm(inward, BigPitch, 0);
        dm(inward, SmallPitch, 0);
        CHECK(inward.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(inward.enable(kPitchPair, 0) == Result::Ok);
        dm_at(inward, BigPitch, -2.02f, 4);
        dm_at(inward, SmallPitch, -70.02f, 4);
        inward.step(4, f);
        CHECK(inward.status().faults == 0);
        CHECK(inward.status().joint[BigPitch].effort == .5f);
        CHECK(inward.status().joint[SmallPitch].effort == .5f);
        dm_at(outward, BigPitch, -4.1f, 8);
        dm_at(outward, SmallPitch, -72.1f, 8);
        outward.step(8, f);
        CHECK(outward.status().faults & PositionLimit);
        CHECK(outward.status().enabled_mask == 0);
    }
    { // Motion assist follows desired direction, slews, and stays within the total effort cap.
        for (int up_direction = -1; up_direction <= 1; up_direction += 2) {
            auto c = bench();
            c.joint[BigPitch].up_direction = up_direction;
            c.joint[SmallPitch].up_direction = -up_direction;
            ArmControl a(c);
            dm(a, BigPitch, 0);
            dm(a, SmallPitch, 0);
            CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
            for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
                JointTuning tuning{};
                CHECK(a.get_tuning(j, tuning));
                tuning.velocity_kp = tuning.velocity_ki = 0;
                tuning.up_ff_nm = .12f;
                tuning.down_ff_nm = .08f;
                CHECK(a.set_tuning(j, tuning, 0) == Result::Ok);
            }
            CHECK(a.enable(kPitchPair, 0) == Result::Ok);
            float q[kJointCount] = {0, float(up_direction * 10), float(-up_direction * 10), 0, 0};
            float previous[kJointCount]{};
            for (uint32_t now = 4; now <= 120; now += 4) {
                dm(a, BigPitch, now, 1);
                dm(a, SmallPitch, now, 1);
                CHECK(a.set_targets(kPitchPair, q, now) == Result::Ok);
                a.step(now, f);
                for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
                    const auto &s = a.status().joint[j];
                    CHECK(std::fabs(s.motion_feedforward_effort - previous[j]) <= .00801f);
                    CHECK(s.effort == s.motion_feedforward_effort);
                    previous[j] = s.motion_feedforward_effort;
                }
            }
            CHECK(a.status().joint[BigPitch].motion_ff_direction == 1);
            CHECK(std::fabs(a.status().joint[BigPitch].effort - up_direction * .12f) < .0001f);
            CHECK(std::fabs(a.status().joint[SmallPitch].effort + up_direction * .12f) < .0001f);
            q[BigPitch] *= -1;
            q[SmallPitch] *= -1;
            for (uint32_t now = 124; now <= 400; now += 4) {
                dm(a, BigPitch, now, 1);
                dm(a, SmallPitch, now, 1);
                a.set_targets(kPitchPair, q, now);
                const auto count = a.step(now, f);
                for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
                    const auto &s = a.status().joint[j];
                    CHECK(std::fabs(s.motion_feedforward_effort - previous[j]) <= .00801f);
                    previous[j] = s.motion_feedforward_effort;
                    const auto *wire = find(f, count, 1, c.joint[j].motor_id);
                    CHECK(wire != nullptr);
                    const unsigned torque = (unsigned(wire->data[6] & 15) << 8) | wire->data[7];
                    CHECK(std::fabs(torque * 20.0f / 4095 - 10 - s.effort) < .005f);
                }
            }
            CHECK(a.status().joint[BigPitch].motion_ff_direction == -1);
            CHECK(std::fabs(a.status().joint[BigPitch].effort + up_direction * .08f) < .0001f);
            CHECK(std::fabs(a.status().joint[SmallPitch].effort - up_direction * .08f) < .0001f);
            CHECK(a.set_effort_limit(BigPitch, .02f) == Result::Ok);
            dm(a, BigPitch, 404, 1);
            dm(a, SmallPitch, 404, 1);
            a.step(404, f);
            CHECK(std::fabs(a.status().joint[BigPitch].effort) <= .020001f);
            CHECK(a.disable_joints(1u << BigPitch) == Result::Ok);
            CHECK(a.status().joint[BigPitch].motion_feedforward_effort == 0);
            CHECK(a.status().joint[BigPitch].motion_ff_direction == 0);
            CHECK(a.status().joint[SmallPitch].enabled);
            a.disable();
            CHECK(a.status().joint[SmallPitch].motion_feedforward_effort == 0);
        }
    }
    { // Hysteresis uses desired speed, tolerates encoder zero quantization, and ramps out.
        auto c = bench();
        c.joint[BigPitch].up_direction = 1;
        c.joint[BigPitch].velocity_kp = c.joint[BigPitch].velocity_ki = 0;
        c.joint[BigPitch].up_ff_nm = .1f;
        ArmControl a(c);
        dm(a, BigPitch, 0);
        CHECK(a.capture_zero(1u << BigPitch, 0) == Result::Ok);
        CHECK(a.enable(1u << BigPitch, 0) == Result::Ok);
        dm_at(a, BigPitch, -.2f, 4);
        a.step(4, f);
        CHECK(a.status().joint[BigPitch].motion_ff_direction == 0);
        dm_at(a, BigPitch, -.3f, 8);
        a.step(8, f);
        CHECK(a.status().joint[BigPitch].motion_ff_direction == 1);
        dm_at(a, BigPitch, -.2f, 12);
        a.step(12, f);
        CHECK(a.status().joint[BigPitch].motion_ff_direction == 1);
        dm_at(a, BigPitch, -.1f, 16);
        a.step(16, f);
        CHECK(a.status().joint[BigPitch].motion_ff_direction == 0);
        for (uint32_t now = 20; now <= 32; now += 4) {
            dm(a, BigPitch, now, 1);
            a.step(now, f);
        }
        CHECK(a.status().joint[BigPitch].motion_feedforward_effort == 0);
        CHECK(a.status().joint[BigPitch].effort == 0);
    }
    { // Non-pitch axes cannot accidentally acquire a torque feedforward.
        ArmControl a(bench());
        JointTuning t{};
        CHECK(a.get_tuning(Roll, t));
        t.torque_bias_nm = .2f;
        CHECK(a.set_tuning(Roll, t, 0) == Result::InvalidArgument);
        t.torque_bias_nm = 0;
        t.up_ff_nm = .1f;
        CHECK(a.set_tuning(Roll, t, 0) == Result::InvalidArgument);
        CHECK(a.get_tuning(BigPitch, t));
        t.up_ff_nm = -.1f;
        CHECK(a.set_tuning(BigPitch, t, 0) == Result::InvalidArgument);
        t.up_ff_nm = 6.51f;
        CHECK(a.set_tuning(BigPitch, t, 0) == Result::InvalidArgument);
        t.up_ff_nm = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_tuning(BigPitch, t, 0) == Result::InvalidArgument);
    }
    { // HAL tick绕回不能触发误失联；原码8192不是有效编码器值。
        ArmControl a(bench());
        const uint32_t t = 0xfffffffeu;
        all(a, t);
        a.capture_zero(kAllJoints, t);
        a.enable(1, t);
        a.step(t, f);
        all(a, 0xffffffffu);
        a.step(0xffffffffu, f);
        all(a, 0);
        a.step(0, f);
        CHECK(a.status().faults == 0);
    }
}
} // namespace
extern "C" void test_entry() {
    run();
    test_done = 1;
    __asm volatile("bkpt #0");
    for (;;) {}
}
