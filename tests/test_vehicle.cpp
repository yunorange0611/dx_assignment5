#include "Vehicle/vehicle_state.h"
#include "Power/chassis_power.h"
#include "Imu/attitude_estimator.h"
#include "Arm/arm_control.h"
#include "Chassis/chassis_config.h"
#include "vehicle_state_task.h"
#include "task_ins.h"
#include <cmath>
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
namespace {
bool near(float x, float y, float error = .001f) {
    return std::fabs(x - y) < error;
}
PowerMeterStatus meter(uint32_t now, float energy = 400, float power = 10) {
    PowerMeterStatus m{};
    m.frames = 1;
    m.received_ms = now;
    m.energy_j = energy;
    m.power_w = power;
    return m;
}
AttitudeSample sample(uint32_t ms, uint32_t frame, float ax = 0, float ay = 0,
                      float az = 9.80665f) {
    AttitudeSample s{};
    s.received_ms = ms;
    s.frame = frame;
    s.temperature_c = 25;
    s.accel_m_s2[0] = ax;
    s.accel_m_s2[1] = ay;
    s.accel_m_s2[2] = az;
    return s;
}
AttitudeConfig attitude_config() {
    AttitudeConfig c{};
    c.installation_confirmed = c.thresholds_confirmed = true;
    c.calibration_samples = 10;
    c.calibration_min_ms = 36;
    c.correction_tau_s = .02f;
    return c;
}
void sources(uint32_t now, VehicleChassisInput &ch, VehicleArmInput &arm, VehicleImuInput &imu) {
    ch = {};
    arm = {};
    imu = {};
    ch.ready = ch.configuration_valid = ch.wheels_stationary = ch.power_session_allowed = true;
    ch.hardware_confirmed = ch.power_configuration_valid = ch.power_confirmed = true;
    ch.updated_ms = now;
    for (unsigned i = 0; i < 2; ++i) {
        ch.feedback_received[i] = true;
        ch.feedback_ms[i] = now;
    }
    arm.ready = arm.configuration_valid = true;
    arm.updated_ms = now;
    arm.all_feedback_received = arm.all_online = arm.all_calibrated = arm.all_positions_finite =
        true;
    imu.ready = imu.configuration_valid = imu.valid = imu.calibrated = imu.confirmed = true;
    imu.updated_ms = imu.received_ms = now;
}
VehicleState evaluate(uint32_t now, const VehicleConfig &cfg, const VehicleChassisInput &ch,
                      const VehicleArmInput &arm, const VehicleImuInput &imu, float yaw_half) {
    const VehicleTiming timing{kChassisConfig.feedback_timeout_ms, kArmConfig.feedback_timeout_ms,
                               kAttitudeConfig.max_sample_gap_ms};
    return evaluate_vehicle(now, cfg, VehicleInput{ch, arm, imu, yaw_half}, timing);
}
} // namespace
int test_vehicle() {
    { // No first meter frame and unconfirmed parameters cannot permit a session.
        auto unconfirmed = kChassisPowerConfig;
        unconfirmed.confirmed = false;
        ChassisPower defaults(unconfirmed);
        CHECK(!defaults.step(0, meter(0), false, true).session_allowed);
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.require_meter = true; // This block verifies the strict legacy policy.
        ChassisPower p(cfg);
        PowerMeterStatus empty{};
        CHECK(!p.step(0, empty, false, true).session_allowed &&
              p.status().reason == PowerBlock::NoMeter);
        CHECK(p.step(4, meter(4), false, true).session_allowed &&
              p.status().phase == PowerPhase::Ready);
        CHECK(p.step(8, meter(8), true, false).phase == PowerPhase::Driving);
        const float candidate[2] = {2000, -2000}, rpm[2] = {100, 100};
        float out[2];
        p.limit(candidate, rpm, out);
        CHECK(out[0] <= cfg.drive_current_ma && out[1] >= -cfg.braking_current_ma);
        CHECK(out[0] > 0 && out[1] < 0);
        p.step(12, meter(12, 400, 80), true, false);
        p.limit(candidate, rpm, out);
        CHECK(near(out[0], 2000) && near(out[1], -2000)); // Power does not clamp drive current.
        p.step(16, meter(16, 150), true, false);
        CHECK(near(p.status().speed_scale, 70.f / 220.f)); // Energy scale only; power filter is slow.
        const float nan[2] = {100, std::numeric_limits<float>::quiet_NaN()};
        p.limit(nan, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0);
    }
    { // Power governor lowers the speed target, not the drive-current ceiling.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.power_filter_ms = 4;
        ChassisPower p(cfg);
        p.step(0, meter(0, 400, 10), false, true);
        p.step(4, meter(4, 400, 10), true, false);
        for (uint32_t t = 8; t <= 40; t += 4)
            p.step(t, meter(t, 400, 80), true, false);
        CHECK(p.status().filtered_power_w > 75);
        CHECK(p.status().speed_scale > .5f && p.status().speed_scale < .6f);
        CHECK(!p.status().stop_requested && p.status().phase == PowerPhase::Driving);
        CHECK(p.status().power_over_50_ms > 0 && p.status().power_over_latched);
        const float candidate[2] = {5000, -5000}, rpm[2] = {100, 100};
        float out[2];
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 5000 && out[1] == -5000);
    }
    { // Diagnostic only: raw over-50 time, latch and minimum energy; no power cut.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = false; // Diagnostics must still observe the meter when unconfirmed.
        ChassisPower p(cfg);
        p.step(0, meter(0, 400, 60), false, true);
        CHECK(p.status().power_over_latched && p.status().power_over_50_ms == 0);
        CHECK(near(p.status().min_energy_j, 400));
        p.step(4, meter(4, 390, 60), false, true);
        CHECK(p.status().power_over_50_ms == 4);
        p.step(8, meter(8, 380, 49.9f), false, true);
        CHECK(p.status().power_over_50_ms == 4 && p.status().power_over_latched == 1);
        CHECK(near(p.status().min_energy_j, 380));
        p.step(300, meter(8, 300, 60), false, true); // Stale frame: no time and no energy count.
        CHECK(p.status().power_over_50_ms == 4 && near(p.status().min_energy_j, 380));
        p.step(304, meter(304, 370, 60), false, true); // First fresh frame after a gap: no elapsed.
        CHECK(p.status().power_over_50_ms == 4);
        p.step(308, meter(308, 360, 60), false, true);
        CHECK(p.status().power_over_50_ms == 8);
        CHECK(near(p.status().min_energy_j, 360));
        p.reset_diagnostics();
        CHECK(p.status().power_over_50_ms == 0 && !p.status().power_over_latched &&
              p.status().min_energy_j == 0);
    }
    { // Runtime power tuning is validated, but confirmed=false keeps it inactive.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = false;
        ChassisPower p(cfg);
        p.set_power_tuning(42, 200);
        CHECK(near(p.config().budget_w, 42) && near(p.config().power_filter_ms, 200));
        p.set_power_tuning(50, 0);
        CHECK(near(p.config().budget_w, 42) && near(p.config().power_filter_ms, 200));
    }
    { // Low energy starts bounded controlled braking, and recovery never cancels the stop.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        ChassisPower p(cfg);
        p.step(0, meter(0), false, true);
        p.step(4, meter(4), true, false);
        for (uint32_t t = 8; t < 108; t += 4)
            CHECK(p.step(t, meter(t, 70), true, false).phase == PowerPhase::Driving);
        CHECK(p.step(108, meter(108, 70), true, false).phase == PowerPhase::StopLowEnergy);
        CHECK(p.status().stop_requested && p.status().session_allowed);
        const float candidate[2] = {1000, -1000}, rpm[2] = {100, 100};
        float out[2];
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 0 && out[1] == -1000);
        const float reverse[2] = {-100, -100};
        p.limit(candidate, reverse, out);
        CHECK(out[0] == 1000 && out[1] == 0);
        CHECK(p.step(112, meter(112), true, false).phase == PowerPhase::StopLowEnergy);
        CHECK(p.step(116, meter(116), true, true).phase == PowerPhase::Inhibited);
        CHECK(p.status().inhibited && !p.status().session_allowed);
        CHECK(!p.clear(120, meter(120), false));
        CHECK(!p.clear(120, meter(120, 200), true));
        CHECK(!p.clear(321, meter(0), true));
        CHECK(p.clear(324, meter(324), true));
        CHECK(p.status().power_over_50_ms == 0 && p.status().power_over_latched == 0 &&
              p.status().min_energy_j == 0);
        CHECK(!p.status().session_allowed && !p.status().inhibited);
        CHECK(!p.status().stop_requested && p.status().speed_scale == 0 &&
              p.status().drive_scale == 0);
        CHECK(p.step(328, meter(328), false, true).phase == PowerPhase::Ready);
    }
    { // A short low-energy glitch must not start a stop.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        ChassisPower p(cfg);
        p.step(0, meter(0), false, true);
        p.step(4, meter(4), true, false);
        for (uint32_t t = 8; t < 100; t += 4)
            CHECK(p.step(t, meter(t, 70), true, false).phase == PowerPhase::Driving);
        CHECK(p.step(100, meter(100, 400), true, false).phase == PowerPhase::Driving);
        CHECK(!p.status().stop_requested && p.status().session_allowed);
    }
    { // Strict policy: meter loss has a finite stopping window.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.require_meter = true;
        cfg.timeout_ms = 20;
        cfg.stopping_timeout_ms = 100;
        ChassisPower p(cfg);
        p.step(0, meter(0), false, true);
        p.step(4, meter(4), true, false);
        auto stale = meter(4);
        CHECK(p.step(28, stale, true, false).phase == PowerPhase::StopFeedback);
        CHECK(p.status().session_allowed && !p.status().meter_online && p.status().stop_requested);
        CHECK(p.step(124, stale, true, false).session_allowed);
        CHECK(p.step(128, stale, true, false).phase == PowerPhase::Inhibited);
        CHECK(p.status().reason == PowerBlock::StopTimeout && !p.status().session_allowed);
        float out[2];
        const float c[2] = {-100, -100}, rpm[2] = {100, 100};
        p.limit(c, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0);
        CHECK(!p.step(132, meter(132), true, false).session_allowed);
        auto invalid = cfg;
        invalid.budget_w = 50;
        ChassisPower unsafe(invalid);
        CHECK(!unsafe.status().configuration_valid);
        ChassisPower wrap(cfg);
        wrap.step(0xfffffff0u, meter(0xfffffff0u), false, true);
        CHECK(wrap.step(0, meter(0), true, false).session_allowed);
    }
    { // Strict policy: meter loss immediately after the first granted cycle still
      // permits bounded braking instead of a silent no-output state.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.require_meter = true;
        ChassisPower p(cfg);
        CHECK(p.step(0, meter(0), false, true).session_allowed);
        PowerMeterStatus absent{};
        CHECK(p.step(4, absent, true, false).phase == PowerPhase::StopFeedback);
        CHECK(p.status().stop_requested && p.status().session_allowed);
        CHECK(p.step(8, absent, false, false).phase == PowerPhase::Inhibited);
        CHECK(p.status().reason == PowerBlock::ExternalStop && !p.status().session_allowed);
        CHECK(p.clear(12, meter(12), true));
        CHECK(!p.status().session_allowed && !p.status().stop_requested && !p.status().inhibited);
        CHECK(p.status().speed_scale == 0 && p.status().drive_scale == 0);
        float out[2];
        const float candidate[2] = {-500, -500}, rpm[2] = {100, 100};
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0); // Clearing never emits a deferred brake/start.
    }
    { // M1: meter loss holds the last limit factor, then ramps it out; no inhibit.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true; // require_meter defaults to false = M1 policy.
        cfg.timeout_ms = 20;
        cfg.meter_hold_ms = 100;
        cfg.meter_stale_ramp_ms = 200;
        cfg.meter_recovery_ramp_ms = 200;
        cfg.power_filter_ms = 4;
        ChassisPower p(cfg);
        p.step(0, meter(0, 400, 10), false, true);
        for (uint32_t t = 4; t <= 40; t += 4)
            p.step(t, meter(t, 400, 90), true, false);
        CHECK(p.status().speed_scale > .4f && p.status().speed_scale < .6f);
        const float held = p.status().speed_scale;
        const PowerMeterStatus stale = meter(40, 400, 90);
        p.step(64, stale, true, false); // 24ms since the last frame, inside the hold.
        CHECK(p.status().meter_held && !p.status().meter_stale && p.status().meter_online == 0);
        CHECK(!p.status().stop_requested && p.status().session_allowed &&
              p.status().phase == PowerPhase::Driving);
        CHECK(near(p.status().speed_scale, held));
        float out[2];
        const float candidate[2] = {1000, -1000}, rpm[2] = {100, 100};
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 1000 && out[1] == -1000);
        p.step(124, stale, true, false); // 84ms: still held.
        CHECK(p.status().meter_held && near(p.status().speed_scale, held));
        p.step(144, stale, true, false); // 104ms: hold expired, ramp starts at held.
        CHECK(p.status().meter_stale && !p.status().meter_held);
        CHECK(near(p.status().speed_scale, held) && !p.status().stop_requested);
        p.step(244, stale, true, false); // Half of the 200ms stale ramp.
        CHECK(p.status().speed_scale > held && p.status().speed_scale < 1.0f);
        CHECK(near(p.status().speed_scale, (held + 1.0f) * .5f, 0.002f));
        p.step(344, stale, true, false); // Stale ramp complete; still driving.
        CHECK(near(p.status().speed_scale, 1.0f) && !p.status().inhibited &&
              p.status().session_allowed);
        // Fresh data returns: ramp back to the newly computed limit, no clear.
        p.step(348, meter(348, 400, 90), true, false);
        CHECK(p.status().meter_online && !p.status().meter_stale &&
              !p.status().stop_requested);
        CHECK(p.status().speed_scale > .5f && p.status().speed_scale <= 1.0f);
        p.step(448, meter(448, 400, 90), true, false); // Half of the recovery ramp.
        CHECK(near(p.status().speed_scale, .75f, 0.002f));
        p.step(548, meter(548, 400, 90), true, false); // Recovery complete.
        CHECK(near(p.status().speed_scale, .5f, 0.002f));
    }
    { // M1: low-energy confirmation needs fresh frames; a lost link cannot extend
      // the debounce, and a confirmed stop is never auto-cancelled by recovery.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.timeout_ms = 20;
        cfg.meter_hold_ms = 100;
        cfg.meter_stale_ramp_ms = 100;
        cfg.meter_recovery_ramp_ms = 100;
        cfg.power_filter_ms = 4;
        ChassisPower p(cfg);
        p.step(0, meter(0, 70, 10), false, true);
        p.step(4, meter(4, 70, 10), true, false);
        p.step(8, meter(8, 70, 10), true, false);
        const PowerMeterStatus stale_low = meter(8, 70, 10);
        p.step(32, stale_low, true, false); // Timeout breaks the low-energy timer.
        CHECK(p.status().meter_held && !p.status().stop_requested &&
              p.status().phase == PowerPhase::Driving);
        p.step(132, stale_low, true, false); // Hold expired; still no low-energy stop.
        CHECK(p.status().meter_stale && p.status().phase == PowerPhase::Driving &&
              !p.status().stop_requested && !p.status().inhibited);
        p.step(236, meter(236, 400, 10), true, false); // Recovery is not a stop.
        CHECK(p.status().meter_online && p.status().phase == PowerPhase::Driving &&
              !p.status().stop_requested);

        // Re-reading the same low frame for 100ms must not confirm low energy.
        auto dwell_cfg = kChassisPowerConfig;
        dwell_cfg.confirmed = true;
        ChassisPower dwell(dwell_cfg);
        const PowerMeterStatus one_low = meter(0, 70, 10);
        dwell.step(0, one_low, false, true);
        for (uint32_t t = 4; t <= 104; t += 4)
            dwell.step(t, one_low, true, false);
        CHECK(dwell.status().phase != PowerPhase::StopLowEnergy &&
              !dwell.status().stop_requested && dwell.status().session_allowed);

        auto stop_cfg = cfg;
        stop_cfg.stopping_timeout_ms = 1000;
        ChassisPower stop(stop_cfg);
        stop.step(0, meter(0, 70, 10), false, true);
        for (uint32_t t = 4; t <= 104; t += 4)
            stop.step(t, meter(t, 70, 10), true, false);
        CHECK(stop.status().phase == PowerPhase::StopLowEnergy);
        const PowerMeterStatus stop_stale = meter(104, 70, 10);
        stop.step(132, stop_stale, true, false); // Meter loss must not cancel it.
        CHECK(stop.status().phase == PowerPhase::StopLowEnergy &&
              stop.status().stop_requested && stop.status().session_allowed);
        stop.step(136, stop_stale, true, true);
        CHECK(stop.status().phase == PowerPhase::Inhibited && stop.status().inhibited);
    }
    { // M1: no first meter frame at all no longer blocks the chassis.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.meter_stale_ramp_ms = 0;
        ChassisPower p(cfg);
        PowerMeterStatus empty{};
        CHECK(p.step(0, empty, false, true).session_allowed);
        CHECK(p.status().phase == PowerPhase::Ready && p.status().meter_stale &&
              p.status().reason == PowerBlock::NoMeter);
        CHECK(p.step(4, empty, true, false).phase == PowerPhase::Driving);
        CHECK(p.status().session_allowed && !p.status().stop_requested &&
              !p.status().inhibited);
        float out[2];
        const float candidate[2] = {1000, -1000}, rpm[2] = {100, 100};
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 1000 && out[1] == -1000);
    }
    { // Real healthy snapshots permit slow drive before active; no startup deadlock.
        VehicleConfig cfg{};
        cfg.posture_confirmed = true;
        VehicleChassisInput ch{};
        VehicleArmInput arm{};
        VehicleImuInput imu{};
        sources(100, ch, arm, imu);
        auto s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.healthy && s.locks.chassis_stopped && s.locks.arm_allows_drive);
        CHECK(!s.locks.arm_allows_fast && s.locks.drive_yaw_allowed);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        CHECK(s.locks.drive_yaw_max_speed_deg_s == cfg.drive_yaw_speed_deg_s);
        s = evaluate(104, cfg, ch, arm, imu, 50);
        CHECK(s.locks.drive_yaw_max_deg == cfg.drive_yaw_deg);
        sources(100, ch, arm, imu);
        ch.power_confirmed = 0;
        ch.power_session_allowed = 0;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(!(s.reasons & VehicleEnergyInhibited)); // Unconfirmed power is not an energy stop.
        CHECK(s.reasons & VehicleConfiguration);
        for (unsigned bad = 0; bad < 13; ++bad) {
            sources(100, ch, arm, imu);
            auto conf = cfg;
            if (bad == 0)
                ch.updated_ms = 0;
            if (bad == 1)
                arm.updated_ms = 0;
            if (bad == 2)
                imu.updated_ms = 0;
            if (bad == 3)
                imu.valid = 0;
            if (bad == 4)
                imu.tipped = 1;
            if (bad == 5)
                ch.feedback_received[0] = false;
            if (bad == 6)
                ch.feedback_ms[1] = 0;
            if (bad == 7)
                ch.power_session_allowed = 0;
            if (bad == 8)
                conf.posture_confirmed = false;
            if (bad == 9)
                ch.hardware_confirmed = 0;
            if (bad == 10)
                ch.power_configuration_valid = 0;
            if (bad == 11)
                ch.power_confirmed = 0;
            if (bad == 12)
                imu.configuration_valid = 0;
            s = evaluate(100, conf, ch, arm, imu, 10);
            CHECK(!s.healthy && s.reasons && !s.locks.arm_allows_drive && !s.locks.arm_allows_fast);
        }
        sources(100, ch, arm, imu);
        arm.big_pitch_deg = cfg.drive_pitch_deg + 1;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.all_positions_finite = false;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.all_calibrated = false;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        ch.power_stop_requested = 1;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.healthy && !s.locks.arm_allows_drive && s.locks.drive_yaw_allowed);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        sources(100, ch, arm, imu);
        arm.yaw_deg = 11;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.locks.drive_yaw_allowed && !s.locks.arm_allows_drive);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        sources(100, ch, arm, imu);
        arm.yaw_deg = 21;
        s = evaluate(100, cfg, ch, arm, imu, 50);
        CHECK(s.healthy && s.locks.drive_yaw_allowed && !s.locks.arm_allows_drive);
        CHECK(s.locks.drive_yaw_min_deg == -20 && s.locks.drive_yaw_max_deg == 20);
        sources(100, ch, arm, imu);
        arm.faults = 1;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons & VehicleArmFault);
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 40;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK((s.reasons & VehicleArmStale) && !s.healthy && !s.locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 30;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10)
                  .locks.arm_allows_drive); // Exact combined source-age boundary.
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 0xfffffff0u;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons &
              VehicleArmStale); // No uint32_t addition wrap.
        sources(100, ch, arm, imu);
        arm.all_feedback_received = false;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons & VehicleArmStale);
        sources(100, ch, arm, imu);
        CHECK(!evaluate(100, cfg, ch, arm, imu, std::numeric_limits<float>::quiet_NaN()).healthy);
        // Publishing a fresh snapshot does not make stale wheel/sensor timestamps fresh.
        ch.updated_ms = 120;
        arm.updated_ms = 120;
        imu.updated_ms = 120;
        imu.received_ms = 90;
        CHECK(evaluate(120, cfg, ch, arm, imu, 10).reasons & VehicleImuInvalid);
    }
    { // The task bridge retains each actual cause and rejects malformed health.
        VehicleHealth health{100, 0, 1, 1};
        const uint32_t timeout = kVehicleConfig.status_timeout_ms;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == 0);
        CHECK(Vehicle_ProtectionFromHealth(100, nullptr, timeout) == PROTECT_VEHICLE_STATE);
        CHECK(Vehicle_ProtectionFromHealth(160, &health, timeout) == 0);
        CHECK(Vehicle_ProtectionFromHealth(161, &health, timeout) == PROTECT_VEHICLE_STATE);
        const uint32_t faults[] = {VehicleConfiguration,   VehicleChassisStale, VehicleArmStale,
                                   VehicleImuInvalid,      VehicleTipped,       VehicleWheelFault,
                                   VehicleEnergyInhibited, VehicleArmFault};
        const uint32_t expected[] = {
            PROTECT_VEHICLE_STATE, PROTECT_CHASSIS, PROTECT_VEHICLE_STATE, PROTECT_IMU,
            PROTECT_TIPPED,        PROTECT_CHASSIS, PROTECT_POWER,         PROTECT_VEHICLE_STATE};
        for (unsigned i = 0; i < 8; ++i) {
            health.reasons = faults[i];
            health.healthy = 0;
            CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == expected[i]);
        }
        health.reasons = 0xff;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) ==
              (PROTECT_VEHICLE_STATE | PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED |
               PROTECT_POWER));
        health.reasons = 0x100;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.reasons = 0;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.healthy = 2;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.healthy = 1;
        health.ready = 0;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health = {0xfffffff0u, 0, 1, 1};
        CHECK(Vehicle_ProtectionFromHealth(16, &health, timeout) == 0);
    }
    { // Permission expires at the original source deadline, including after a revoke.
        InsCalibrationPermission permission{};
        CHECK(!INS_CalibrationPermissionValid(&permission, 0, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(nullptr, 0, 0, 0));
        permission = {100, 90, 30, 1, 1};
        CHECK(INS_CalibrationPermissionValid(&permission, 130, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(&permission, 131, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(&permission, 99, 0, 0));
        permission.valid_for_ms = 31;
        CHECK(!INS_CalibrationPermissionValid(&permission, 100, 0, 0));
        permission = {100, 50, 0, 1, 1};
        CHECK(!INS_CalibrationPermissionValid(&permission, 100, 0, 0));
        permission = {120, 100, 20, 1, 1};
        CHECK(!INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission.oldest_source_updated_ms = 101;
        permission.valid_for_ms = 21;
        CHECK(INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission.allowed = 0;
        CHECK(!INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission = {0xfffffff0u, 0xfffffff0u, 40, 1, 1};
        CHECK(INS_CalibrationPermissionValid(&permission, 16, 1, 0xffffffe0u));
        CHECK(!INS_CalibrationPermissionValid(&permission, 25, 0, 0));

        auto cfg = attitude_config();
        AttitudeEstimator attitude(cfg);
        permission = {0, 0, 32, 1, 1};
        for (unsigned n = 0; n < 9; ++n) {
            const uint32_t now = n * 4;
            attitude.step(sample(now, n + 1), .004f,
                          INS_CalibrationPermissionValid(&permission, now, 0, 0));
        }
        CHECK(attitude.status().calibration_count == 9 && !attitude.status().calibrated);
        attitude.step(sample(36, 10), .004f, INS_CalibrationPermissionValid(&permission, 36, 0, 0));
        CHECK(attitude.status().calibration_count == 0 && !attitude.status().calibrated);
        permission = {40, 40, 40, 1, 1};
        attitude.step(sample(40, 11), .004f,
                      INS_CalibrationPermissionValid(&permission, 40, 1, 36));
        CHECK(attitude.status().calibration_count == 1 && !attitude.status().calibrated);
    }
    { // Static calibration is gated by all outputs disabled and enough continuous samples.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 20; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(!a.status().calibrated && !a.status().valid);
        a.restart_calibration();
        for (unsigned n = 20; n < 29; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(!a.status().calibrated);
        a.step(sample(116, 30), .004f, true);
        CHECK(a.status().calibrated && a.status().valid && a.status().tilt_deg < .01f);
        AttitudeConfig unconfirmed_attitude{};
        unconfirmed_attitude.installation_confirmed = false;
        unconfirmed_attitude.thresholds_confirmed = false;
        AttitudeEstimator defaults(unconfirmed_attitude);
        for (unsigned n = 0; n < 600; ++n)
            defaults.step(sample(n * 4, n + 1), .004f, true);
        CHECK(defaults.status().configuration_valid && !defaults.status().confirmed &&
              !defaults.status().valid);
        auto reflected = cfg;
        reflected.sensor_sign[0] = -1;
        AttitudeEstimator bad(reflected);
        CHECK(!bad.status().configuration_valid);
        reflected.sensor_axis[0] = 1;
        reflected.sensor_axis[1] = 0;
        AttitudeEstimator right_handed(reflected);
        CHECK(right_handed.status().configuration_valid);
    }
    { // Flip/release persistence and hysteresis; a brief tilt is not a latched tip.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        unsigned n = 10;
        for (; n < 20; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, -9.80665f), .004f, false);
        CHECK(!a.status().tipped); // less than trip_ms
        for (; n < 80; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, -9.80665f), .004f, false);
        CHECK(a.status().tipped && a.status().tilt_deg > 179);
        for (; n < 140; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(a.status().tipped); // release requires 500 ms, not one flat sample.
        for (; n < 260; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(a.status().valid && !a.status().tipped && a.status().tilt_deg < .1f);
        for (; n < 410; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, 20), .004f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeAccelUntrusted));
        a.step(sample(n * 4, n + 1), .004f, false);
        ++n;
        CHECK(a.status().valid);
        auto bad = sample(n * 4, n + 1);
        bad.accel_m_s2[0] = std::numeric_limits<float>::quiet_NaN();
        a.step(bad, .004f, false);
        ++n;
        CHECK(!a.status().valid && (a.status().faults & AttitudeSampleInvalid));
        a.step(sample(n * 4, n + 1), .004f, false);
        ++n;
        a.step(sample(n * 4 + 100, n + 1), .1f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeTiming));
    }
    { // Tilt thresholds are tunable; invalid pairs are ignored.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(a.status().calibrated && a.status().valid);
        a.set_tilt_tuning(20, 10, 8, 8);
        CHECK(near(a.config().trip_tilt_deg, 20) && near(a.config().release_tilt_deg, 10));
        const float ax = 9.80665f * .5f, az = 9.80665f * .8660254f;
        unsigned n = 10;
        for (; n < 20; ++n)
            a.step(sample(n * 4, n + 1, ax, 0, az), .004f, false);
        CHECK(a.status().tipped);
        a.set_tilt_tuning(50, 35, 8, 8);
        for (; n < 30; ++n)
            a.step(sample(n * 4, n + 1, ax, 0, az), .004f, false);
        CHECK(!a.status().tipped);
        a.set_tilt_tuning(10, 40, 8, 8); // release must stay below trip
        CHECK(near(a.config().trip_tilt_deg, 50) && near(a.config().release_tilt_deg, 35));
    }
    { // Any invalid sample interrupts a continuous static-calibration window.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 9; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(a.status().calibration_count == 9 && !a.status().calibrated);
        auto invalid = sample(36, 10);
        invalid.gyro_rad_s[0] = std::numeric_limits<float>::quiet_NaN();
        a.step(invalid, .004f, true);
        CHECK(!a.status().valid && a.status().calibration_count == 0);
        a.step(sample(40, 11), .004f, true);
        CHECK(!a.status().calibrated && a.status().calibration_count == 1);
        a.step(sample(44, 12), .004f, false);
        CHECK(!a.status().calibrated && a.status().calibration_count == 0);
        for (unsigned n = 12; n < 21; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        a.invalidate(84);
        CHECK(a.status().calibration_count == 0);
        a.step(sample(84, 22), .004f, true);
        CHECK(!a.status().calibrated && a.status().calibration_count == 1);
    }
    { // Bias subtraction, duplicate frame rejection and unsigned timestamp rollover.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n) {
            auto s = sample(n * 4, n + 1);
            s.gyro_rad_s[0] = .01f;
            a.step(s, .004f, true);
        }
        CHECK(a.status().valid && near(a.status().gyro_bias_rad_s[0], .01f, .00001f));
        a.step(sample(40, 10), .004f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeSampleInvalid));
        AttitudeEstimator wrap(cfg);
        for (unsigned n = 0; n < 10; ++n)
            wrap.step(sample(0xfffffff0u + n * 4, n + 1), .004f, true);
        CHECK(wrap.status().valid && !(wrap.status().faults & AttitudeTiming));
        wrap.invalidate(100);
        CHECK(!wrap.status().valid && wrap.status().updated_ms == 100);
    }
    { // The installation map rotates accel and gyro into the same body frame.
        auto cfg = attitude_config();
        cfg.sensor_axis[0] = 1;
        cfg.sensor_axis[1] = 0;
        cfg.sensor_sign[0] = 1;
        cfg.sensor_sign[1] = -1;
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(a.status().valid && a.status().configuration_valid);
        auto moving = sample(40, 11);
        moving.gyro_rad_s[0] = 1;
        a.step(moving, .004f, false);
        CHECK(a.status().valid && near(a.status().gyro_body_rad_s[0], 0));
        CHECK(near(a.status().gyro_body_rad_s[1], -1) && near(a.status().gyro_body_rad_s[2], 0));
        CHECK(near(a.status().accel_body_m_s2[2], 9.80665f));
        CHECK(a.status().up_body[0] > 0 && near(a.status().up_body[1], 0));
    }
    return 0;
}
