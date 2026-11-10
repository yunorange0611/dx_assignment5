#include "vehicle_state.h"
#include "chassis_config.h"
#include "arm_control.h"
#include "attitude_estimator.h"
#include <cmath>
namespace engineer {
VehicleState evaluate_vehicle(uint32_t now,const VehicleConfig &cfg,const ChassisStatus &ch,
                              const ArmStatus &arm,const InsStatus &imu,float yaw_half) {
    VehicleState s{}; s.updated_ms=now;
    const bool config=cfg.posture_confirmed && cfg.status_timeout_ms>0 &&
        std::isfinite(cfg.drive_pitch_deg) && cfg.drive_pitch_deg>0 &&
        std::isfinite(cfg.drive_roll_deg) && cfg.drive_roll_deg>0 &&
        std::isfinite(cfg.drive_yaw_deg) && cfg.drive_yaw_deg>0 &&
        std::isfinite(cfg.drive_yaw_speed_deg_s) && cfg.drive_yaw_speed_deg_s>0 &&
        std::isfinite(yaw_half) && yaw_half>0;
    if(!config || !ch.hardware_confirmed || !ch.power_configuration_valid || !ch.power_confirmed)
        s.reasons|=VehicleConfiguration;
    const bool chassis_fresh=ch.ready && uint32_t(now-ch.updated_ms)<=cfg.status_timeout_ms;
    if(!chassis_fresh) s.reasons|=VehicleChassisStale;
    if(!ch.configuration_valid || ch.faults) s.reasons|=VehicleWheelFault;
    for(unsigned i=0;i<2;++i)
        if(!ch.feedback_frames[i] || uint32_t(now-ch.feedback_ms[i])>kChassisConfig.feedback_timeout_ms) s.reasons|=VehicleWheelFault;
    const bool arm_fresh=arm.ready && uint32_t(now-arm.updated_ms)<=cfg.status_timeout_ms;
    if(!arm_fresh) s.reasons|=VehicleArmStale;
    if(arm.faults || !arm.configuration_valid) s.reasons|=VehicleArmFault;
    bool arm_feedback_fresh=true;
    for(const auto &j:arm.joint)
        arm_feedback_fresh &= j.received &&
            uint64_t(j.feedback_age_ms)+uint32_t(now-arm.updated_ms)<=kArmConfig.feedback_timeout_ms;
    if(!arm_feedback_fresh) s.reasons|=VehicleArmStale;
    if(!imu.ready || !imu.configuration_valid || !imu.valid || !imu.confirmed || !imu.calibrated || imu.faults ||
        uint32_t(now-imu.updated_ms)>cfg.status_timeout_ms ||
        uint32_t(now-imu.received_ms)>kAttitudeConfig.max_sample_gap_ms) s.reasons|=VehicleImuInvalid;
    if(imu.tipped) s.reasons|=VehicleTipped;
    if(!ch.power_session_allowed) s.reasons|=VehicleEnergyInhibited;
    s.healthy=s.reasons==0;
    s.locks.chassis_stopped=chassis_fresh && ch.wheels_stationary && !(s.reasons & VehicleWheelFault);
    bool posture=arm_fresh && arm_feedback_fresh && !arm.faults && arm.configuration_valid;
    for(unsigned i=0;i<ARM_JOINT_COUNT;++i) {
        const auto &j=arm.joint[i];
        posture &= j.online && j.calibrated && std::isfinite(j.position_deg);
    }
    posture &= std::fabs(arm.joint[ARM_BIG_PITCH].position_deg)<=cfg.drive_pitch_deg &&
        std::fabs(arm.joint[ARM_SMALL_PITCH].position_deg)<=cfg.drive_pitch_deg &&
        std::fabs(arm.joint[ARM_ROLL].position_deg)<=cfg.drive_roll_deg;
    s.locks.arm_allows_drive=s.healthy && posture && !ch.power_stop_requested;
    s.locks.arm_allows_fast=0; // Fast is intentionally deferred.
    const float half=std::fmin(yaw_half,cfg.drive_yaw_deg);
    // Keep the known envelope present: an out-of-range angle must block driving,
    // not erase the envelope and accidentally permit unrestricted driving.
    s.locks.drive_yaw_allowed=s.healthy && posture;
    s.locks.arm_allows_drive &= std::fabs(arm.joint[ARM_YAW].position_deg)<=half;
    if(s.locks.drive_yaw_allowed) {
        s.locks.drive_yaw_min_deg=-half; s.locks.drive_yaw_max_deg=half;
        s.locks.drive_yaw_max_speed_deg_s=cfg.drive_yaw_speed_deg_s;
    }
    return s;
}
}
