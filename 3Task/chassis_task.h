#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t updated_ms, faults, rx_dropped, tx_errors;
    uint32_t feedback_ms[2], feedback_frames[2], power_received_ms, power_frames;
    float measured_rpm[2], target_rpm[2], current_ma[2], temperature_c[2];
    float v_m_s, omega_rad_s, power_w, energy_j;
    uint8_t ready, configuration_valid, hardware_confirmed, enabled, wheels_stationary,
        bench_active;
    uint8_t power_configuration_valid, power_confirmed;
    uint8_t power_session_allowed, power_stop_requested, power_inhibited, power_phase, power_reason;
} ChassisStatus;
extern ChassisStatus g_chassis_status; // Read-only snapshot.
enum ChassisDebugMode {
    CHASSIS_DEBUG_STOP,
    CHASSIS_DEBUG_CLEAR,
    CHASSIS_DEBUG_CURRENT,
    CHASSIS_DEBUG_SPEED
};
typedef struct {
    uint32_t mode, wheel, duration_ms;
    float value;                // CURRENT=mA (<=300), SPEED=rotor rpm (<=300)
    uint32_t suspended_confirm; // 0x53555350, manual acknowledgement; not a sensor.
    uint32_t request_seq; // Write this last. Requests are finite pulses, never a persistent enable.
} ChassisDebugRequest;
typedef struct {
    uint32_t consumed_seq, result, until_ms;
} ChassisDebugStatus;
extern volatile ChassisDebugRequest g_chassis_debug_request;
extern ChassisDebugStatus g_chassis_debug_status;
// Write all values, then request_seq last. Only accepted while wheel output is
// disabled with fresh stationary feedback; values live in RAM until restart.
typedef struct {
    float velocity_kp, velocity_ki, max_current_ma;
    uint32_t request_seq;
} ChassisTuneRequest;
typedef struct {
    uint32_t consumed_seq, active_seq, result; // 0Idle 1Applied 2Invalid 3Busy 4NotReady
    float velocity_kp, velocity_ki, max_current_ma;
} ChassisTuneStatus;
extern volatile ChassisTuneRequest g_chassis_tune_request;
extern ChassisTuneStatus g_chassis_tune_status; // Read-only active parameters/result.
int Chassis_GetStatus(ChassisStatus *out);
void chassis_task(void *argument);
#ifdef __cplusplus
}
#endif
