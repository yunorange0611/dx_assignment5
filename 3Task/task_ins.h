#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t updated_ms, received_ms, frames, faults, calibration_count;
    uint32_t driver_errors, driver_attempts, driver_read_failures;
    float accel_body_m_s2[3], gyro_body_rad_s[3], gyro_bias_rad_s[3], up_body[3];
    float tilt_deg, accel_norm_m_s2, temperature_c;
    uint8_t ready, configuration_valid, confirmed, calibrated, valid, accel_trusted;
    uint8_t tipped, calibrating, driver_phase, accel_id, gyro_id;
} InsStatus;
extern InsStatus g_ins_status; // Read-only snapshot; use getter across tasks.
// 0 idle, 1 request static bias calibration. Consumed by ins_task; never enables.
extern volatile uint32_t g_ins_debug_command;
int INS_GetStatus(InsStatus *out);
void ins_task(void *argument);
#ifdef __cplusplus
}
#endif
