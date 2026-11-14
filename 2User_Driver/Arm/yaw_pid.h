#pragma once
// Yaw-only PID migrated from project4's PID_Calculate pattern.
// Pure control math: no HAL, RTOS, CAN or task dependencies.
namespace engineer {

struct YawPidConfig {
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float max_i_error = 0; // integrate while |error| is below this; reverse error always integrates
    float max_sigma = 0;   // limit of the error integral, not the Ki-scaled output
    float max_output = 0;  // non-negative output magnitude
    float d_tau = 0.01f;   // derivative low-pass time constant, seconds
};

struct YawPid {
    float kp = 0, ki = 0, kd = 0;
    float max_i_error = 0, max_sigma = 0, max_output = 0, d_tau = 0.01f;
    float error = 0, error_delta = 0, sigma_error = 0, last_error = 0;
    float last_feedback = 0, d_lpf = 0, output = 0, output_prelimit = 0;
    bool first_run = true;
};

void YawPid_Init(YawPid &pid, const YawPidConfig &config);
void YawPid_Reset(YawPid &pid);

// Exact project4 PID_Calculate math; feedforward/final device limits are external.
float YawPid_Calculate(YawPid &pid, float target, float feedback, float dt);

} // namespace engineer

extern "C" {
extern volatile float tune_yaw_position_ki, tune_yaw_position_kd;
extern volatile float tune_yaw_position_max_i_error, tune_yaw_position_max_sigma;
extern volatile float tune_yaw_position_max_output, tune_yaw_position_d_tau;
extern volatile float tune_yaw_velocity_kd, tune_yaw_velocity_max_i_error;
extern volatile float tune_yaw_velocity_max_sigma, tune_yaw_velocity_d_tau;
}
