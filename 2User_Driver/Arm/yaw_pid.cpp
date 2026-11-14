#include "Arm/yaw_pid.h"
#include <cmath>

namespace engineer {
namespace {
float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
bool finite(float x) {
    return std::isfinite(x);
}
} // namespace

void YawPid_Init(YawPid &pid, const YawPidConfig &c) {
    pid = {};
    pid.kp = c.kp;
    pid.ki = c.ki;
    pid.kd = c.kd;
    pid.max_i_error = c.max_i_error;
    pid.max_sigma = c.max_sigma;
    pid.max_output = c.max_output;
    pid.d_tau = c.d_tau;
    YawPid_Reset(pid);
}

void YawPid_Reset(YawPid &pid) {
    pid.error = pid.error_delta = pid.sigma_error = pid.last_error = 0;
    pid.last_feedback = pid.d_lpf = pid.output = pid.output_prelimit = 0;
    pid.first_run = true;
}

float YawPid_Calculate(YawPid &pid, float target, float feedback, float dt,
                       float feedforward, float final_output_limit, bool allow_positive,
                       bool allow_negative) {
    if (!finite(target) || !finite(feedback) || !finite(dt) || dt <= 0 ||
        !finite(feedforward) || !finite(final_output_limit) || final_output_limit < 0)
        return pid.output;

    float raw_d = 0;
    if (pid.first_run) {
        pid.last_feedback = feedback;
        pid.last_error = target - feedback;
        pid.first_run = false;
    } else {
        // Measurement derivative: a target step must not create a derivative kick.
        raw_d = -(feedback - pid.last_feedback) / dt;
    }

    pid.error = target - feedback;
    pid.error_delta = pid.error - pid.last_error;
    const float alpha = pid.d_tau > 0 ? dt / (pid.d_tau + dt) : 1.0f;
    pid.d_lpf = (1.0f - alpha) * pid.d_lpf + alpha * raw_d;

    float new_sigma = pid.sigma_error;
    // Integral separation: stop integrating far from the target, but always
    // allow a reverse error to unwind the existing integral.
    const bool unwinding = pid.error * pid.sigma_error < 0;
    if (std::fabs(pid.error) < pid.max_i_error || unwinding) {
        new_sigma += pid.error * dt;
        // Outside the separation band, unwind only as far as zero.
        if (unwinding && std::fabs(pid.error) >= pid.max_i_error &&
            new_sigma * pid.sigma_error < 0)
            new_sigma = 0;
    }
    if (pid.max_sigma > 0)
        new_sigma = clampf(new_sigma, -pid.max_sigma, pid.max_sigma);
    else
        new_sigma = 0;

    const float p_out = pid.kp * pid.error;
    float i_out = pid.ki * new_sigma;
    const float d_out = pid.kd * pid.d_lpf;
    float total = p_out + i_out + d_out + feedforward;
    const float lo = allow_negative ? -final_output_limit : 0;
    const float hi = allow_positive ? final_output_limit : 0;
    float final = clampf(total, lo, hi);

    // Conditional integration: do not accumulate while the final command is
    // saturated in the same direction as the error.
    if (total != final && pid.error * (total - final) > 0 && !unwinding) {
        new_sigma = pid.sigma_error;
        i_out = pid.ki * new_sigma;
        total = p_out + i_out + d_out + feedforward;
        final = clampf(total, lo, hi);
    }

    pid.sigma_error = new_sigma;
    pid.last_error = pid.error;
    pid.last_feedback = feedback;
    pid.output_prelimit = total;
    pid.output = final;
    return final;
}

} // namespace engineer
