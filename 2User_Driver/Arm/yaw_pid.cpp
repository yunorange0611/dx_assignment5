#include "Arm/yaw_pid.h"
#include <cmath>
#include "Arm/yaw_config.h"

extern "C" {
volatile float tune_yaw_position_ki = 0, tune_yaw_position_kd = 0;
volatile float tune_yaw_position_max_i_error = 0, tune_yaw_position_max_sigma = 0;
volatile float tune_yaw_position_max_output = engineer::kYawConfig.max_speed_deg_s;
volatile float tune_yaw_position_d_tau = 0.01f;
volatile float tune_yaw_velocity_kd = 0;
volatile float tune_yaw_velocity_max_i_error = 50.0f;
volatile float tune_yaw_velocity_max_sigma = 200.0f; // deg: independent error-integral bound
volatile float tune_yaw_velocity_d_tau = 0.01f;
}

namespace engineer {
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

float YawPid_Calculate(YawPid &state, float target, float feedback, float dt)
{
    YawPid *pid = &state;
    float raw_d;
    float new_sigma = pid->sigma_error; // 默认冻结积分

    // 1. 首次运行保护：对齐历史数据，防止开机瞬间的微分突变
    if (pid->first_run) {
        pid->last_feedback = feedback;
        pid->last_error = target - feedback;
        raw_d = 0.0f;
        pid->first_run = 0;
    } else {
        // 测量微分 (规避目标值突变带来的微分踢)
        raw_d = -(feedback - pid->last_feedback) / dt;
    }

    // 2. 误差计算
    pid->error = target - feedback;
    pid->error_delta = pid->error - pid->last_error;

    // 3. 动态时间常数低通滤波 (解耦控制周期 dt)
    float alpha = dt / (pid->d_tau + dt);
    pid->d_lpf = (1.0f - alpha) * pid->d_lpf + alpha * raw_d;

    // 4. 积分分离：误差小于阈值才积分；
    //    但误差和已有积分方向相反时（积分在“往回退”）不受阈值限制，照常积分。
    //    否则：反方向攒满的积分 + 大误差 → 积分被冻结退不掉 → 抵消 P、推不动 → 误差一直大 → 死锁
    //    （大 Yaw 位置环实测：一个方向很快到位，另一个方向走不动）
    if (fabsf(pid->error) < pid->max_i_error ||
        (pid->error * pid->sigma_error) < 0.0f) {
        new_sigma += pid->error * dt;
    }

    // 5. 积分限幅
    if (new_sigma > pid->max_sigma) {
        new_sigma = pid->max_sigma;
    } else if (new_sigma < -pid->max_sigma) {
        new_sigma = -pid->max_sigma;
    }

    // 6. 核心计算
    float p_out = pid->kp * pid->error;
    float i_out = pid->ki * new_sigma;
    float d_out = pid->kd * pid->d_lpf;
    float total_out = p_out + i_out + d_out;

    pid->output_prelimit = total_out; // read-only diagnostic, does not affect calculation

    // 7. 输出限幅与精细化抗积分饱和 (条件积分)
    if (total_out > pid->max_output) {
        total_out = pid->max_output;
        // 正饱和且误差还在扩大(同号)，禁止继续正向积分
        if (pid->error > 0.0f) {
            new_sigma = pid->sigma_error;
        }
    } else if (total_out < -pid->max_output) {
        total_out = -pid->max_output;
        // 负饱和且误差还在扩大(同号)，禁止继续负向积分
        if (pid->error < 0.0f) {
            new_sigma = pid->sigma_error;
        }
    }

    // 8. 状态更新
    pid->sigma_error = new_sigma;
    pid->last_error = pid->error;
    pid->last_feedback = feedback;
    pid->output = total_out;

    return pid->output;
}

} // namespace engineer
