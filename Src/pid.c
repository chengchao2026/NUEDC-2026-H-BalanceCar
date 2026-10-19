#include "pid.h"

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void PID_Init(PID_T *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit,
              float derivative_alpha)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->KI_MAX = integral_limit;
    pid->OUT_MAX = output_limit;
    pid->integral_min = -integral_limit;
    pid->integral_max = integral_limit;
    pid->output_min = -output_limit;
    pid->output_max = output_limit;
    pid->derivative_alpha = PID_Clamp(derivative_alpha, 0.0f, 1.0f);
    PID_Reset(pid);
}

void PID_Reset(PID_T *pid)
{
    if (pid == 0) return;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->out = 0.0f;
    pid->kpout = 0.0f;
    pid->kiout = 0.0f;
    pid->kdout = 0.0f;
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->Errorint = 0.0f;
    pid->derivative = 0.0f;
    pid->initialized = 0U;
}

float PID_Update(PID_T *pid, float target, float actual, float dt_s)
{
    float error;
    float raw_derivative;
    float candidate_integral;
    float unsaturated_output;
    float saturated_output;
    uint8_t allow_integral;

    if (pid == 0 || dt_s <= 0.0f) return 0.0f;

    /* 允许旧调参程序继续修改 OUT_MAX/KI_MAX，并同步到显式上下限。 */
    if (pid->OUT_MAX > 0.0f) {
        pid->output_min = -pid->OUT_MAX;
        pid->output_max = pid->OUT_MAX;
    }
    if (pid->KI_MAX > 0.0f) {
        pid->integral_min = -pid->KI_MAX;
        pid->integral_max = pid->KI_MAX;
    }

    pid->target = target;
    pid->actual = actual;
    pid->Error1 = pid->Error0;
    error = target - actual;
    pid->Error0 = error;

    if (pid->initialized == 0U) {
        raw_derivative = 0.0f;
        pid->initialized = 1U;
    } else {
        raw_derivative = (pid->Error0 - pid->Error1) / dt_s;
    }

    /* 微分项再做一次轻量低通，降低步数角度量化和视觉残余噪声。 */
    pid->derivative += pid->derivative_alpha *
                       (raw_derivative - pid->derivative);

    candidate_integral = PID_Clamp(pid->Errorint + error * dt_s,
                                   pid->integral_min,
                                   pid->integral_max);
    pid->kpout = pid->kp * error;
    pid->kiout = pid->ki * candidate_integral;
    pid->kdout = pid->kd * pid->derivative;
    unsaturated_output = pid->kpout + pid->kiout + pid->kdout;
    saturated_output = PID_Clamp(unsaturated_output,
                                 pid->output_min,
                                 pid->output_max);

    /* 条件积分抗饱和：输出饱和时，仅允许能把输出拉回线性区的积分方向。 */
    allow_integral = (unsaturated_output == saturated_output) ||
                     ((unsaturated_output > pid->output_max) && (error < 0.0f)) ||
                     ((unsaturated_output < pid->output_min) && (error > 0.0f));
    if (allow_integral != 0U) {
        pid->Errorint = candidate_integral;
    }

    pid->kiout = pid->ki * pid->Errorint;
    pid->out = PID_Clamp(pid->kpout + pid->kiout + pid->kdout,
                         pid->output_min,
                         pid->output_max);
    return pid->out;
}

float PID_motor(PID_T *pid, int16_t actual_pos)
{
    return PID_Update(pid, pid->target, (float)actual_pos, 0.001f);
}
