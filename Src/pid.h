#ifndef PID_H
#define PID_H

#include <stdint.h>

/*
 * 浮点 PID 控制器。
 * STM32F407 带单精度硬件 FPU，因此控制链中的增益、积分、微分和限幅
 * 全部使用 float，避免定点缩放造成的量化误差。
 */
typedef struct pid_t
{
    float kp;
    float ki;
    float kd;

    float target;
    float actual;
    float out;

    float kpout;
    float kiout;
    float kdout;

    float Error0;
    float Error1;
    float Errorint;

    /* 保留原工程字段名，便于现有串口调参代码继续工作。 */
    float OUT_MAX;
    float KI_MAX;

    /* 新增的显式上下限及状态。 */
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float derivative;
    float derivative_alpha;
    uint8_t initialized;
} PID_T;

void PID_Init(PID_T *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit,
              float derivative_alpha);
void PID_Reset(PID_T *pid);
float PID_Update(PID_T *pid, float target, float actual, float dt_s);

/* 兼容原工程接口；新串级控制使用 PID_Update。 */
float PID_motor(PID_T *pid, int16_t actual_pos);

#endif
