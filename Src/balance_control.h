#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "pid.h"
#include <stdint.h>

/* ======================== 集中调参区 ========================
 * 所有量均采用 float，角度单位为 deg，位置单位为 cm，时间单位为 s。
 * 调参时优先按“外环 -> 内环 -> 前馈”的顺序逐级进行。
 */

/* 视觉坐标与滤波。$B 像素协议默认 320 像素覆盖 25 cm 摆杆。 */
#define BAL_VISION_CENTER_PIXEL           323.5f
#define BAL_VISION_ROD_PIXELS             613.0f
#define BAL_ROD_LENGTH_CM                 25.0f
#define BAL_VISION_LPF_ALPHA              0.65f

/* 小球目标位置及外环：位置误差(cm) -> 目标摆杆角度(deg)。
 * 坐标系与 MaixCam 侧标定(AXIS_START_CM=0, AXIS_END_CM=25)保持一致：
 * 0 cm 为摆杆一端，25 cm 为另一端，杆中心在 12.5 cm。 */
#define BAL_BALL_TARGET_CM                12.5f
/* 任务1(PB12)相对"0点"的偏移量；0点默认是 BAL_BALL_TARGET_CM，
 * 也可以在运行时被 PB14 改成球当前所在位置(main.c 的 balance_center_cm)。 */
#define BAL_SEQUENCE_OFFSET_CM             5.0f
#define BAL_SEQUENCE_ARRIVAL_TOLERANCE_CM  0.80f
#define BAL_SEQUENCE_FINAL_MAX_SPEED_CMS    3.0f
#define BAL_SEQUENCE_FINAL_DWELL_MS         300U
#define BAL_SEQUENCE_TOTAL_LIMIT_MS        5000U
#define BAL_OUTER_KP                      2.00f
#define BAL_OUTER_KI                      0.00f
#define BAL_OUTER_KD                      0.45f
#define BAL_OUTER_INTEGRAL_LIMIT          3.0f
#define BAL_TARGET_ANGLE_LIMIT_DEG        1.2f
/* 由 +/-267 步的四连杆模型得到约 -2.63/+2.48 deg，留出机械余量。 */
#define BAL_TOTAL_ANGLE_LIMIT_DEG         2.2f
#define BAL_TARGET_ANGLE_SLEW_DEG_S      60.0f
#define BAL_OUTER_DERIVATIVE_ALPHA        0.55f
#define BAL_BALL_PREDICTION_HORIZON_S      0.120f
#define BAL_BALL_PREDICTION_MAX_LEAD_CM    2.0f
/* 实测球偏移后倾斜方向与外环期望相反，翻转为 -1.0f 修正。 */
#define BAL_OUTER_DIRECTION_SIGN          -1.0f

/* 内环：目标角度与步数估算角度之差 -> PWM 脉冲角速度(deg/s)。 */
#define BAL_INNER_KP                     12.0f
#define BAL_INNER_KI                      0.00f
#define BAL_INNER_KD                      0.05f
#define BAL_INNER_INTEGRAL_LIMIT          8.0f
#define BAL_PWM_OUTPUT_LIMIT_DEG_S        30.0f
#define BAL_INNER_DERIVATIVE_ALPHA        0.25f

/* 底盘前馈：小车加速度(m/s^2) -> 抵消惯性力所需角度(deg)。
 * 小角度理论值约为 -atan(a/g) = -5.84 deg/(m/s^2)。若安装方向相反，
 * 只需修改该宏的正负号。当前工程无底盘模块，所以未注入时加速度为 0。
 */
#define BAL_CHASSIS_FF_GAIN_DEG_PER_MPS2 (-5.84f)
#define BAL_CHASSIS_FF_LIMIT_DEG           1.0f
#define BAL_CHASSIS_ACCEL_LPF_ALPHA        0.20f

/* 步进执行器换算与安全限制。内环输出决定 TIM3 PWM 的脉冲频率。 */
/*
 * 实测连杆机构参数（中心距）：电机摇臂 35 mm、连杆 55.5 mm、平衡杆
 * 固定转轴到驱动点 200 mm。零点时摇臂向右上 5°，连杆从下往上向右
 * 偏离竖直 8°。正步数从电机正面观察为逆时针，并抬高平衡杆。
 */
#define BAL_MOTOR_STEPS_PER_REV            6400.0f
#define BAL_CRANK_RADIUS_MM                   35.0f
#define BAL_CONNECTING_ROD_LENGTH_MM          55.5f
#define BAL_BALANCE_ROD_ARM_MM               200.0f
#define BAL_ZERO_CRANK_ANGLE_DEG                5.0f
#define BAL_ZERO_LINK_RIGHT_OF_VERTICAL_DEG     8.0f
#define BAL_POSITIVE_STEP_CRANK_SIGN            1.0f
#define BAL_STEPPER_START_RATE_STEPS_S     20.0f
#define BAL_STEPPER_STOP_RATE_STEPS_S       8.0f
#define BAL_MOTOR_COMMAND_SLEW_STEPS_S2  20000.0f
#define BAL_STEPPER_ACCEL_STEPS_S2        8000.0f
#define BAL_STEPPER_SOFT_LIMIT_STEPS       267
#define BAL_CONTROL_PERIOD_S               0.001f
#define BAL_VISION_DROPOUT_HOLD_MS           60U
#define BAL_VISION_DROPOUT_LEVEL_RATE_DEG_S  15.0f
#define BAL_VISION_TIMEOUT_MS                250U
/* 仅反转闭环输出，不改变底层 STEP/DIR/EN 定义。 */
#define BAL_MOTOR_DIRECTION_SIGN           1.0f

typedef struct {
    PID_T outer_pid;
    PID_T inner_pid;
    float ball_target_cm;
    float filtered_ball_pos_cm;
    float predicted_ball_pos_cm;
    float estimated_ball_vel_cms;
    float outer_target_angle_deg;
    float feedforward_angle_deg;
    float compensated_target_angle_deg;
    float actual_angle_deg;
    float pwm_command_deg_s;
    float chassis_speed_mps;
    float chassis_accel_mps2;
    float previous_ball_pos_cm;
    float previous_chassis_speed_mps;
    uint8_t ball_initialized;
    uint8_t chassis_speed_initialized;
} BalanceCascade_t;

void BalanceCascade_Init(BalanceCascade_t *control);
void BalanceCascade_Reset(BalanceCascade_t *control);
float BalanceCascade_UpdateOuter(BalanceCascade_t *control,
                                 float filtered_ball_pos_cm,
                                 float dt_s);
float BalanceCascade_UpdateInner(BalanceCascade_t *control,
                                 float actual_angle_deg,
                                 float dt_s);

/* 二选一接入底盘状态：已有加速度时直接调用第一个；只有速度时调用第二个。 */
void BalanceCascade_SetChassisAcceleration(BalanceCascade_t *control,
                                           float accel_mps2);
void BalanceCascade_SetChassisSpeed(BalanceCascade_t *control,
                                    float speed_mps,
                                    float dt_s);

/* 四连杆非线性换算：软件步数零点对应实物标定零点，不再把电机角度当作平衡杆角度。 */
float BalanceMechanism_StepsToRodAngleDeg(int32_t motor_steps);
float BalanceMechanism_RodRateToMotorStepRate(int32_t motor_steps,
                                               float rod_rate_deg_s);

#endif
