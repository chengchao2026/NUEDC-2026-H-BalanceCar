#include "balance_control.h"
#include <math.h>

#define BAL_PI_F 3.14159265358979323846f

static float Balance_DegToRad(float angle_deg)
{
    return angle_deg * (BAL_PI_F / 180.0f);
}

static float Balance_RadToDeg(float angle_rad)
{
    return angle_rad * (180.0f / BAL_PI_F);
}

float BalanceMechanism_StepsToRodAngleDeg(int32_t motor_steps)
{
    const float crank_r = BAL_CRANK_RADIUS_MM;
    const float link_l = BAL_CONNECTING_ROD_LENGTH_MM;
    const float rod_r = BAL_BALANCE_ROD_ARM_MM;
    const float theta_zero = Balance_DegToRad(BAL_ZERO_CRANK_ANGLE_DEG);
    const float link_zero = Balance_DegToRad(BAL_ZERO_LINK_RIGHT_OF_VERTICAL_DEG);
    float theta;
    float b0_x, b0_y, c0_x, c0_y, pivot_x, pivot_y;
    float b_x, b_y, dx, dy, center_distance;
    float along, height_sq, height;
    float base_x, base_y, perp_x, perp_y;
    float c1_x, c1_y, c2_x, c2_y;
    float err1, err2, c_x, c_y;

    /*
     * A=电机轴，B=摇臂孔，C=连杆上端，D=平衡杆固定转轴。
     * 零点由实测姿态构造出 D 相对 A 的固定坐标；任意电机角度下，C 是
     * “以 B 为圆心、55.5 mm”和“以 D 为圆心、200 mm”两圆的交点。
     */
    b0_x = crank_r * cosf(theta_zero);
    b0_y = crank_r * sinf(theta_zero);
    c0_x = b0_x + link_l * sinf(link_zero);
    c0_y = b0_y + link_l * cosf(link_zero);
    pivot_x = c0_x - rod_r;
    pivot_y = c0_y;

    theta = theta_zero + BAL_POSITIVE_STEP_CRANK_SIGN *
            ((float)motor_steps * 2.0f * BAL_PI_F / BAL_MOTOR_STEPS_PER_REV);
    b_x = crank_r * cosf(theta);
    b_y = crank_r * sinf(theta);
    dx = b_x - pivot_x;
    dy = b_y - pivot_y;
    center_distance = sqrtf(dx * dx + dy * dy);
    if (center_distance <= 0.001f) return 0.0f;

    along = (rod_r * rod_r - link_l * link_l +
             center_distance * center_distance) / (2.0f * center_distance);
    height_sq = rod_r * rod_r - along * along;
    if (height_sq < 0.0f) {
        /* 几何上无交点意味着机构参数或步数已超出物理工作域。 */
        return 0.0f;
    }
    height = sqrtf(height_sq);
    base_x = pivot_x + along * dx / center_distance;
    base_y = pivot_y + along * dy / center_distance;
    perp_x = -dy / center_distance;
    perp_y = dx / center_distance;
    c1_x = base_x + height * perp_x;
    c1_y = base_y + height * perp_y;
    c2_x = base_x - height * perp_x;
    c2_y = base_y - height * perp_y;

    /* 两圆有两个数学交点，选择靠近实物零点 C0 的装配支路。 */
    err1 = (c1_x - c0_x) * (c1_x - c0_x) +
           (c1_y - c0_y) * (c1_y - c0_y);
    err2 = (c2_x - c0_x) * (c2_x - c0_x) +
           (c2_y - c0_y) * (c2_y - c0_y);
    if (err1 <= err2) {
        c_x = c1_x;
        c_y = c1_y;
    } else {
        c_x = c2_x;
        c_y = c2_y;
    }

    return Balance_RadToDeg(atan2f(c_y - pivot_y, c_x - pivot_x));
}

float BalanceMechanism_RodRateToMotorStepRate(int32_t motor_steps,
                                               float rod_rate_deg_s)
{
    float angle_before;
    float angle_after;
    float rod_deg_per_step;

    /* 用相邻两步中心差分计算当前姿态的局部传动比，自动体现非线性。 */
    angle_before = BalanceMechanism_StepsToRodAngleDeg(motor_steps - 1);
    angle_after = BalanceMechanism_StepsToRodAngleDeg(motor_steps + 1);
    rod_deg_per_step = 0.5f * (angle_after - angle_before);
    if (fabsf(rod_deg_per_step) < 0.00001f) return 0.0f;
    return rod_rate_deg_s / rod_deg_per_step;
}

static float Balance_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void BalanceCascade_Init(BalanceCascade_t *control)
{
    if (control == 0) return;

    PID_Init(&control->outer_pid,
             BAL_OUTER_KP, BAL_OUTER_KI, BAL_OUTER_KD,
             BAL_OUTER_INTEGRAL_LIMIT,
             BAL_TARGET_ANGLE_LIMIT_DEG,
             BAL_OUTER_DERIVATIVE_ALPHA);
    PID_Init(&control->inner_pid,
             BAL_INNER_KP, BAL_INNER_KI, BAL_INNER_KD,
             BAL_INNER_INTEGRAL_LIMIT,
             BAL_PWM_OUTPUT_LIMIT_DEG_S,
             BAL_INNER_DERIVATIVE_ALPHA);
    control->ball_target_cm = BAL_BALL_TARGET_CM;
    BalanceCascade_Reset(control);
}

void BalanceCascade_Reset(BalanceCascade_t *control)
{
    if (control == 0) return;

    PID_Reset(&control->outer_pid);
    PID_Reset(&control->inner_pid);
    control->filtered_ball_pos_cm = control->ball_target_cm;
    control->predicted_ball_pos_cm = control->ball_target_cm;
    control->estimated_ball_vel_cms = 0.0f;
    control->outer_target_angle_deg = 0.0f;
    control->feedforward_angle_deg = 0.0f;
    control->compensated_target_angle_deg = 0.0f;
    control->actual_angle_deg = 0.0f;
    control->pwm_command_deg_s = 0.0f;
    control->chassis_speed_mps = 0.0f;
    control->chassis_accel_mps2 = 0.0f;
    control->previous_ball_pos_cm = control->ball_target_cm;
    control->previous_chassis_speed_mps = 0.0f;
    control->ball_initialized = 0U;
    control->chassis_speed_initialized = 0U;
}

float BalanceCascade_UpdateOuter(BalanceCascade_t *control,
                                 float filtered_ball_pos_cm,
                                 float dt_s)
{
    float pid_angle_deg;
    float requested_angle_deg;
    float max_angle_change_deg;
    float angle_change_deg;
    float prediction_lead_cm;

    if (control == 0 || dt_s <= 0.0f) return 0.0f;

    control->filtered_ball_pos_cm = filtered_ball_pos_cm;
    if (control->ball_initialized == 0U) {
        control->previous_ball_pos_cm = filtered_ball_pos_cm;
        control->estimated_ball_vel_cms = 0.0f;
        control->ball_initialized = 1U;
    } else {
        control->estimated_ball_vel_cms =
            (filtered_ball_pos_cm - control->previous_ball_pos_cm) / dt_s;
        control->previous_ball_pos_cm = filtered_ball_pos_cm;
    }

    /*
     * 光滑PPR管中小球惯性明显。用当前速度预测约120 ms后的球位，让控制器
     * 在球到达0点之前就减小原倾角并建立反向制动力。预测位移限制为±2 cm，
     * 防止视觉重捕获产生的瞬时速度尖峰导致错误大动作。
     */
    prediction_lead_cm = Balance_Clamp(
        control->estimated_ball_vel_cms * BAL_BALL_PREDICTION_HORIZON_S,
        -BAL_BALL_PREDICTION_MAX_LEAD_CM,
         BAL_BALL_PREDICTION_MAX_LEAD_CM);
    control->predicted_ball_pos_cm = Balance_Clamp(
        filtered_ball_pos_cm + prediction_lead_cm,
        0.0f,
        BAL_ROD_LENGTH_CM);

    /* 外环只关心小球位置，输出一个受限目标角度；方向宏用于匹配机械安装。 */
    pid_angle_deg = PID_Update(&control->outer_pid,
                               control->ball_target_cm,
                               control->predicted_ball_pos_cm,
                               dt_s);
    /* OUT_MAX 可由上位机在线修改，不能再用编译期默认值二次截断。 */
    requested_angle_deg = Balance_Clamp(
        BAL_OUTER_DIRECTION_SIGN * pid_angle_deg,
        -control->outer_pid.OUT_MAX,
         control->outer_pid.OUT_MAX);

    /*
     * 增大位置环比例增益可让偏离 0 点时更快产生足够倾角；这里再限制每帧
     * 目标倾角的变化量，避免视觉误差突变让步进电机瞬间猛冲或反向。
     */
    max_angle_change_deg = BAL_TARGET_ANGLE_SLEW_DEG_S * dt_s;
    angle_change_deg = requested_angle_deg - control->outer_target_angle_deg;
    angle_change_deg = Balance_Clamp(angle_change_deg,
                                     -max_angle_change_deg,
                                      max_angle_change_deg);
    control->outer_target_angle_deg += angle_change_deg;
    return control->outer_target_angle_deg;
}

float BalanceCascade_UpdateInner(BalanceCascade_t *control,
                                 float actual_angle_deg,
                                 float dt_s)
{
    if (control == 0 || dt_s <= 0.0f) return 0.0f;

    control->actual_angle_deg = actual_angle_deg;
    control->feedforward_angle_deg = Balance_Clamp(
        control->chassis_accel_mps2 * BAL_CHASSIS_FF_GAIN_DEG_PER_MPS2,
        -BAL_CHASSIS_FF_LIMIT_DEG,
         BAL_CHASSIS_FF_LIMIT_DEG);

    /*
     * 前馈角直接叠加在外环角度目标上：加速/刹车发生时无需等待小球已经
     * 偏离才动作，从而提前抵消底盘惯性力；随后仍由内环 PID 消除角度误差。
     */
    control->compensated_target_angle_deg = Balance_Clamp(
        control->outer_target_angle_deg + control->feedforward_angle_deg,
        -BAL_TOTAL_ANGLE_LIMIT_DEG,
         BAL_TOTAL_ANGLE_LIMIT_DEG);

    control->pwm_command_deg_s = BAL_MOTOR_DIRECTION_SIGN *
        PID_Update(&control->inner_pid,
                   control->compensated_target_angle_deg,
                   actual_angle_deg,
                   dt_s);
    return control->pwm_command_deg_s;
}

void BalanceCascade_SetChassisAcceleration(BalanceCascade_t *control,
                                           float accel_mps2)
{
    if (control == 0) return;

    /* 对底盘加速度也做一阶低通，避免速度差分尖峰直接冲击摆杆。 */
    control->chassis_accel_mps2 += BAL_CHASSIS_ACCEL_LPF_ALPHA *
                                   (accel_mps2 - control->chassis_accel_mps2);
}

void BalanceCascade_SetChassisSpeed(BalanceCascade_t *control,
                                    float speed_mps,
                                    float dt_s)
{
    float raw_accel_mps2;

    if (control == 0 || dt_s <= 0.0f) return;

    control->chassis_speed_mps = speed_mps;
    if (control->chassis_speed_initialized == 0U) {
        control->previous_chassis_speed_mps = speed_mps;
        control->chassis_speed_initialized = 1U;
        BalanceCascade_SetChassisAcceleration(control, 0.0f);
        return;
    }

    raw_accel_mps2 = (speed_mps - control->previous_chassis_speed_mps) / dt_s;
    control->previous_chassis_speed_mps = speed_mps;
    BalanceCascade_SetChassisAcceleration(control, raw_accel_mps2);
}
