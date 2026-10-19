/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "../../Src/OLED.h"        // 暂无 OLED，注释屏蔽
#include "../../Src/delay.h"
// #include "../../Src/BMP.h"          // 依赖 OLED，注释屏蔽
#include "../../Src/stepper.h"
// #include "../../Src/Key.h"          // 无按键，改为开机自动进入任务模式
// #include "../../Src/Buzzer.h"       // 暂无蜂鸣器，注释屏蔽
// #include "../../Src/Laser.h"        // 暂无激光，注释屏蔽
#include "../../Src/Tmc2209.h"
#include "../../Src/Uart_maixcam.h"
#include "../../Src/balance_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Set to 1 to run the standalone motor test loop instead of vision balancing.
   Use it after swapping motors to check wiring, direction and Vref before
   trusting the closed loop. Set back to 0 for normal operation. */
#define MOTOR_TEST_MODE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
Stepper_t balance_motor;
BalanceCascade_t balance_controller;

typedef enum {
    BALANCE_WAIT_FRAME = 0,
    BALANCE_TO_POSITIVE,
    BALANCE_TO_NEGATIVE,
    BALANCE_HOLD_NEGATIVE,
    BALANCE_HOLD_ZERO,
    BALANCE_MISSION_TIMEOUT,
    BALANCE_FAULT_STOP
} BalanceState_t;

typedef enum {
    APP_MODE_IDLE = 0,
    APP_MODE_TASK3,
    APP_MODE_TASK4
} AppMode_t;

static volatile BalanceState_t balance_state = BALANCE_WAIT_FRAME;
static volatile AppMode_t app_mode = APP_MODE_IDLE;
static uint32_t last_frame_tick;
static uint32_t last_outer_update_tick;
static uint8_t frame_initialized;
static uint8_t vision_dropout_leveling;
static float applied_motor_rate_steps_s;
static uint8_t motor_rate_active;
static uint32_t mission_start_tick;
static uint32_t final_arrival_tick;
static volatile uint32_t mission_complete_time_ms;
static volatile int32_t tuning_raw_pos_x100;
static volatile int32_t tuning_est_pos_x100;
static volatile int32_t tuning_est_vel_x100;
static volatile int32_t tuning_target_vel_x100;
static volatile int32_t tuning_vel_error_x100;
static volatile int32_t tuning_target_angle_x100;
static volatile uint8_t tuning_sample_valid;
static uint32_t key1_last_press_tick;
static uint32_t key2_last_press_tick;
static uint32_t key3_last_press_tick;
/* 运行时零点：默认等于 BAL_BALL_TARGET_CM，KEY3(PB14) 按下时改成当前球位置，
   把球放在轨道任意位置后按一下就以那里为新的"0点"（悬停目标）。 */
static float balance_center_cm = BAL_BALL_TARGET_CM;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Balance_SetMechanicalLevelZero(void)
{
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    balance_motor.state = STOP;
    balance_motor.current_pos = 0;
    balance_motor.target_pos = 0;
    balance_motor.steps_to_move = 0U;
    balance_motor.irq_call_count = 0U;
    balance_motor.accel_count = 0U;
    BalanceCascade_Reset(&balance_controller);
    __set_PRIMASK(saved_primask);
}

static void Balance_Stop(void)
{
    applied_motor_rate_steps_s = 0.0f;
    motor_rate_active = 0U;
    Stepper_Stop(&balance_motor);
}

static void BalanceMission_Reset(void)
{
    balance_state = BALANCE_WAIT_FRAME;
    vision_dropout_leveling = 0U;
    mission_start_tick = 0U;
    final_arrival_tick = 0U;
    mission_complete_time_ms = 0U;
    balance_controller.ball_target_cm = balance_center_cm;
}

static void BalanceMission_Start(uint32_t now_tick)
{
    /* 第一个有效视觉帧开始计时：0点 -> +5 cm。 */
    mission_start_tick = now_tick;
    final_arrival_tick = 0U;
    mission_complete_time_ms = 0U;
    balance_controller.ball_target_cm = balance_center_cm + BAL_SEQUENCE_OFFSET_CM;
    PID_Reset(&balance_controller.outer_pid);
    PID_Reset(&balance_controller.inner_pid);
    balance_state = BALANCE_TO_POSITIVE;
}

static void BalanceMission_Update(float actual_ball_pos_cm,
                                  float predicted_ball_pos_cm,
                                  float ball_velocity_cms,
                                  uint32_t now_tick)
{
    float error_cm;

    if (balance_state == BALANCE_TO_POSITIVE) {
        if (predicted_ball_pos_cm >=
            (balance_center_cm + BAL_SEQUENCE_OFFSET_CM - BAL_SEQUENCE_ARRIVAL_TOLERANCE_CM)) {
            /* 预测位置先到阈值就折返，让高速小球提前建立反向制动力。 */
            balance_controller.ball_target_cm = balance_center_cm - BAL_SEQUENCE_OFFSET_CM;
            PID_Reset(&balance_controller.outer_pid);
            PID_Reset(&balance_controller.inner_pid);
            final_arrival_tick = 0U;
            balance_state = BALANCE_TO_NEGATIVE;
        }
    } else if (balance_state == BALANCE_TO_NEGATIVE) {
        error_cm = (balance_center_cm - BAL_SEQUENCE_OFFSET_CM) - actual_ball_pos_cm;
        if (fabsf(error_cm) <= BAL_SEQUENCE_ARRIVAL_TOLERANCE_CM &&
            fabsf(ball_velocity_cms) <= BAL_SEQUENCE_FINAL_MAX_SPEED_CMS) {
            if (final_arrival_tick == 0U) final_arrival_tick = now_tick;
            if ((now_tick - final_arrival_tick) >= BAL_SEQUENCE_FINAL_DWELL_MS) {
                mission_complete_time_ms = now_tick - mission_start_tick;
                balance_state = BALANCE_HOLD_NEGATIVE;
            }
        } else {
            final_arrival_tick = 0U;
        }
    }

    if ((balance_state == BALANCE_TO_POSITIVE ||
         balance_state == BALANCE_TO_NEGATIVE) &&
        (now_tick - mission_start_tick) > BAL_SEQUENCE_TOTAL_LIMIT_MS) {
        /* 超时后仍以-5 cm为安全最终目标，同时保留超时状态供调试读取。 */
        balance_controller.ball_target_cm = balance_center_cm - BAL_SEQUENCE_OFFSET_CM;
        PID_Reset(&balance_controller.outer_pid);
        PID_Reset(&balance_controller.inner_pid);
        balance_state = BALANCE_MISSION_TIMEOUT;
    }
}

static void BalanceHoldZero_Start(void)
{
    balance_controller.ball_target_cm = balance_center_cm;
    PID_Reset(&balance_controller.outer_pid);
    PID_Reset(&balance_controller.inner_pid);
    balance_state = BALANCE_HOLD_ZERO;
}

static void App_SelectMode(AppMode_t new_mode)
{
    uint32_t saved_primask = __get_PRIMASK();

    /* 模式切换期间停止控制中断可见的运动状态，避免两种任务互相残留。 */
    __disable_irq();
    Balance_Stop();
    BalanceCascade_Reset(&balance_controller);
    BalanceMission_Reset();
    frame_initialized = 0U;
    last_frame_tick = 0U;
    last_outer_update_tick = 0U;
    tuning_sample_valid = 0U;
    app_mode = new_mode;

    if (new_mode == APP_MODE_IDLE) {
        /* TMC2209 EN低有效：停止模式断开驱动，确保上电和待机都不改变角度。 */
        HAL_GPIO_WritePin(Yaw_EN_GPIO_Port, Yaw_EN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Yaw_EN_GPIO_Port, Yaw_EN_Pin, GPIO_PIN_RESET);
    }
    __set_PRIMASK(saved_primask);
}

/* KEY1(PB12)/KEY2(PB13)/KEY3(PB14), active low, 80 ms debounce -- same
   behavior as the reference project: KEY1 (re)starts the +5cm/-5cm task
   sequence, KEY2 returns to the normal centered hold. KEY3 re-zeroes the
   hold target to wherever the ball currently is (put the ball anywhere on
   the rail, press KEY3, that spot becomes the new "0 point"). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_tick = HAL_GetTick();
    float new_center_cm;

    if (GPIO_Pin == KEY1_Pin &&
        (now_tick - key1_last_press_tick) >= 80U) {
        key1_last_press_tick = now_tick;
        App_SelectMode(APP_MODE_TASK3);
    } else if (GPIO_Pin == KEY2_Pin &&
               (now_tick - key2_last_press_tick) >= 80U) {
        key2_last_press_tick = now_tick;
        App_SelectMode(APP_MODE_TASK4);
    } else if (GPIO_Pin == KEY3_Pin &&
               (now_tick - key3_last_press_tick) >= 80U) {
        key3_last_press_tick = now_tick;
        /* No recent valid ball position to re-zero to -- ignore the press
           instead of silently zeroing to a stale/garbage reading. */
        if (frame_initialized == 0U) return;

        new_center_cm = balance_controller.filtered_ball_pos_cm;
        if (new_center_cm < 0.0f) new_center_cm = 0.0f;
        if (new_center_cm > BAL_ROD_LENGTH_CM) new_center_cm = BAL_ROD_LENGTH_CM;
        balance_center_cm = new_center_cm;
        App_SelectMode(APP_MODE_TASK4);
    }
}

static int32_t Balance_ClampSteps(int32_t target_steps)
{
    if (target_steps > BAL_STEPPER_SOFT_LIMIT_STEPS) {
        target_steps = BAL_STEPPER_SOFT_LIMIT_STEPS;
    }
    if (target_steps < -BAL_STEPPER_SOFT_LIMIT_STEPS) {
        target_steps = -BAL_STEPPER_SOFT_LIMIT_STEPS;
    }
    return target_steps;
}

static void Balance_ApplyPwmControl(float pwm_command_deg_s)
{
    float requested_rate_steps_s;
    float pwm_rate_steps_s;
    float abs_rate_steps_s;
    float max_rate_change_steps_s;
    float rate_change_steps_s;
    int32_t target_steps;
    uint32_t saved_primask;

    /*
     * 内环输出不是舵机占空比，而是步进电机的目标脉冲频率：
     *   deg/s * steps/deg = steps/s。
     * 以短预测窗口生成滚动位置目标，TIM3 仍保持 50% 占空比并通过频率
     * 表示控制强度，从而兼容现有 TMC2209 STEP/DIR 硬件。
     */
    requested_rate_steps_s = BalanceMechanism_RodRateToMotorStepRate(
        balance_motor.current_pos, pwm_command_deg_s);

    /*
     * 电机命令使用独立斜坡：位置环可以迅速给出较大修正倾角，但 STEP 频率
     * 不允许瞬间跳变或直接换向，从而兼顾“修正有力”和“机械动作不猛烈”。
     */
    max_rate_change_steps_s = BAL_MOTOR_COMMAND_SLEW_STEPS_S2 *
                              BAL_CONTROL_PERIOD_S;
    rate_change_steps_s = requested_rate_steps_s - applied_motor_rate_steps_s;
    if (rate_change_steps_s > max_rate_change_steps_s) {
        rate_change_steps_s = max_rate_change_steps_s;
    } else if (rate_change_steps_s < -max_rate_change_steps_s) {
        rate_change_steps_s = -max_rate_change_steps_s;
    }
    applied_motor_rate_steps_s += rate_change_steps_s;
    pwm_rate_steps_s = applied_motor_rate_steps_s;
    abs_rate_steps_s = fabsf(pwm_rate_steps_s);
    /* 启动阈值高于停止阈值，避免小命令在阈值附近每毫秒启停。 */
    if (motor_rate_active == 0U) {
        if (abs_rate_steps_s < BAL_STEPPER_START_RATE_STEPS_S) {
            if (balance_motor.state != STOP) Balance_Stop();
            return;
        }
        motor_rate_active = 1U;
    } else if (abs_rate_steps_s < BAL_STEPPER_STOP_RATE_STEPS_S) {
        Balance_Stop();
        return;
    }

    /*
     * 不再生成几十毫秒的短行程，否则位置模式每次刚起步就进入减速段。
     * 这里把运行方向的软限位作为远端目标，让梯形速度真正建立起来；内环
     * 仍按实际杆角连续减小速度，并在到达所需倾角前主动停止或反向。
     */
    target_steps = (pwm_rate_steps_s > 0.0f) ?
                   BAL_STEPPER_SOFT_LIMIT_STEPS :
                  -BAL_STEPPER_SOFT_LIMIT_STEPS;
    target_steps = Balance_ClampSteps(target_steps);

    /* TIM3 更新实时步数，短临界区确保轨迹字段不会被中断看到半更新状态。 */
    saved_primask = __get_PRIMASK();
    __disable_irq();
    if (target_steps == balance_motor.current_pos) {
        Balance_Stop();
        __set_PRIMASK(saved_primask);
        return;
    }
    Stepper_Set_Speed_Profile(&balance_motor,
                              abs_rate_steps_s,
                              BAL_STEPPER_ACCEL_STEPS_S2);
    Stepper_Goto(&balance_motor, target_steps);
    __set_PRIMASK(saved_primask);
}

void Balance_InjectChassisAcceleration(float accel_mps2)
{
    BalanceCascade_SetChassisAcceleration(&balance_controller, accel_mps2);
}

void Balance_InjectChassisSpeed(float speed_mps, float dt_s)
{
    BalanceCascade_SetChassisSpeed(&balance_controller, speed_mps, dt_s);
}

#define TUNING_RX_LINE_LEN 64U

static char tuning_rx_line[TUNING_RX_LINE_LEN];
static uint8_t tuning_rx_len;
static uint8_t tuning_stream_enabled = 1U;
static uint32_t tuning_last_tx_tick;

static void Tuning_SendText(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 20U);
}

static void Tuning_SendParam(const char *name, int32_t value)
{
    char line[48];
    int len = snprintf(line, sizeof(line), "@P,%s,%ld\n", name, (long)value);

    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 20U);
    }
}

static void Tuning_SendAllParams(void)
{
    Tuning_SendParam("OKP", (int32_t)(balance_controller.outer_pid.kp * 1000.0f));
    Tuning_SendParam("OKI", (int32_t)(balance_controller.outer_pid.ki * 1000.0f));
    Tuning_SendParam("OKD", (int32_t)(balance_controller.outer_pid.kd * 1000.0f));
    Tuning_SendParam("IKP", (int32_t)(balance_controller.inner_pid.kp * 1000.0f));
    Tuning_SendParam("IKI", (int32_t)(balance_controller.inner_pid.ki * 1000.0f));
    Tuning_SendParam("IKD", (int32_t)(balance_controller.inner_pid.kd * 1000.0f));
    Tuning_SendParam("ALIM", (int32_t)(balance_controller.outer_pid.OUT_MAX * 100.0f));
    Tuning_SendParam("PWM", (int32_t)(balance_controller.inner_pid.OUT_MAX * 100.0f));
}

static uint8_t Tuning_SetParam(const char *name, int32_t value)
{
    if (strcmp(name, "OKP") == 0 && value >= 0 && value <= 10000) {
        balance_controller.outer_pid.kp = (float)value * 0.001f;
    } else if (strcmp(name, "OKI") == 0 && value >= 0 && value <= 10000) {
        balance_controller.outer_pid.ki = (float)value * 0.001f;
    } else if (strcmp(name, "OKD") == 0 && value >= 0 && value <= 10000) {
        balance_controller.outer_pid.kd = (float)value * 0.001f;
    } else if (strcmp(name, "IKP") == 0 && value >= 0 && value <= 30000) {
        balance_controller.inner_pid.kp = (float)value * 0.001f;
    } else if (strcmp(name, "IKI") == 0 && value >= 0 && value <= 30000) {
        balance_controller.inner_pid.ki = (float)value * 0.001f;
    } else if (strcmp(name, "IKD") == 0 && value >= 0 && value <= 30000) {
        balance_controller.inner_pid.kd = (float)value * 0.001f;
    } else if (strcmp(name, "ALIM") == 0 && value >= 10 && value <= 220) {
        balance_controller.outer_pid.OUT_MAX = (float)value * 0.01f;
    } else if (strcmp(name, "PWM") == 0 && value >= 100 && value <= 10000) {
        balance_controller.inner_pid.OUT_MAX = (float)value * 0.01f;
    } else {
        return 0U;
    }
    /* 在线修改增益后清空两级积分和微分历史，避免旧状态导致瞬时冲击。 */
    PID_Reset(&balance_controller.outer_pid);
    PID_Reset(&balance_controller.inner_pid);
    return 1U;
}

static void Tuning_HandleLine(char *line)
{
    char *name;
    char *value_text;
    char *endptr;
    long value;
    char response[48];
    int len;

    if (strcmp(line, "@GET") == 0) {
        Tuning_SendAllParams();
        Tuning_SendText("@OK,GET\n");
        return;
    }
    if (strcmp(line, "@STREAM,1") == 0) {
        tuning_stream_enabled = 1U;
        Tuning_SendText("@OK,STREAM,1\n");
        return;
    }
    if (strcmp(line, "@STREAM,0") == 0) {
        tuning_stream_enabled = 0U;
        Tuning_SendText("@OK,STREAM,0\n");
        return;
    }
    if (strncmp(line, "@SET,", 5U) != 0) {
        Tuning_SendText("@ERR,CMD\n");
        return;
    }

    name = &line[5];
    value_text = strchr(name, ',');
    if (value_text == NULL) {
        Tuning_SendText("@ERR,FORMAT\n");
        return;
    }
    *value_text++ = '\0';
    value = strtol(value_text, &endptr, 10);
    if (*endptr != '\0' || Tuning_SetParam(name, (int32_t)value) == 0U) {
        Tuning_SendText("@ERR,VALUE\n");
        return;
    }

    len = snprintf(response, sizeof(response), "@OK,%s,%ld\n", name, value);
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)response, (uint16_t)len, 20U);
    }
}

static void Tuning_ProcessRx(void)
{
    uint8_t ch;

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        ch = (uint8_t)(huart2.Instance->DR & 0xFFU);
        if (ch == '\r') continue;
        if (ch == '\n') {
            tuning_rx_line[tuning_rx_len] = '\0';
            if (tuning_rx_len > 0U) Tuning_HandleLine(tuning_rx_line);
            tuning_rx_len = 0U;
        } else if (tuning_rx_len < (TUNING_RX_LINE_LEN - 1U)) {
            tuning_rx_line[tuning_rx_len++] = (char)ch;
        } else {
            tuning_rx_len = 0U;
        }
    }
}

static void Tuning_SendTelemetry(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t saved_primask;
    int32_t raw_pos;
    int32_t est_pos;
    int32_t est_vel;
    int32_t target_vel;
    int32_t vel_error;
    int32_t target_angle;
    int32_t command_angle;
    int32_t motor_pos;
    uint8_t sample_valid;
    uint8_t state;
    char line[160];
    int len;

    if (tuning_stream_enabled == 0U ||
        (now_tick - tuning_last_tx_tick) < 50U) return;
    tuning_last_tx_tick = now_tick;

    saved_primask = __get_PRIMASK();
    __disable_irq();
    raw_pos = tuning_raw_pos_x100;
    est_pos = tuning_est_pos_x100;
    est_vel = tuning_est_vel_x100;
    target_vel = tuning_target_vel_x100;
    vel_error = tuning_vel_error_x100;
    target_angle = tuning_target_angle_x100;
    command_angle = (int32_t)(balance_controller.compensated_target_angle_deg * 100.0f);
    motor_pos = balance_motor.current_pos;
    sample_valid = tuning_sample_valid;
    state = (uint8_t)balance_state;
    __set_PRIMASK(saved_primask);

    len = snprintf(line, sizeof(line),
                   "@T,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u\n",
                   (unsigned long)now_tick, (unsigned int)sample_valid,
                   (long)raw_pos, (long)est_pos, (long)est_vel,
                   (long)target_vel, (long)vel_error, (long)target_angle,
                   (long)command_angle, (long)motor_pos, (unsigned int)state);
    if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 20U);
    }
}

static void Tuning_Process(void)
{
    Tuning_ProcessRx();
    Tuning_SendTelemetry();
}

#if MOTOR_TEST_MODE
/* Standalone motor bring-up test. Moves a fixed step count forward, then
   back, forever, so wiring, Vref and direction can be checked without any
   vision input. Position mode, matching balance_motor's normal run mode --
   the old continuous speed-mode test no longer applies since the motor is
   always initialized in MODE_POSITION. The delay must exceed the move's own
   accel/run/decel time or the next call is silently dropped (state != STOP). */
static void MotorTest_Run(void)
{
    const int32_t test_steps = 800;
    const float test_speed = 400.0f;
    const float test_accel = 800.0f;

    for (;;) {
        Stepper_Set_Speed_Profile(&balance_motor, test_speed, test_accel);
        Stepper_Move(&balance_motor, test_steps);
        delay_ms(4000);

        Stepper_Set_Speed_Profile(&balance_motor, test_speed, test_accel);
        Stepper_Move(&balance_motor, -test_steps);
        delay_ms(4000);
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  /* MX_GPIO_Init() enables the driver (EN active-low) before TIM3 has
     configured PA6/STEP as its alternate function, leaving STEP floating
     while the driver is already listening -- that stray transition is what
     causes the motor to twitch once at power-on. Force EN back to disabled
     here and only re-enable once TIM3 and Stepper_Init have both settled. */
  HAL_GPIO_WritePin(Yaw_EN_GPIO_Port, Yaw_EN_Pin, GPIO_PIN_SET);

  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_UART5_Init();

  DWT_Init();
  delay_ms(1000);

  /* Motor is physically wired to the yaw channel (TIM3/PA6=STEP, PC4=DIR,
     PA7=EN), not pitch (TIM1/PE9) -- confirmed by raw PWM test. TMC2209 UART
     config is intentionally skipped, matching the confirmed-working reference
     project (relies on MS1/MS2 pin-strapped standalone mode instead). */
  Stepper_Init(&balance_motor, &htim3, TIM_CHANNEL_1,
               Yaw_DIR_GPIO_Port, Yaw_DIR_Pin, MODE_POSITION);
  BalanceCascade_Init(&balance_controller);
  BalanceMission_Reset();
  /* The rail must be physically level here. Without an angle sensor this is
     the only reference from software steps to the real mechanical angle. */
  Balance_SetMechanicalLevelZero();
  delay_ms(50);
  /* No physical key wired: start directly in the continuous hold-at-center
     task instead of waiting for a button press. */
  App_SelectMode(APP_MODE_TASK4);

#if MOTOR_TEST_MODE
  MotorTest_Run();
#endif

  VisionUart_Init();
  HAL_TIM_Base_Start_IT(&htim4);

  while (1)
  {
    /* Parser work stays outside interrupt context. */
    VisionUart_Process();
    Tuning_Process();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Buzzer_Pin|Pitch_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_SDA_Pin|Yaw_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Yaw_EN_GPIO_Port, Yaw_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pitch_DIR_Pin|OLED_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Buzzer_Pin Pitch_EN_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|Pitch_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Yaw_EN_Pin */
  GPIO_InitStruct.Pin = Yaw_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Yaw_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Yaw_DIR_Pin */
  GPIO_InitStruct.Pin = Yaw_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Yaw_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Pitch_DIR_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = Pitch_DIR_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Laser_Pin */
  GPIO_InitStruct.Pin = Laser_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Laser_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY4_Pin */
  GPIO_InitStruct.Pin = KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* KEY1_Pin(PB12) starts task 1 (drive +5cm -> hold -5cm), KEY2_Pin(PB13)
     exits back to the normal centered hold, KEY3_Pin(PB14) re-zeroes the
     hold target to wherever the ball currently is. All active low, EXTI
     falling edge, matching the reference project's PB12/PB13 behavior. */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t vision_valid;
    uint32_t now_tick;
    uint32_t outer_elapsed_ms;
    float filtered_pos_cm;
    float vision_vel_cms;
    float outer_dt_s;
    float actual_angle_deg;
    float pwm_command_deg_s;

    if (htim->Instance != TIM4) return;
    now_tick = HAL_GetTick();

    /* 上电及KEY1停止状态不运行视觉闭环，也不会产生任何STEP脉冲。 */
    if (app_mode == APP_MODE_IDLE) return;

    if (VisionUart_ConsumeFilteredPosition(&vision_valid,
                                           &filtered_pos_cm,
                                           &vision_vel_cms)) {
        if (vision_valid == 0U) {
            /* 偶发丢帧不再立刻清PID和停机，由下方分级丢帧保护处理。 */
            tuning_sample_valid = 0U;
        } else {
            if (frame_initialized == 0U) {
                if (app_mode == APP_MODE_TASK3) {
                    BalanceMission_Start(now_tick);
                } else {
                    BalanceHoldZero_Start();
                }
            }

            /* 外环仅在新视觉样本到达时更新，dt 使用真实帧间隔。 */
            outer_elapsed_ms = (last_outer_update_tick == 0U) ?
                               30U : (now_tick - last_outer_update_tick);
            if (outer_elapsed_ms < 5U) outer_elapsed_ms = 5U;
            if (outer_elapsed_ms > 100U) outer_elapsed_ms = 100U;
            outer_dt_s = (float)outer_elapsed_ms * 0.001f;
            last_outer_update_tick = now_tick;
            BalanceCascade_UpdateOuter(&balance_controller,
                                       filtered_pos_cm,
                                       outer_dt_s);
            if (frame_initialized != 0U && app_mode == APP_MODE_TASK3) {
                BalanceMission_Update(
                    filtered_pos_cm,
                    balance_controller.predicted_ball_pos_cm,
                    balance_controller.estimated_ball_vel_cms,
                    now_tick);
            }
            (void)vision_vel_cms;
            last_frame_tick = now_tick;
            frame_initialized = 1U;
            vision_dropout_leveling = 0U;
            tuning_raw_pos_x100 = (int32_t)(filtered_pos_cm * 100.0f);
        }
    }

    if (frame_initialized != 0U &&
        (now_tick - last_frame_tick) > BAL_VISION_DROPOUT_HOLD_MS) {
        float level_step_deg = BAL_VISION_DROPOUT_LEVEL_RATE_DEG_S *
                               BAL_CONTROL_PERIOD_S;

        /*
         * 先容忍短暂丢帧；超过60 ms仍未重捕获时将杆角平滑拉回0°，
         * 防止旧方向持续施力把光滑PPR管中的球推过中心。
         */
        if (vision_dropout_leveling == 0U) {
            PID_Reset(&balance_controller.outer_pid);
            vision_dropout_leveling = 1U;
        }
        if (balance_controller.outer_target_angle_deg > level_step_deg) {
            balance_controller.outer_target_angle_deg -= level_step_deg;
        } else if (balance_controller.outer_target_angle_deg < -level_step_deg) {
            balance_controller.outer_target_angle_deg += level_step_deg;
        } else {
            balance_controller.outer_target_angle_deg = 0.0f;
        }
    }

    if (frame_initialized != 0U &&
        (now_tick - last_frame_tick > BAL_VISION_TIMEOUT_MS)) {
        frame_initialized = 0U;
        tuning_sample_valid = 0U;
        BalanceCascade_Reset(&balance_controller);
        BalanceMission_Reset();
        balance_state = BALANCE_FAULT_STOP;
        Balance_Stop();
        return;
    }

    if (frame_initialized != 0U) {
        /*
         * 内环固定 1 kHz：目标角度 = 外环角度 + 底盘加速度前馈角。
         * 当前工程没有角度传感器，因此 actual_angle_deg 由 TIM3 已输出的
         * 步数换算；若机械发生失步，该估算无法感知，后续接编码器时只需
         * 在此替换实际角度来源，不需要改变两级 PID 数据流。
         */
        actual_angle_deg = BalanceMechanism_StepsToRodAngleDeg(
            balance_motor.current_pos);
        pwm_command_deg_s = BalanceCascade_UpdateInner(&balance_controller,
                                                       actual_angle_deg,
                                                       BAL_CONTROL_PERIOD_S);
        Balance_ApplyPwmControl(pwm_command_deg_s);

        tuning_est_pos_x100 =
            (int32_t)(balance_controller.predicted_ball_pos_cm * 100.0f);
        tuning_est_vel_x100 =
            (int32_t)(balance_controller.estimated_ball_vel_cms * 100.0f);
        tuning_target_vel_x100 =
            (int32_t)(balance_controller.pwm_command_deg_s * 100.0f);
        tuning_vel_error_x100 =
            (int32_t)((balance_controller.compensated_target_angle_deg -
                       balance_controller.actual_angle_deg) * 100.0f);
        tuning_target_angle_x100 =
            (int32_t)(balance_controller.outer_target_angle_deg * 100.0f);
        tuning_sample_valid = 1U;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
