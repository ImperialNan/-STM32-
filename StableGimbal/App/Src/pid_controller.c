/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   通用 PID 控制算法实现（增量式，支持单级和串级 PID）
  *
  *  增量式 PID 公式：
  *    Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2*e(k-1) + e(k-2)]
  *    u(k)  = u(k-1) + Δu(k)
  ******************************************************************************
  */
#include "pid_controller.h"
#include <math.h>

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

static float Clamp(float value, float limit)
{
    if (value > limit)  return limit;
    if (value < -limit) return -limit;
    return value;
}

/* ========================================================================== */
/*  单级 PID 控制器（增量式）                                                  */
/* ========================================================================== */

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->dead_zone      = 0.0f;
    pid->output_limit    = 100.0f;

    pid->d_filter_alpha = 1.0f;   /* 默认不滤波 */

    pid->prev_error  = 0.0f;
    pid->prev2_error = 0.0f;
    pid->prev_delta_d = 0.0f;
    pid->output      = 0.0f;
}

void PID_SetLimits(PID_Controller_t *pid, float dead_zone,
                   float output_limit)
{
    pid->dead_zone      = dead_zone;
    pid->output_limit   = output_limit;
}

void PID_SetOutputLimit(PID_Controller_t *pid, float output_max)
{
    /* 限制累计输出的绝对值 */
    if (pid->output > output_max) {
        pid->output = output_max;
    } else if (pid->output < -output_max) {
        pid->output = -output_max;
    }
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->prev_error  = 0.0f;
    pid->prev2_error = 0.0f;
    pid->prev_delta_d = 0.0f;
    pid->output      = 0.0f;
}

void PID_SetDerivativeFilter(PID_Controller_t *pid, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->d_filter_alpha = alpha;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    float delta;

    /* 死区处理 */
    if (fabsf(error) < pid->dead_zone) {
        error = 0.0f;
    }

    /* 增量式 PID：
     *   Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
     * 微分项经过一阶低通滤波：d_filtered = α * d_raw + (1-α) * d_prev
     */
    float d_raw = error - 2.0f * pid->prev_error + pid->prev2_error;
    float d_filtered = pid->d_filter_alpha * d_raw
                     + (1.0f - pid->d_filter_alpha) * pid->prev_delta_d;

    delta = pid->Kp * (error - pid->prev_error)
          + pid->Ki * error
          + pid->Kd * d_filtered;

    pid->prev_delta_d = d_filtered;

    /* 增量限幅 */
    if (delta > pid->output_limit) {
        delta = pid->output_limit;
    } else if (delta < -pid->output_limit) {
        delta = -pid->output_limit;
    }

    /* 累加到输出 */
    pid->output += delta;

    /* 更新历史误差 */
    pid->prev2_error = pid->prev_error;
    pid->prev_error  = error;

    return pid->output;
}


/* ========================================================================== */
/*  传感器融合                                                                 */
/* ========================================================================== */

void SensorFusion_Compute(const SensorFeedback_t *feedback,
                          float *fused_angle, float *fused_angular_vel)
{
    switch (feedback->source) {
    case SENSOR_SOURCE_IMU_ONLY:
        /* 仅使用 IMU */
        *fused_angle       = feedback->imu_angle;
        *fused_angular_vel = feedback->imu_angular_vel;
        break;

    case SENSOR_SOURCE_AX12A_ONLY:
        /* 仅使用 AX-12A */
        if (feedback->ax12a_valid) {
            *fused_angle       = feedback->ax12a_angle;
            *fused_angular_vel = feedback->ax12a_speed;
        } else {
            /* AX-12A 数据无效时回退到 IMU */
            *fused_angle       = feedback->imu_angle;
            *fused_angular_vel = feedback->imu_angular_vel;
        }
        break;

    case SENSOR_SOURCE_FUSION:
    default:
        /* IMU + AX-12A 加权融合 */
        if (feedback->ax12a_valid) {
            float w = feedback->imu_weight;
            *fused_angle       = w * feedback->imu_angle       + (1.0f - w) * feedback->ax12a_angle;
            *fused_angular_vel = w * feedback->imu_angular_vel + (1.0f - w) * feedback->ax12a_speed;
        } else {
            /* AX-12A 数据无效时仅使用 IMU */
            *fused_angle       = feedback->imu_angle;
            *fused_angular_vel = feedback->imu_angular_vel;
        }
        break;
    }
}


/* ========================================================================== */
/*  串级 PID 控制器（增量式）                                                  */
/* ========================================================================== */

void CascadePID_Init(CascadePID_t *cascade,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd)
{
    /* 初始化外环（角度环） */
    PID_Init(&cascade->outer, outer_kp, outer_ki, outer_kd);

    /* 初始化内环（角速度环） */
    PID_Init(&cascade->inner, inner_kp, inner_ki, inner_kd);

    /* 默认限幅 */
    cascade->outer_output_limit = 200.0f;   /* 最大角速度指令 200°/s */
    cascade->inner_output_limit = 50.0f;    /* 最大舵机偏移 50° */

    /* 中间量初始化 */
    cascade->velocity_cmd = 0.0f;

    /* 速度输出默认值 */
    cascade->servo_speed = 200;
    cascade->speed_gain  = 2.0f;   /* 每 1°/s 角速度误差 → 速度 +2 */
    cascade->speed_min   = 50;
    cascade->speed_max   = 1023;

    /* 融合值初始化 */
    cascade->fused_angle       = 0.0f;
    cascade->fused_angular_vel = 0.0f;
}

void CascadePID_SetLimits(CascadePID_t *cascade,
                          float outer_dead_zone, float outer_delta_limit, float outer_output_limit,
                          float inner_dead_zone, float inner_delta_limit, float inner_output_limit)
{
    /* 外环限幅 */
    PID_SetLimits(&cascade->outer, outer_dead_zone, outer_delta_limit);
    cascade->outer_output_limit = outer_output_limit;

    /* 内环限幅 */
    PID_SetLimits(&cascade->inner, inner_dead_zone, inner_delta_limit);
    cascade->inner_output_limit = inner_output_limit;
}

void CascadePID_Reset(CascadePID_t *cascade)
{
    PID_Reset(&cascade->outer);
    PID_Reset(&cascade->inner);
    cascade->velocity_cmd = 0.0f;
    cascade->servo_speed  = 200;
    cascade->fused_angle     = 0.0f;
    cascade->fused_angular_vel = 0.0f;
}

float CascadePID_Compute(CascadePID_t *cascade, float target_angle,
                          const SensorFeedback_t *feedback)
{
    /* Step 1: 传感器融合 */
    SensorFusion_Compute(feedback,
                         &cascade->fused_angle,
                         &cascade->fused_angular_vel);

    /* Step 2: 外环 PID（角度环，增量式） */
    cascade->velocity_cmd = PID_Compute(&cascade->outer, target_angle, cascade->fused_angle);

    /* 外环输出绝对值限幅 */
    cascade->velocity_cmd = Clamp(cascade->velocity_cmd, cascade->outer_output_limit);

    /* Step 3: 内环 PID（角速度环，增量式） */
    float servo_offset = PID_Compute(&cascade->inner, cascade->velocity_cmd, cascade->fused_angular_vel);

    /* 内环输出绝对值限幅 */
    servo_offset = Clamp(servo_offset, cascade->inner_output_limit);

    return servo_offset;
}

void CascadePID_ComputeOuter(CascadePID_t *cascade, float target_angle,
                              const SensorFeedback_t *feedback)
{
    /* 传感器融合 */
    SensorFusion_Compute(feedback,
                         &cascade->fused_angle,
                         &cascade->fused_angular_vel);

    /* 外环 PID（角度环）→ 角速度指令 */
    cascade->velocity_cmd = PID_Compute(&cascade->outer, target_angle, cascade->fused_angle);

    /* 外环输出绝对值限幅 */
    cascade->velocity_cmd = Clamp(cascade->velocity_cmd, cascade->outer_output_limit);
}

float CascadePID_ComputeInner(CascadePID_t *cascade,
                               const SensorFeedback_t *feedback)
{
    /* 更新融合值（内环需要最新的角速度） */
    SensorFusion_Compute(feedback,
                         &cascade->fused_angle,
                         &cascade->fused_angular_vel);

    /* 内环 PID（角速度环）：使用外环输出的 velocity_cmd 作为目标
     * 注：velocity_cmd 由外环 ComputeOuter 更新，此处保持不变（零阶保持） */
    float servo_offset = PID_Compute(&cascade->inner, cascade->velocity_cmd, cascade->fused_angular_vel);

    /* 内环输出绝对值限幅 */
    servo_offset = Clamp(servo_offset, cascade->inner_output_limit);

    /* 速度计算：speed = K * |角速度误差| */
    float vel_error = fabsf(cascade->velocity_cmd - cascade->fused_angular_vel);
    float speed_f = cascade->speed_gain * vel_error;
    if (speed_f < (float)cascade->speed_min) speed_f = (float)cascade->speed_min;
    if (speed_f > (float)cascade->speed_max) speed_f = (float)cascade->speed_max;
    cascade->servo_speed = (uint16_t)speed_f;

    return servo_offset;
}
