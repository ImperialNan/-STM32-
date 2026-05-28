/**
  ******************************************************************************
  * @file    pid_controller_v2.c
  * @brief   串级 PID 控制器 V2 实现
  *
  *  外环 (位置型):
  *    u(k) = Kp * e(k) + Ki*dt * Σe + Kd * [meas(k-1) - meas(k)] / dt
  *    微分作用于测量值，避免设定值突变引发的微分冲击
  *
  *  内环 (增量型):
  *    Δu(k) = Kp*(e(k)-e(k-1)) + Ki*dt*e(k) + Kd/dt*(e(k)-2*e(k-1)+e(k-2))
  *    u(k)  = u(k-1) + Δu(k)
  *
  *  抗饱和:
  *    - 外环: 输出硬钳位 + 条件积分 (内环饱和时冻结外环积分)
  *    - 内环: 累计输出硬钳位 + 条件积分 (输出饱和时冻结积分)
  *
  *  前馈:
  *    servo_offset += Kff * fused_w   (IMU角速度直接补偿)
  ******************************************************************************
  */
#include "pid_controller_v2.h"
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*  内部工具函数                                                               */
/* ========================================================================== */

static inline float clamp(float v, float lim) {
    if (v >  lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

static inline float fdead(float v, float dz) {
    if (fabsf(v) < dz) return 0.0f;
    return v;
}

/* ========================================================================== */
/*  位置型 PID                                                                 */
/* ========================================================================== */

void PID_Pos_Init(PID_Pos_t *pid,
                  float Kp, float Ki_per_sec, float Kd,
                  float dead_zone, float out_abs_max,
                  float out_inc_max, float i_abs_max,
                  float d_filter_alpha)
{
    memset(pid, 0, sizeof(PID_Pos_t));

    pid->Kp         = Kp;
    pid->Ki_per_sec = Ki_per_sec;
    pid->Kd         = Kd;
    pid->dead_zone  = dead_zone;
    pid->out_abs_max = out_abs_max;
    pid->out_inc_max = out_inc_max;
    pid->i_abs_max  = i_abs_max;

    if (d_filter_alpha < 0.0f) d_filter_alpha = 0.0f;
    if (d_filter_alpha > 1.0f) d_filter_alpha = 1.0f;
    pid->d_filter_alpha = d_filter_alpha;
}

float PID_Pos_Compute(PID_Pos_t *pid, float setpoint, float measured, float dt)
{
    /* 防止 dt 异常 (首次调用或溢出) */
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;   /* fallback: 10ms */
    }

    /* 1. 误差 & 死区 */
    float error = fdead(setpoint - measured, pid->dead_zone);

    /* 2. 比例项 */
    float P = pid->Kp * error;

    /* 3. 积分项 (条件积分 + 抗饱和) */
    if (!pid->saturated) {
        pid->integral += error * dt;
        pid->integral  = clamp(pid->integral, pid->i_abs_max);
    }
    /* 饱和时不累加积分 (conditional integration) */
    float I = pid->Ki_per_sec * pid->integral;

    /* 4. 微分项 (作用于测量值, 带低通滤波)
     *    D = -Kd * d(meas)/dt ≈ -Kd * (meas(k) - meas(k-1)) / dt
     */
    float d_raw = -(measured - pid->prev_measured) / dt;  /* 负号: 微分作用于测量值 */
    float d_filt = pid->d_filter_alpha * d_raw
                 + (1.0f - pid->d_filter_alpha) * pid->prev_deriv;
    pid->prev_deriv = d_filt;
    float D = pid->Kd * d_filt;

    /* 5. 合成 & 增量限幅 */
    float raw = P + I + D;
    float inc = raw - pid->prev_output;
    inc = clamp(inc, pid->out_inc_max);
    float output = pid->prev_output + inc;

    /* 6. 绝对值硬钳位 */
    pid->saturated = (output >= pid->out_abs_max || output <= -pid->out_abs_max);
    output = clamp(output, pid->out_abs_max);

    /* 7. 更新状态 */
    pid->prev_output  = output;
    pid->prev_measured = measured;

    return output;
}

void PID_Pos_Reset(PID_Pos_t *pid)
{
    pid->integral     = 0.0f;
    pid->prev_measured = 0.0f;
    pid->prev_deriv   = 0.0f;
    pid->prev_output  = 0.0f;
    pid->saturated    = false;
}

/* ========================================================================== */
/*  增量型 PID                                                                 */
/* ========================================================================== */

void PID_Inc_Init(PID_Inc_t *pid,
                  float Kp, float Ki_per_sec, float Kd,
                  float dead_zone, float out_abs_max,
                  float out_inc_max,
                  float d_filter_alpha)
{
    memset(pid, 0, sizeof(PID_Inc_t));

    pid->Kp          = Kp;
    pid->Ki_per_sec  = Ki_per_sec;
    pid->Kd          = Kd;
    pid->dead_zone   = dead_zone;
    pid->out_abs_max = out_abs_max;
    pid->out_inc_max = out_inc_max;

    if (d_filter_alpha < 0.0f) d_filter_alpha = 0.0f;
    if (d_filter_alpha > 1.0f) d_filter_alpha = 1.0f;
    pid->d_filter_alpha = d_filter_alpha;
}

float PID_Inc_Compute(PID_Inc_t *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;
    }

    float error = fdead(setpoint - measured, pid->dead_zone);

    /* 条件积分: 输出已饱和时冻结积分，防止 windup */
    float Ki_effective = pid->saturated ? 0.0f : pid->Ki_per_sec;

    /* 增量式 PID:
     * Δu = Kp*(e(k)-e(k-1)) + Ki*dt*e(k) + Kd/dt*(e(k)-2*e(k-1)+e(k-2))
     */
    float d_raw = (error - 2.0f * pid->prev_error + pid->prev2_error) / dt;
    float d_filt = pid->d_filter_alpha * d_raw
                 + (1.0f - pid->d_filter_alpha) * pid->prev_deriv;
    pid->prev_deriv = d_filt;

    float delta = pid->Kp * (error - pid->prev_error)
                + Ki_effective * dt * error
                + pid->Kd * d_filt;

    /* 增量限幅 */
    delta = clamp(delta, pid->out_inc_max);

    /* 累加 & 绝对值钳位 */
    pid->output += delta;
    pid->saturated = (pid->output >= pid->out_abs_max
                   || pid->output <= -pid->out_abs_max);
    pid->output = clamp(pid->output, pid->out_abs_max);

    /* 更新历史误差 */
    pid->prev2_error = pid->prev_error;
    pid->prev_error  = error;

    return pid->output;
}

void PID_Inc_Reset(PID_Inc_t *pid)
{
    pid->prev_error  = 0.0f;
    pid->prev2_error = 0.0f;
    pid->prev_deriv  = 0.0f;
    pid->output      = 0.0f;
    pid->saturated   = false;
}

/* ========================================================================== */
/*  传感器融合                                                                 */
/* ========================================================================== */

void SensorFusion_Update(SensorData_t *s)
{
    switch (s->source) {

    case SENSOR_SRC_IMU:
        s->fused_angle = s->imu_angle;
        s->fused_w     = s->imu_w;
        break;

    case SENSOR_SRC_AX12A:
        if (s->ax12a_valid) {
            s->fused_angle = s->ax12a_angle;
            s->fused_w     = s->ax12a_speed;
        } else {
            /* fallback to IMU */
            s->fused_angle = s->imu_angle;
            s->fused_w     = s->imu_w;
        }
        break;

    case SENSOR_SRC_FUSION:
    default:
        if (s->ax12a_valid) {
            float w = s->imu_weight;
            s->fused_angle = w * s->imu_angle + (1.0f - w) * s->ax12a_angle;
            s->fused_w     = w * s->imu_w     + (1.0f - w) * s->ax12a_speed;
        } else {
            s->fused_angle = s->imu_angle;
            s->fused_w     = s->imu_w;
        }
        break;
    }
}

/* ========================================================================== */
/*  串级 PID                                                                   */
/* ========================================================================== */

void CascadePID_v2_Init(CascadePID_v2_t *c,
                        float oKp, float oKi, float oKd,
                        float o_dead_zone, float o_abs_max, float o_inc_max, float o_i_max,
                        float o_df_alpha,
                        float iKp, float iKi, float iKd,
                        float i_dead_zone, float i_abs_max, float i_inc_max,
                        float i_df_alpha,
                        float Kff,
                        float speed_K, uint16_t spd_min, uint16_t spd_max)
{
    memset(c, 0, sizeof(CascadePID_v2_t));

    /* 外环 (位置型) */
    PID_Pos_Init(&c->outer,
                 oKp, oKi, oKd,
                 o_dead_zone, o_abs_max, o_inc_max, o_i_max,
                 o_df_alpha);

    /* 内环 (增量型) */
    PID_Inc_Init(&c->inner,
                 iKp, iKi, iKd,
                 i_dead_zone, i_abs_max, i_inc_max,
                 i_df_alpha);

    /* 前馈 */
    c->Kff = Kff;

    /* 速度映射 */
    c->speed_K   = speed_K;
    c->speed_min = spd_min;
    c->speed_max = spd_max;
    c->servo_speed = spd_min;
}

void CascadePID_v2_Reset(CascadePID_v2_t *c)
{
    PID_Pos_Reset(&c->outer);
    PID_Inc_Reset(&c->inner);
    c->velocity_cmd = 0.0f;
    c->servo_offset = 0.0f;
    c->servo_speed  = c->speed_min;
    c->outer_sat    = false;
    c->inner_sat    = false;
    memset(&c->sensor, 0, sizeof(SensorData_t));
}

/* ── 一步完成 (内外环同频率, 简单模式) ── */

float CascadePID_v2_Compute(CascadePID_v2_t *c,
                             float target_angle, float dt)
{
    /* Step 0: 传感器融合 */
    SensorFusion_Update(&c->sensor);

    /* Step 1: 外环 (位置型 PID, 角度 → 角速度指令) */
    c->velocity_cmd = PID_Pos_Compute(&c->outer, target_angle,
                                       c->sensor.fused_angle, dt);
    c->outer_sat = c->outer.saturated;

    /* Step 2: 抗饱和 —— 内环饱和时通知外环 */
    if (c->inner_sat) {
        c->outer.saturated = true;   /* 冻结外环积分 */
    }

    /* Step 3: 内环 (增量式 PID, 角速度 → 舵机偏移) */
    c->servo_offset = PID_Inc_Compute(&c->inner, c->velocity_cmd,
                                       c->sensor.fused_w, dt);
    c->inner_sat = c->inner.saturated;

    /* Step 4: 角速度前馈 */
    c->servo_offset += c->Kff * c->sensor.fused_w;
    c->servo_offset  = clamp(c->servo_offset, c->inner.out_abs_max);

    /* Step 5: 速度计算 */
    float vel_err = fabsf(c->velocity_cmd - c->sensor.fused_w);
    float spd_f   = c->speed_K * vel_err;
    if (spd_f < (float)c->speed_min) spd_f = (float)c->speed_min;
    if (spd_f > (float)c->speed_max) spd_f = (float)c->speed_max;
    c->servo_speed = (uint16_t)spd_f;

    return c->servo_offset;
}

/* ── 双速率模式: 仅外环 (低频, 如 10Hz) ── */

void CascadePID_v2_ComputeOuter(CascadePID_v2_t *c,
                                 float target_angle, float dt_outer)
{
    /* 传感器融合 */
    SensorFusion_Update(&c->sensor);

    /* 外环 PID */
    c->velocity_cmd = PID_Pos_Compute(&c->outer, target_angle,
                                       c->sensor.fused_angle, dt_outer);
    c->outer_sat = c->outer.saturated;

    /* 抗饱和: 内环饱和 → 冻结外环积分 */
    if (c->inner_sat) {
        c->outer.saturated = true;
    }

    /* 注: velocity_cmd 在后续内环调用中保持不变 (零阶保持) */
}

/* ── 双速率模式: 仅内环 (高频, 如 100Hz) ── */

float CascadePID_v2_ComputeInner(CascadePID_v2_t *c, float dt_inner)
{
    /* 传感器融合 (更新角速度) */
    SensorFusion_Update(&c->sensor);

    /* 内环 PID */
    c->servo_offset = PID_Inc_Compute(&c->inner, c->velocity_cmd,
                                       c->sensor.fused_w, dt_inner);
    c->inner_sat = c->inner.saturated;

    /* 前馈 */
    c->servo_offset += c->Kff * c->sensor.fused_w;
    c->servo_offset  = clamp(c->servo_offset, c->inner.out_abs_max);

    /* 速度 */
    float vel_err = fabsf(c->velocity_cmd - c->sensor.fused_w);
    float spd_f   = c->speed_K * vel_err;
    if (spd_f < (float)c->speed_min) spd_f = (float)c->speed_min;
    if (spd_f > (float)c->speed_max) spd_f = (float)c->speed_max;
    c->servo_speed = (uint16_t)spd_f;

    return c->servo_offset;
}
