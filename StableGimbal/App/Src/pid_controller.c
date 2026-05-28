/**
  ******************************************************************************
  * @file    pid_controller.c
  * @brief   串级 PID 控制器实现
  *
  *  外环 (位置型):
  *    u(k) = kp * e(k) + kiPerSec * dt * Σe + kd * [meas(k-1) - meas(k)] / dt
  *    微分作用于测量值，避免设定值突变引发的微分冲击
  *
  *  内环 (增量型):
  *    Δu(k) = kp*(e(k)-e(k-1)) + kiPerSec*dt*e(k) + kd/dt*(e(k)-2*e(k-1)+e(k-2))
  *    u(k)  = u(k-1) + Δu(k)
  *
  *  抗饱和:
  *    - 外环: 输出硬钳位 + 条件积分 (内环饱和时冻结外环积分)
  *    - 内环: 累计输出硬钳位 + 条件积分 (输出饱和时冻结积分)
  *
  *  前馈:
  *    servoOffset += kff * fusedW   (IMU角速度直接补偿)
  ******************************************************************************
  */
#include "pid_controller.h"
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

void pidPosInit(PidPos *pid,
                float kp, float kiPerSec, float kd,
                float deadZone, float outAbsMax,
                float outIncMax, float iAbsMax,
                float dFilterAlpha)
{
    memset(pid, 0, sizeof(PidPos));

    pid->kp          = kp;
    pid->kiPerSec    = kiPerSec;
    pid->kd          = kd;
    pid->deadZone    = deadZone;
    pid->outAbsMax   = outAbsMax;
    pid->outIncMax   = outIncMax;
    pid->iAbsMax     = iAbsMax;

    if (dFilterAlpha < 0.0f) dFilterAlpha = 0.0f;
    if (dFilterAlpha > 1.0f) dFilterAlpha = 1.0f;
    pid->dFilterAlpha = dFilterAlpha;
}

float pidPosCompute(PidPos *pid, float setpoint, float measured, float dt)
{
    /* 防止 dt 异常 (首次调用或溢出) */
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;   /* fallback: 10ms */
    }

    /* 1. 误差 & 死区 */
    float error = fdead(setpoint - measured, pid->deadZone);

    /* 2. 比例项 */
    float P = pid->kp * error;

    /* 3. 积分项 (条件积分 + 抗饱和) */
    if (!pid->saturated) {
        pid->integral += error * dt;
        pid->integral  = clamp(pid->integral, pid->iAbsMax);
    }
    /* 饱和时不累加积分 (conditional integration) */
    float I = pid->kiPerSec * pid->integral;

    /* 4. 微分项 (作用于测量值, 带低通滤波)
     *    D = -kd * d(meas)/dt ≈ -kd * (meas(k) - meas(k-1)) / dt
     */
    float dRaw = -(measured - pid->prevMeasured) / dt;  /* 负号: 微分作用于测量值 */
    float dFilt = pid->dFilterAlpha * dRaw
                + (1.0f - pid->dFilterAlpha) * pid->prevDeriv;
    pid->prevDeriv = dFilt;
    float D = pid->kd * dFilt;

    /* 5. 合成 & 增量限幅 */
    float raw = P + I + D;
    float inc = raw - pid->prevOutput;
    inc = clamp(inc, pid->outIncMax);
    float output = pid->prevOutput + inc;

    /* 6. 绝对值硬钳位 */
    pid->saturated = (output >= pid->outAbsMax || output <= -pid->outAbsMax);
    output = clamp(output, pid->outAbsMax);

    /* 7. 更新状态 */
    pid->prevOutput  = output;
    pid->prevMeasured = measured;

    return output;
}

void pidPosReset(PidPos *pid)
{
    pid->integral     = 0.0f;
    pid->prevMeasured = 0.0f;
    pid->prevDeriv    = 0.0f;
    pid->prevOutput   = 0.0f;
    pid->saturated    = false;
}

/* ========================================================================== */
/*  增量型 PID                                                                 */
/* ========================================================================== */

void pidIncInit(PidInc *pid,
                float kp, float kiPerSec, float kd,
                float deadZone, float outAbsMax,
                float outIncMax,
                float dFilterAlpha)
{
    memset(pid, 0, sizeof(PidInc));

    pid->kp          = kp;
    pid->kiPerSec    = kiPerSec;
    pid->kd          = kd;
    pid->deadZone    = deadZone;
    pid->outAbsMax   = outAbsMax;
    pid->outIncMax   = outIncMax;

    if (dFilterAlpha < 0.0f) dFilterAlpha = 0.0f;
    if (dFilterAlpha > 1.0f) dFilterAlpha = 1.0f;
    pid->dFilterAlpha = dFilterAlpha;

    pid->firstCall = true;
}

float pidIncCompute(PidInc *pid, float setpoint, float measured, float dt)
{
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;
    }

    float error = fdead(setpoint - measured, pid->deadZone);

    /* 首帧：用当前误差初始化历史值，消除微分项尖峰 */
    if (pid->firstCall) {
        pid->prevError  = error;
        pid->prev2Error = error;
        pid->firstCall  = false;
    }

    /* 条件积分: 输出已饱和时冻结积分，防止 windup */
    float kiEffective = pid->saturated ? 0.0f : pid->kiPerSec;

    /* 增量式 PID:
     * Δu = kp*(e(k)-e(k-1)) + kiPerSec*dt*e(k) + kd/dt*(e(k)-2*e(k-1)+e(k-2))
     */
    float dRaw = (error - 2.0f * pid->prevError + pid->prev2Error) / dt;
    float dFilt = pid->dFilterAlpha * dRaw
                + (1.0f - pid->dFilterAlpha) * pid->prevDeriv;
    pid->prevDeriv = dFilt;

    float delta = pid->kp * (error - pid->prevError)
                + kiEffective * dt * error
                + pid->kd * dFilt;

    /* 增量限幅 */
    delta = clamp(delta, pid->outIncMax);

    /* 累加 & 绝对值钳位 */
    pid->output += delta;
    pid->saturated = (pid->output >= pid->outAbsMax
                   || pid->output <= -pid->outAbsMax);
    pid->output = clamp(pid->output, pid->outAbsMax);

    /* 更新历史误差 */
    pid->prev2Error = pid->prevError;
    pid->prevError  = error;

    return pid->output;
}

void pidIncReset(PidInc *pid)
{
    pid->prevError  = 0.0f;
    pid->prev2Error = 0.0f;
    pid->prevDeriv  = 0.0f;
    pid->output     = 0.0f;
    pid->saturated  = false;
    pid->firstCall  = true;
}

/* ========================================================================== */
/*  传感器融合                                                                 */
/* ========================================================================== */

void sensorFusionUpdate(SensorData *s)
{
    switch (s->source) {

    case SENSOR_SRC_IMU:
        s->fusedAngle = s->imuAngle;
        s->fusedW     = s->imuW;
        break;

    case SENSOR_SRC_AX12A:
        if (s->ax12aValid) {
            s->fusedAngle = s->ax12aAngle;
            s->fusedW     = s->ax12aSpeed;
        } else {
            /* fallback to IMU */
            s->fusedAngle = s->imuAngle;
            s->fusedW     = s->imuW;
        }
        break;

    case SENSOR_SRC_FUSION:
    default:
        if (s->ax12aValid) {
            float w = s->imuWeight;
            s->fusedAngle = w * s->imuAngle + (1.0f - w) * s->ax12aAngle;
            s->fusedW     = w * s->imuW     + (1.0f - w) * s->ax12aSpeed;
        } else {
            s->fusedAngle = s->imuAngle;
            s->fusedW     = s->imuW;
        }
        break;
    }
}

/* ========================================================================== */
/*  串级 PID                                                                   */
/* ========================================================================== */

void cascadePidInit(CascadePid *c,
                    float oKp, float oKi, float oKd,
                    float oDeadZone, float oAbsMax, float oIncMax, float oIMax,
                    float oDfAlpha,
                    float iKp, float iKi, float iKd,
                    float iDeadZone, float iAbsMax, float iIncMax,
                    float iDfAlpha,
                    float kff,
                    float speedK, uint16_t spdMin, uint16_t spdMax)
{
    memset(c, 0, sizeof(CascadePid));

    /* 外环 (位置型) */
    pidPosInit(&c->outer,
               oKp, oKi, oKd,
               oDeadZone, oAbsMax, oIncMax, oIMax,
               oDfAlpha);

    /* 内环 (增量型) */
    pidIncInit(&c->inner,
               iKp, iKi, iKd,
               iDeadZone, iAbsMax, iIncMax,
               iDfAlpha);

    /* 前馈 */
    c->kff = kff;

    /* 速度映射 */
    c->speedK     = speedK;
    c->speedMin   = spdMin;
    c->speedMax   = spdMax;
    c->servoSpeed = spdMin;
}

void cascadePidReset(CascadePid *c)
{
    pidPosReset(&c->outer);
    pidIncReset(&c->inner);
    c->velocityCmd = 0.0f;
    c->servoOffset = 0.0f;
    c->servoSpeed  = c->speedMin;
    c->outerSat    = false;
    c->innerSat    = false;
    memset(&c->sensor, 0, sizeof(SensorData));
}

/* ── 一步完成 (内外环同频率, 简单模式) ── */

float cascadePidCompute(CascadePid *c,
                        float targetAngle, float dt)
{
    /* Step 0: 传感器融合 */
    sensorFusionUpdate(&c->sensor);

    /* Step 1: 外环 (位置型 PID, 角度 → 角速度指令) */
    c->velocityCmd = pidPosCompute(&c->outer, targetAngle,
                                    c->sensor.fusedAngle, dt);
    c->outerSat = c->outer.saturated;

    /* Step 2: 抗饱和 —— 内环饱和时通知外环 */
    if (c->innerSat) {
        c->outer.saturated = true;   /* 冻结外环积分 */
    }

    /* Step 3: 内环 (增量式 PID, 角速度 → 舵机偏移) */
    c->servoOffset = pidIncCompute(&c->inner, c->velocityCmd,
                                    c->sensor.fusedW, dt);
    c->innerSat = c->inner.saturated;

    /* Step 4: 角速度前馈 */
    c->servoOffset += c->kff * c->sensor.fusedW;
    c->servoOffset  = clamp(c->servoOffset, c->inner.outAbsMax);

    /* Step 5: 速度计算 */
    float velErr = fabsf(c->velocityCmd - c->sensor.fusedW);
    float spdF   = c->speedK * velErr;
    if (spdF < (float)c->speedMin) spdF = (float)c->speedMin;
    if (spdF > (float)c->speedMax) spdF = (float)c->speedMax;
    c->servoSpeed = (uint16_t)spdF;

    return c->servoOffset;
}

/* ── 双速率模式: 仅外环 (低频, 如 10Hz) ── */

void cascadePidComputeOuter(CascadePid *c,
                             float targetAngle, float dtOuter)
{
    /* 传感器融合 */
    sensorFusionUpdate(&c->sensor);

    /* 外环 PID */
    c->velocityCmd = pidPosCompute(&c->outer, targetAngle,
                                    c->sensor.fusedAngle, dtOuter);
    c->outerSat = c->outer.saturated;

    /* 抗饱和: 内环饱和 → 冻结外环积分 */
    if (c->innerSat) {
        c->outer.saturated = true;
    }

    /* 注: velocityCmd 在后续内环调用中保持不变 (零阶保持) */
}

/* ── 双速率模式: 仅内环 (高频, 如 100Hz) ── */

float cascadePidComputeInner(CascadePid *c, float dtInner)
{
    /* 传感器融合 (更新角速度) */
    sensorFusionUpdate(&c->sensor);

    /* 内环 PID */
    c->servoOffset = pidIncCompute(&c->inner, c->velocityCmd,
                                    c->sensor.fusedW, dtInner);
    c->innerSat = c->inner.saturated;

    /* 前馈 */
    c->servoOffset += c->kff * c->sensor.fusedW;
    c->servoOffset  = clamp(c->servoOffset, c->inner.outAbsMax);

    /* 速度 */
    float velErr = fabsf(c->velocityCmd - c->sensor.fusedW);
    float spdF   = c->speedK * velErr;
    if (spdF < (float)c->speedMin) spdF = (float)c->speedMin;
    if (spdF > (float)c->speedMax) spdF = (float)c->speedMax;
    c->servoSpeed = (uint16_t)spdF;

    return c->servoOffset;
}
