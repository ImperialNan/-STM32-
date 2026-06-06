/**
 * @file    pid.c
 * @brief   三轴稳定云台 PID 控制器实现
 * @note    单级 PID + 测量微分 + 一阶低通滤波 + 积分分离
 * @version 1.0
 */

#include "pid.h"
#include "ax12a.h"
#include <string.h>

/* ========================= 公开 API 实现 =========================== */

/**
 * @brief   最短角度差
 */
float pidShortestAngleDiff(float target, float current)
{
    float diff = target - current;

    while (diff > 180.0f) {
        diff -= 360.0f;
    }
    while (diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

/* ======================= 串级 PID 实现 ============================ */

/**
 * @brief   初始化串级 PID
 */
void cascadedPidInit(CascadedPid *cp, float target)
{
    /* 外环全零 */
    cp->outerKp        = 0.0f;
    cp->outerKi        = 0.0f;
    cp->outerKd        = 0.0f;
    cp->outerIntegral  = 0.0f;
    cp->outerIntegThreshold = 20.0f;
    cp->outerOutputMax = OUTER_OUTPUT_MAX;
    cp->outerDeadband  = 0.5f;
    /* 内环全零 */
    cp->innerKp        = 0.0f;
    cp->innerKi        = 0.0f;
    cp->innerKd        = 0.0f;
    cp->innerIntegral  = 0.0f;
    cp->innerIntegMax  = 100.0f;
    cp->innerDeadband  = 1.0f;
    cp->innerOutputMax = 60.0f;
    /* 运行时 */
    cp->dFilteredGyro    = 0.0f;
    cp->lastGyro         = 0.0f;
    cp->lastOuterError   = 0.0f;
    cp->targetAngle      = target;
    cp->targetAngularVel = 0.0f;
}

/**
 * @brief   串级外环更新 (角度→目标角速度, 20Hz)
 * @note    从角度误差计算目标角速度, 100Hz 内环使用
 */
void cascadedPidOuterUpdate(CascadedPid *cp, float currentAngle)
{
    float angleError;

    angleError = pidShortestAngleDiff(cp->targetAngle,
                                      currentAngle);

    /* 死区 */
    if (angleError > -cp->outerDeadband &&
        angleError <  cp->outerDeadband) {
        angleError = 0.0f;
    }

    /* 外环 PID: targetAngularVel = Kp*e + Ki*∫e + Kd*(de/dt) */
    cp->targetAngularVel  = cp->outerKp * angleError;

    /* 外环积分 (仅当误差 < 积分分离阈值时累加) */
    if (angleError > -cp->outerIntegThreshold &&
        angleError <  cp->outerIntegThreshold) {
        cp->outerIntegral += angleError * PID_TS;
    }

    cp->targetAngularVel += cp->outerKi * cp->outerIntegral;

    /* 外环微分 (误差变化率, dt=5*PID_TS=50ms) */
    {
        float dt = 5.0f * PID_TS;
        float dError = angleError - cp->lastOuterError;

        /* 角度跳变保护: 检测 ±180° 死点附近的突变 */
        if (dError > 180.0f || dError < -180.0f) {
            dError = 0.0f;  /* 跳过本次 D 项 */
        }

        cp->targetAngularVel += cp->outerKd * dError / dt;
        cp->lastOuterError = angleError;
    }

    /* 外环输出限幅 */
    if (cp->targetAngularVel >  OUTER_OUTPUT_MAX) {
        cp->targetAngularVel =  OUTER_OUTPUT_MAX;
    }
    if (cp->targetAngularVel < -OUTER_OUTPUT_MAX) {
        cp->targetAngularVel = -OUTER_OUTPUT_MAX;
    }
}

/**
 * @brief   串级内环更新 (角速度→输出, 100Hz)
 * @note    陀螺低通滤波 + 角速度误差 PI 控制
 *          PID_TS = 1/100 = 0.01s (100Hz)
 */
float cascadedPidInnerUpdate(CascadedPid *cp, float gyro)
{
    float velError;
    float output;

    /* ---- 角速度误差 (直接使用原始陀螺值) ---- */
    velError = cp->targetAngularVel - gyro;

    /* 内环死区 */
    if (velError > -cp->innerDeadband &&
        velError <  cp->innerDeadband) {
        velError = 0.0f;
    }

    /* ---- 内环 PID ---- */
    /* P */
    output = cp->innerKp * velError;

    /* I */
    cp->innerIntegral += velError * PID_TS;
    if (cp->innerIntegral >  cp->innerIntegMax) {
        cp->innerIntegral =  cp->innerIntegMax;
    }
    if (cp->innerIntegral < -cp->innerIntegMax) {
        cp->innerIntegral = -cp->innerIntegMax;
    }
    output += cp->innerKi * cp->innerIntegral;

    /* D (角加速度阻尼, 53Hz 巴特沃斯低通) */
    cp->dFilteredGyro += PID_D_ALPHA * (gyro - cp->dFilteredGyro);
    output += cp->innerKd * (cp->dFilteredGyro - cp->lastGyro) / PID_TS;
    cp->lastGyro = cp->dFilteredGyro;

    /* 内环输出限幅 */
    if (output >  cp->innerOutputMax) { output =  cp->innerOutputMax; }
    if (output < -cp->innerOutputMax) { output = -cp->innerOutputMax; }

    return output;
}

/* ======================= 调优参数加载 ============================= */

/**
 * @brief   初始化串级 PID 并载入调试好的参数
 * @param   axis 1=Pitch, 2=Roll, 3=Yaw
 */
void cascadedPidInitTuned(CascadedPid *cp, float target, uint8_t axis)
{
    cascadedPidInit(cp, target);

    if (axis == 1) {
        /* Pitch */
        cp->innerKp        = 0.03f;
        cp->innerKi        = 0.0f;
        cp->innerKd        = 0.0f;
        cp->innerOutputMax = 60.0f;
        cp->outerKp        = 3.0f;
        cp->outerKi        = 0.0f;
        cp->outerKd        = 0.0f;
    } else if (axis == 2) {
        /* Roll */
        cp->innerKp        = 0.018f;
        cp->innerKi        = 0.0f;
        cp->innerKd        = 0.00f;
        cp->innerOutputMax = 60.0f;
        cp->outerKp        = 9.0f;
        cp->outerKi        = 0.0f;
        cp->outerKd        = 0.3f;
    } else {
        /* Yaw */
        cp->innerKp        = 0.018f;
        cp->innerKi        = 0.0f;
        cp->innerKd        = 0.0f;
        cp->innerOutputMax = 60.0f;
        cp->outerKp        = 9.0f;
        cp->outerKi        = 0.0f;
        cp->outerKd        = 0.3f;
    }
}

/* ======================= 舵机输出封装 ============================== */

void pidAx12aPitchOutput(CascadedPid *cp, float gyro,
                         uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = -cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  OUT_DELTA_MAX) { deltaPos =  OUT_DELTA_MAX; }
    if (deltaPos < -OUT_DELTA_MAX) { deltaPos = -OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[0] + (int32_t)deltaPos;
    if (newPos < PITCH_POS_MIN) { newPos = PITCH_POS_MIN; }
    if (newPos > PITCH_POS_MAX) { newPos = PITCH_POS_MAX; }
    goalPos[0] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 1, goalPos[0]);
}

void pidAx12aRollOutput(CascadedPid *cp, float gyro,
                        uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = -cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  OUT_DELTA_MAX) { deltaPos =  OUT_DELTA_MAX; }
    if (deltaPos < -OUT_DELTA_MAX) { deltaPos = -OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[1] + (int32_t)deltaPos;
    if (newPos < ROLL_POS_MIN) { newPos = ROLL_POS_MIN; }
    if (newPos > ROLL_POS_MAX) { newPos = ROLL_POS_MAX; }
    goalPos[1] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 2, goalPos[1]);
}

void pidAx12aYawOutput(CascadedPid *cp, float gyro,
                       uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  OUT_DELTA_MAX) { deltaPos =  OUT_DELTA_MAX; }
    if (deltaPos < -OUT_DELTA_MAX) { deltaPos = -OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[2] + (int32_t)deltaPos;
    if (newPos < YAW_POS_MIN) { newPos = YAW_POS_MIN; }
    if (newPos > YAW_POS_MAX) { newPos = YAW_POS_MAX; }
    goalPos[2] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 3, goalPos[2]);
}
