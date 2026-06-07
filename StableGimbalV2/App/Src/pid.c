/**
 * @file    pid.c
 * @brief   三轴稳定云台串级 PID 控制器实现
 * @note    外环 (角度→目标角速度) 20Hz + 内环 (角速度→舵机输出) 100Hz
 * @version 1.1
 */

#include "pid.h"
#include "ax12a.h"

/* ========================= 辅助函数 ================================ */

/**
 * @brief   最短角度差 (内部辅助)
 */
static float pidShortestAngleDiff(float target, float current)
{
    float diff = target - current;

    while (diff > PID_HALF_ANGLE) {
        diff -= PID_ANGLE_WRAP;
    }
    while (diff < -PID_HALF_ANGLE) {
        diff += PID_ANGLE_WRAP;
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
    cp->outerIntegThreshold = PID_OUTER_INTEG_THRESHOLD;
    cp->outerOutputMax = PID_OUTER_OUTPUT_MAX;
    cp->outerDeadband  = PID_OUTER_DEADBAND;
    /* 内环全零 */
    cp->innerKp        = 0.0f;
    cp->innerKi        = 0.0f;
    cp->innerKd        = 0.0f;
    cp->innerIntegral  = 0.0f;
    cp->innerIntegMax  = PID_INNER_INTEG_MAX;
    cp->innerDeadband  = PID_INNER_DEADBAND;
    cp->innerOutputMax = PID_INNER_OUTPUT_MAX;
    /* 运行时 */
    cp->filteredGyro     = 0.0f;
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
    /* 外环每 5 拍调用一次 (20Hz), dt=5*PID_TS=50ms */
    if (angleError > -cp->outerIntegThreshold &&
        angleError <  cp->outerIntegThreshold) {
        cp->outerIntegral += angleError * PID_OUTER_DT;
    }

    cp->targetAngularVel += cp->outerKi * cp->outerIntegral;

    /* 外环微分 (误差变化率, dt=5*PID_TS=50ms) */
    {
        float dt = PID_OUTER_DT;
        cp->targetAngularVel += cp->outerKd *
            (angleError - cp->lastOuterError) / dt;
        cp->lastOuterError = angleError;
    }

    /* 外环输出限幅 */
    if (cp->targetAngularVel >  cp->outerOutputMax) {
        cp->targetAngularVel =  cp->outerOutputMax;
    }
    if (cp->targetAngularVel < -cp->outerOutputMax) {
        cp->targetAngularVel = -cp->outerOutputMax;
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

    /* ---- 陀螺低通滤波 ---- */
    cp->filteredGyro += PID_ALPHA * (gyro - cp->filteredGyro);

    /* ---- 角速度误差 ---- */
    velError = cp->targetAngularVel - cp->filteredGyro;

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

    if (axis == PID_AXIS_PITCH) {
        /* Pitch */
        cp->innerKp        = PID_PITCH_INNER_KP;
        cp->innerKi        = PID_PITCH_INNER_KI;
        cp->innerKd        = PID_PITCH_INNER_KD;
        cp->innerOutputMax = PID_PITCH_INNER_OUTPUT_MAX;
        cp->outerKp        = PID_PITCH_OUTER_KP;
        cp->outerKi        = PID_PITCH_OUTER_KI;
        cp->outerKd        = PID_PITCH_OUTER_KD;
    } else if (axis == PID_AXIS_ROLL) {
        /* Roll */
        cp->innerKp        = PID_ROLL_INNER_KP;
        cp->innerKi        = PID_ROLL_INNER_KI;
        cp->innerKd        = PID_ROLL_INNER_KD;
        cp->innerOutputMax = PID_ROLL_INNER_OUTPUT_MAX;
        cp->outerKp        = PID_ROLL_OUTER_KP;
        cp->outerKi        = PID_ROLL_OUTER_KI;
        cp->outerKd        = PID_ROLL_OUTER_KD;
    } else if (axis == PID_AXIS_YAW) {
        /* Yaw */
        cp->innerKp        = PID_YAW_INNER_KP;
        cp->innerKi        = PID_YAW_INNER_KI;
        cp->innerKd        = PID_YAW_INNER_KD;
        cp->innerOutputMax = PID_YAW_INNER_OUTPUT_MAX;
        cp->outerKp        = PID_YAW_OUTER_KP;
        cp->outerKi        = PID_YAW_OUTER_KI;
        cp->outerKd        = PID_YAW_OUTER_KD;
    }
}

/* ======================= 舵机输出封装 ============================== */

void pidAx12aPitchOutput(CascadedPid *cp, float gyro,
                         uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  PID_OUT_DELTA_MAX) { deltaPos =  PID_OUT_DELTA_MAX; }
    if (deltaPos < -PID_OUT_DELTA_MAX) { deltaPos = -PID_OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[0] + (int32_t)deltaPos;
    if (newPos < SERVO_PITCH_POS_MIN) { newPos = SERVO_PITCH_POS_MIN; }
    if (newPos > SERVO_PITCH_POS_MAX) { newPos = SERVO_PITCH_POS_MAX; }
    goalPos[0] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 1, goalPos[0]);
}

void pidAx12aRollOutput(CascadedPid *cp, float gyro,
                        uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = -cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  PID_OUT_DELTA_MAX) { deltaPos =  PID_OUT_DELTA_MAX; }
    if (deltaPos < -PID_OUT_DELTA_MAX) { deltaPos = -PID_OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[1] + (int32_t)deltaPos;
    if (newPos < SERVO_ROLL_POS_MIN) { newPos = SERVO_ROLL_POS_MIN; }
    if (newPos > SERVO_ROLL_POS_MAX) { newPos = SERVO_ROLL_POS_MAX; }
    goalPos[1] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 2, goalPos[1]);
}

void pidAx12aYawOutput(CascadedPid *cp, float gyro,
                       uint16_t *goalPos, void *bus)
{
    Ax12aBus *axBus = (Ax12aBus *)bus;

    float pidOut   = cascadedPidInnerUpdate(cp, gyro);
    float deltaPos = pidOut * PID_POS_SCALE;

    if (deltaPos >  PID_OUT_DELTA_MAX) { deltaPos =  PID_OUT_DELTA_MAX; }
    if (deltaPos < -PID_OUT_DELTA_MAX) { deltaPos = -PID_OUT_DELTA_MAX; }

    int32_t newPos = (int32_t)goalPos[2] + (int32_t)deltaPos;
    if (newPos < SERVO_YAW_POS_MIN) { newPos = SERVO_YAW_POS_MIN; }
    if (newPos > SERVO_YAW_POS_MAX) { newPos = SERVO_YAW_POS_MAX; }
    goalPos[2] = (uint16_t)newPos;
    ax12aSetGoalPosition(axBus, 3, goalPos[2]);
}
