/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   串级 PID 控制器 —— 针对三轴云台优化
  *
  *  架构:
  *    外环 (位置型 PID): 角度误差 → 角速度指令   (10Hz, 微分作用于测量值)
  *    内环 (增量式 PID): 角速度误差 → 舵机偏移量 (100Hz)
  *
  *  特性:
  *    - back-calculation 抗积分饱和
  *    - 角速度前馈 (improve disturbance rejection)
  *    - 微分低通滤波，仅作用于测量值
  *    - Ki 归一化 (dt 补偿，参数与采样率解耦)
  *    - 绝对值输出硬钳位
  *    - 饱和状态标志
  *
  *  硬件: STM32F103XB + JY901S IMU + 3× AX-12A 舵机 (RS485)
  ******************************************************************************
  */
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*  单级 PID —— 位置型（用于外环角度控制）                                     */
/* ========================================================================== */

typedef struct {
    /* —— 用户可调参数 —— */
    float kp;               /* 比例增益 */
    float kiPerSec;         /* 积分增益 (已归一化到 /s, 内部自动乘以 dt) */
    float kd;               /* 微分增益 (作用于测量值) */

    float deadZone;         /* 死区, 误差绝对值小于此值视为 0 */
    float outAbsMax;        /* 输出绝对值上限 (硬钳位) */
    float outIncMax;        /* 输出增量上限 (每步最大变化) */
    float iAbsMax;          /* 积分项绝对值上限 */

    float dFilterAlpha;     /* 微分低通滤波系数 0~1, 1=不滤波 */

    /* —— 内部状态 —— */
    float integral;         /* 积分累加器 */
    float prevMeasured;     /* 上一次测量值 (用于微分) */
    float prevDeriv;        /* 上一次微分值 (滤波用) */
    float prevOutput;       /* 上一次输出 */
    bool  saturated;        /* 输出饱和标志 */

} PidPos;


/* ========================================================================== */
/*  单级 PID —— 增量型（用于内环角速度控制）                                    */
/* ========================================================================== */

typedef struct {
    /* —— 用户可调参数 —— */
    float kp;               /* 比例增益 */
    float kiPerSec;         /* 积分增益 (归一化到 /s) */
    float kd;               /* 微分增益 */

    float deadZone;         /* 死区 */
    float outAbsMax;        /* 累计输出绝对值上限 */
    float outIncMax;        /* 增量上限 (每步最大 Δu) */

    float dFilterAlpha;     /* 微分滤波系数 */

    /* —— 内部状态 —— */
    float prevError;
    float prev2Error;
    float prevDeriv;
    float output;           /* 累计输出 */
    bool  saturated;
    bool  firstCall;        /* 首帧标志：避免误差历史全零导致的微分尖峰 */

} PidInc;


/* ========================================================================== */
/*  传感器数据                                                                 */
/* ========================================================================== */

typedef enum {
    SENSOR_SRC_IMU    = 0,
    SENSOR_SRC_AX12A  = 1,
    SENSOR_SRC_FUSION = 2
} SensorSrc;

typedef struct {
    float    imuAngle;      /* IMU 角度 (°) */
    float    imuW;          /* IMU 角速度 (°/s) */
    float    ax12aAngle;    /* 舵机反馈角度 (°) */
    float    ax12aSpeed;    /* 舵机反馈速度 (°/s) */
    uint8_t  ax12aValid;    /* 舵机数据有效 */

    float    imuWeight;     /* 融合权重 0~1 */
    SensorSrc source;       /* 数据源选择 */

    /* 以下由 sensorFusionUpdate 填充 */
    float fusedAngle;       /* 融合角度 */
    float fusedW;           /* 融合角速度 */
} SensorData;


/* ========================================================================== */
/*  串级 PID 控制器                                                            */
/* ========================================================================== */

typedef struct {
    /* 双环 */
    PidPos outer;           /* 外环: 角度 → 角速度指令 */
    PidInc inner;           /* 内环: 角速度 → 舵机偏移 */

    /* 前馈 */
    float kff;              /* 角速度前馈增益 (feedforward) */

    /* 中间变量 (可读取用于调试) */
    float    velocityCmd;   /* 外环输出 = 内环设定值 (°/s) */
    float    servoOffset;   /* 最终舵机偏移量 (°) */
    uint16_t servoSpeed;    /* 舵机运动速度 (0~1023) */
    float    speedK;        /* 速度映射增益 */
    uint16_t speedMin;
    uint16_t speedMax;

    /* 传感器 */
    SensorData sensor;

    /* 标志 */
    bool outerSat;          /* 外环饱和 */
    bool innerSat;          /* 内环饱和 */

} CascadePid;


/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/* ---- 位置型 PID ---- */
void  pidPosInit(PidPos *pid,
                 float kp, float kiPerSec, float kd,
                 float deadZone, float outAbsMax,
                 float outIncMax, float iAbsMax,
                 float dFilterAlpha);

float pidPosCompute(PidPos *pid, float setpoint, float measured, float dt);

void  pidPosReset(PidPos *pid);

/* ---- 增量型 PID ---- */
void  pidIncInit(PidInc *pid,
                 float kp, float kiPerSec, float kd,
                 float deadZone, float outAbsMax,
                 float outIncMax,
                 float dFilterAlpha);

float pidIncCompute(PidInc *pid, float setpoint, float measured, float dt);

void  pidIncReset(PidInc *pid);

/* ---- 传感器融合 ---- */
void  sensorFusionUpdate(SensorData *s);

/* ---- 串级 PID (一步完成) ---- */

/**
  * @brief 串级 PID 初始化
  * @param kff      角速度前馈增益 (典型值 0.03~0.08)
  * @param speedK   速度映射系数 (角速度误差 °/s → AX-12A speed 值)
  */
void  cascadePidInit(CascadePid *c,
                     /* 外环 */ float oKp, float oKi, float oKd,
                     float oDeadZone, float oAbsMax, float oIncMax, float oIMax,
                     float oDfAlpha,
                     /* 内环 */ float iKp, float iKi, float iKd,
                     float iDeadZone, float iAbsMax, float iIncMax,
                     float iDfAlpha,
                     /* 前馈 */ float kff,
                     /* 速度 */ float speedK, uint16_t spdMin, uint16_t spdMax);

/**
  * @brief 串级 PID 完整计算 (内外环同频调用)
  * @param dt          采样间隔 (s), 例如 0.01 = 10ms
  * @param targetAngle 目标角度 (°)
  * @retval 舵机偏移量 (°)
  */
float cascadePidCompute(CascadePid *c,
                        float targetAngle, float dt);

/**
  * @brief 仅外环 (双速率模式, 低频调用)
  * @param dtOuter     外环采样间隔 (s), 例如 0.1 = 100ms
  */
void  cascadePidComputeOuter(CascadePid *c,
                              float targetAngle, float dtOuter);

/**
  * @brief 仅内环 (双速率模式, 高频调用)
  * @param dtInner     内环采样间隔 (s), 例如 0.01 = 10ms
  * @retval 舵机偏移量 (°)
  * @note  必须先调用 cascadePidComputeOuter 更新 velocityCmd
  */
float cascadePidComputeInner(CascadePid *c, float dtInner);

/* 重置 */
void  cascadePidReset(CascadePid *c);


#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
