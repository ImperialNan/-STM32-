/**
 * @file    pid.h
 * @brief   三轴稳定云台串级 PID 控制器
 * @note    外环 (角度→目标角速度) 20Hz + 内环 (角速度→舵机输出) 100Hz
 * @version 1.1
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================= 宏定义 ================================= */

#define PID_TS              (1.0f / 100.0f)  /* 控制周期 10ms (100Hz) */
#define PID_GYRO_FC         15.0f   /* 角速度低通截止频率 15Hz (PI项) */
#define PID_ALPHA           (PID_TS / (PID_TS + 1.0f / \
                              (2.0f * 3.14159265f * PID_GYRO_FC)))
#define PID_D_FC            53.0f   /* D项巴特沃斯低通截止频率 53Hz (τ≈3ms) */
#define PID_D_ALPHA         ((2.0f * 3.14159265f * PID_D_FC * PID_TS) / \
                              (2.0f * 3.14159265f * PID_D_FC * PID_TS + 1.0f))
#define PID_POS_SCALE       (1024.0f / 300.0f)  /* 位/度 */

/* ---- 角度常量 ---- */
#define PID_ANGLE_WRAP      360.0f              /* 全圆角度 (度) */
#define PID_HALF_ANGLE      180.0f              /* 半圆角度 (度) */

/* ---- 舵机 (AX-12A) 硬件限制 ---- */
#define SERVO_PITCH_POS_MIN     0               /* Pitch舵机最小位置 (0°) */
#define SERVO_PITCH_POS_MAX     1023            /* Pitch舵机最大位置 (300°) */
#define SERVO_ROLL_POS_MIN      0               /* Roll舵机最小位置 (0°) */
#define SERVO_ROLL_POS_MAX      1023            /* Roll舵机最大位置 (300°) */
#define SERVO_YAW_POS_MIN       0               /* Yaw舵机最小位置 (0°) */
#define SERVO_YAW_POS_MAX       1023            /* Yaw舵机最大位置 (300°) */

/* ---- 外环默认参数 (cascadedPidInit) ---- */
#define PID_OUTER_INTEG_THRESHOLD   20.0f       /* 外环积分分离阈值 (度) */
#define PID_OUTER_OUTPUT_MAX        120.0f      /* 外环输出限幅 (°/s) */
#define PID_OUTER_DEADBAND          0.5f        /* 外环死区 (度) */

/* ---- 内环默认参数 (cascadedPidInit) ---- */
#define PID_INNER_INTEG_MAX         100.0f      /* 内环积分上限 */
#define PID_INNER_DEADBAND          1.0f        /* 内环死区 (°/s) */
#define PID_INNER_OUTPUT_MAX        60.0f       /* 内环输出限幅 (度) */

/* ---- 外环时序 ---- */
#define PID_OUTER_PERIOD_RATIO      5.0f        /* 外环周期 = 内环周期 × 此值 (5×10ms=50ms, 20Hz) */
#define PID_OUTER_DT                (PID_OUTER_PERIOD_RATIO * PID_TS)

/* ---- 输出增量限幅 ---- */
#define PID_OUT_DELTA_MAX           15.0f       /* 单次位置增量限幅 (步) */

/* ---- 轴编号枚举 ---- */
#define PID_AXIS_PITCH              1
#define PID_AXIS_ROLL               2
#define PID_AXIS_YAW                3

/* ---- Pitch 调试参数 ---- */
#define PID_PITCH_INNER_KP          0.016f
#define PID_PITCH_INNER_KI          0.0f
#define PID_PITCH_INNER_KD          0.0f
#define PID_PITCH_INNER_OUTPUT_MAX  60.0f
#define PID_PITCH_OUTER_KP          10.0f
#define PID_PITCH_OUTER_KI          0.1f
#define PID_PITCH_OUTER_KD          0.3f

/* ---- Roll 调试参数 ---- */
#define PID_ROLL_INNER_KP           0.016f
#define PID_ROLL_INNER_KI           0.0f
#define PID_ROLL_INNER_KD           0.0f
#define PID_ROLL_INNER_OUTPUT_MAX   60.0f
#define PID_ROLL_OUTER_KP           6.0f
#define PID_ROLL_OUTER_KI           0.1f
#define PID_ROLL_OUTER_KD           0.3f

/* ---- Yaw 调试参数 ---- */
#define PID_YAW_INNER_KP            0.015f
#define PID_YAW_INNER_KI            0.0f
#define PID_YAW_INNER_KD            0.0f
#define PID_YAW_INNER_OUTPUT_MAX    60.0f
#define PID_YAW_OUTER_KP            6.0f
#define PID_YAW_OUTER_KI            1.5f
#define PID_YAW_OUTER_KD            0.5f

/* ========================= 类型定义 ================================ */

/**
 * @brief   串级 PID 状态
 * @note    外环 (角度→目标角速度) 20Hz + 内环 (角速度→舵机输出) 100Hz
 */
typedef struct {
    /* ---- 外环 (角度→目标角速度) ---- */
    float outerKp;          /* 外环 P */
    float outerKi;          /* 外环 I */
    float outerKd;          /* 外环 D (角速度前馈) */
    float outerIntegral;    /* 外环积分累加 */
    float outerIntegThreshold; /* 外环积分分离阈值 (度) */
    float outerOutputMax;   /* 外环输出限幅 (°/s) */
    float outerDeadband;    /* 外环死区 (度) */

    /* ---- 内环 (角速度→输出) ---- */
    float innerKp;          /* 内环 P */
    float innerKi;          /* 内环 I */
    float innerKd;          /* 内环 D (角加速度阻尼) */
    float innerIntegral;    /* 内环积分累加 */
    float innerIntegMax;    /* 内环积分上限 */
    float innerDeadband;    /* 内环死区 (°/s) */
    float innerOutputMax;   /* 内环输出限幅 (度) */

    /* ---- 运行时状态 ---- */
    float filteredGyro;     /* 陀螺低通滤波值 (15Hz, PI项用) */
    float dFilteredGyro;    /* D项巴特沃斯滤波值 (53Hz, τ≈3ms) */
    float lastGyro;         /* 上一拍D项滤波值 (内环D差分用) */
    float lastOuterError;   /* 上一拍外环角度误差 (外环D用) */
    float targetAngle;      /* 目标角度 (度) */
    float targetAngularVel; /* 外环输出的目标角速度 (°/s) */
} CascadedPid;

/* ========================= 公开 API ================================ */

/**
 * @brief   初始化串级 PID (所有参数默认=0, 目标=target)
 * @param   cp     串级 PID 状态指针
 * @param   target 目标角度
 */
void cascadedPidInit(CascadedPid *cp, float target);

/**
 * @brief   初始化串级 PID 并载入调试好的参数
 * @param   cp     串级 PID 状态指针
 * @param   target 目标角度
 * @param   axis   轴编号: 1=Pitch, 2=Roll, 3=Yaw
 */
void cascadedPidInitTuned(CascadedPid *cp, float target, uint8_t axis);

/**
 * @brief   串级外环更新 (角度→目标角速度, 30Hz)
 * @param   cp           串级 PID 状态指针
 * @param   currentAngle 当前角度 (度)
 */
void cascadedPidOuterUpdate(CascadedPid *cp, float currentAngle);

/**
 * @brief   串级内环更新 (角速度→输出, 60Hz)
 * @param   cp   串级 PID 状态指针
 * @param   gyro 当前轴角速度 (°/s)
 * @return  PID 输出 (度), 正值 = 向正方向移动舵机
 */
float cascadedPidInnerUpdate(CascadedPid *cp, float gyro);

/**
 * @brief   串级PID + 舵机输出 (Pitch, 舵机1)
 * @param   cp      串级 PID 状态指针
 * @param   gyro    当前轴角速度 (°/s)
 * @param   goalPos 舵机位置缓冲区指针
 * @param   bus     AX-12A 总线句柄
 */
void pidAx12aPitchOutput(CascadedPid *cp, float gyro,
                         uint16_t *goalPos, void *bus);

/**
 * @brief   串级PID + 舵机输出 (Roll, 舵机2)
 */
void pidAx12aRollOutput(CascadedPid *cp, float gyro,
                        uint16_t *goalPos, void *bus);

/**
 * @brief   串级PID + 舵机输出 (Yaw, 舵机3)
 */
void pidAx12aYawOutput(CascadedPid *cp, float gyro,
                       uint16_t *goalPos, void *bus);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
