/**
  ******************************************************************************
  * @file    pid_controller_v2.h
  * @brief   串级 PID 控制器 V2 —— 针对三轴云台优化
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
#ifndef __PID_CONTROLLER_V2_H
#define __PID_CONTROLLER_V2_H

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
    float Kp;               /* 比例增益 */
    float Ki_per_sec;       /* 积分增益 (已归一化到 /s, 内部自动乘以 dt) */
    float Kd;               /* 微分增益 (作用于测量值) */

    float dead_zone;        /* 死区, 误差绝对值小于此值视为 0 */
    float out_abs_max;      /* 输出绝对值上限 (硬钳位) */
    float out_inc_max;      /* 输出增量上限 (每步最大变化) */
    float i_abs_max;        /* 积分项绝对值上限 */

    float d_filter_alpha;   /* 微分低通滤波系数 0~1, 1=不滤波 */

    /* —— 内部状态 —— */
    float integral;         /* 积分累加器 */
    float prev_measured;    /* 上一次测量值 (用于微分) */
    float prev_deriv;       /* 上一次微分值 (滤波用) */
    float prev_output;      /* 上一次输出 */
    bool  saturated;        /* 输出饱和标志 */

} PID_Pos_t;   /* Position-type PID */


/* ========================================================================== */
/*  单级 PID —— 增量型（用于内环角速度控制）                                    */
/* ========================================================================== */

typedef struct {
    /* —— 用户可调参数 —— */
    float Kp;               /* 比例增益 */
    float Ki_per_sec;       /* 积分增益 (归一化到 /s) */
    float Kd;               /* 微分增益 */

    float dead_zone;        /* 死区 */
    float out_abs_max;      /* 累计输出绝对值上限 */
    float out_inc_max;      /* 增量上限 (每步最大 Δu) */

    float d_filter_alpha;   /* 微分滤波系数 */

    /* —— 内部状态 —— */
    float prev_error;
    float prev2_error;
    float prev_deriv;
    float output;           /* 累计输出 */
    bool  saturated;

} PID_Inc_t;   /* Incremental PID */


/* ========================================================================== */
/*  传感器数据                                                                 */
/* ========================================================================== */

typedef enum {
    SENSOR_SRC_IMU   = 0,
    SENSOR_SRC_AX12A = 1,
    SENSOR_SRC_FUSION = 2
} SensorSrc_t;

typedef struct {
    float   imu_angle;         /* IMU 角度 (°) */
    float   imu_w;             /* IMU 角速度 (°/s) */
    float   ax12a_angle;       /* 舵机反馈角度 (°) */
    float   ax12a_speed;       /* 舵机反馈速度 (°/s) */
    uint8_t ax12a_valid;       /* 舵机数据有效 */

    float       imu_weight;    /* 融合权重 0~1 */
    SensorSrc_t source;        /* 数据源选择 */

    /* 以下由 SensorFusion 填充 */
    float fused_angle;         /* 融合角度 */
    float fused_w;             /* 融合角速度 */
} SensorData_t;


/* ========================================================================== */
/*  串级 PID 控制器                                                            */
/* ========================================================================== */

typedef struct {
    /* 双环 */
    PID_Pos_t outer;           /* 外环: 角度 → 角速度指令 */
    PID_Inc_t inner;           /* 内环: 角速度 → 舵机偏移 */

    /* 前馈 */
    float Kff;                 /* 角速度前馈增益 (feedforward) */

    /* 中间变量 (可读取用于调试) */
    float velocity_cmd;        /* 外环输出 = 内环设定值 (°/s) */
    float servo_offset;        /* 最终舵机偏移量 (°) */
    uint16_t servo_speed;      /* 舵机运动速度 (0~1023) */
    float    speed_K;          /* 速度映射增益 */
    uint16_t speed_min;
    uint16_t speed_max;

    /* 传感器 */
    SensorData_t sensor;

    /* 标志 */
    bool outer_sat;            /* 外环饱和 */
    bool inner_sat;            /* 内环饱和 */

} CascadePID_v2_t;


/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/* ---- 位置型 PID ---- */
void   PID_Pos_Init(PID_Pos_t *pid,
                    float Kp, float Ki_per_sec, float Kd,
                    float dead_zone, float out_abs_max,
                    float out_inc_max, float i_abs_max,
                    float d_filter_alpha);

float  PID_Pos_Compute(PID_Pos_t *pid, float setpoint, float measured, float dt);

void   PID_Pos_Reset(PID_Pos_t *pid);

/* ---- 增量型 PID ---- */
void   PID_Inc_Init(PID_Inc_t *pid,
                    float Kp, float Ki_per_sec, float Kd,
                    float dead_zone, float out_abs_max,
                    float out_inc_max,
                    float d_filter_alpha);

float  PID_Inc_Compute(PID_Inc_t *pid, float setpoint, float measured, float dt);

void   PID_Inc_Reset(PID_Inc_t *pid);

/* ---- 传感器融合 ---- */
void   SensorFusion_Update(SensorData_t *s);

/* ---- 串级 PID (一步完成) ---- */

/**
  * @brief 串级 PID 初始化
  * @param Kff      角速度前馈增益 (典型值 0.03~0.08)
  * @param speed_K  速度映射系数 (角速度误差 °/s → AX-12A speed 值)
  */
void   CascadePID_v2_Init(CascadePID_v2_t *c,
                          /* 外环 */ float oKp, float oKi, float oKd,
                          float o_dead_zone, float o_abs_max, float o_inc_max, float o_i_max,
                          float o_df_alpha,
                          /* 内环 */ float iKp, float iKi, float iKd,
                          float i_dead_zone, float i_abs_max, float i_inc_max,
                          float i_df_alpha,
                          /* 前馈 */ float Kff,
                          /* 速度 */ float speed_K, uint16_t spd_min, uint16_t spd_max);

/**
  * @brief 串级 PID 完整计算 (内外环同频调用)
  * @param dt           采样间隔 (s), 例如 0.01 = 10ms
  * @param target_angle 目标角度 (°)
  * @retval 舵机偏移量 (°)
  */
float  CascadePID_v2_Compute(CascadePID_v2_t *c,
                              float target_angle, float dt);

/**
  * @brief 仅外环 (双速率模式, 低频调用)
  * @param dt_outer     外环采样间隔 (s), 例如 0.1 = 100ms
  */
void   CascadePID_v2_ComputeOuter(CascadePID_v2_t *c,
                                   float target_angle, float dt_outer);

/**
  * @brief 仅内环 (双速率模式, 高频调用)
  * @param dt_inner     内环采样间隔 (s), 例如 0.01 = 10ms
  * @retval 舵机偏移量 (°)
  * @note  必须先调用 ComputeOuter 更新 velocity_cmd
  */
float  CascadePID_v2_ComputeInner(CascadePID_v2_t *c, float dt_inner);

/* 重置 */
void   CascadePID_v2_Reset(CascadePID_v2_t *c);


#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_V2_H */
