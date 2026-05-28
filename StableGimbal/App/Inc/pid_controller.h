/**
  ******************************************************************************
  * @file    pid_controller.h
  * @brief   通用 PID 控制算法（增量式，支持单级和串级 PID）
  *          纯软件模块，不依赖硬件外设。
  *
  *  增量式 PID 优势：
  *    - 输出为增量 Δu，不存在积分饱和问题
  *    - 切换/重启时无冲击（增量从零开始累加）
  *    - 执行器故障时自动"退饱和"
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
/*  单级 PID 控制器（增量式）                                                  */
/* ========================================================================== */

typedef struct {
    /* PID 参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 限幅参数 */
    float dead_zone;        /* 死区：误差在此范围内视为零 */
    float output_limit;     /* 输出增量限幅（每次最大变化量） */

    /* 微分滤波 */
    float d_filter_alpha;   /* 微分项低通滤波系数 (0~1)，1=不滤波 */

    /* 内部状态 */
    float prev_error;       /* 上一次误差 e(k-1) */
    float prev2_error;      /* 上上次误差 e(k-2) */
    float prev_delta_d;     /* 上一次滤波后的微分项 */
    float output;           /* 当前累计输出 */
} PID_Controller_t;

/**
  * @brief  初始化 PID 控制器
  * @param  pid: PID 控制器句柄
  * @param  Kp, Ki, Kd: PID 参数
  */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/**
  * @brief  设置 PID 限幅参数
  * @param  dead_zone: 死区（误差在此范围内视为零）
  * @param  output_limit: 输出增量限幅（每次最大变化量）
  */
void PID_SetLimits(PID_Controller_t *pid, float dead_zone,
                   float output_limit);

/**
  * @brief  设置 PID 输出绝对值限幅
  * @param  output_max: 输出绝对值上限
  */
void PID_SetOutputLimit(PID_Controller_t *pid, float output_max);

/**
  * @brief  设置微分项低通滤波系数
  * @param  alpha: 滤波系数 (0~1)，1=不滤波，0.1=强滤波。默认 1.0
  */
void PID_SetDerivativeFilter(PID_Controller_t *pid, float alpha);

/**
  * @brief  重置 PID 状态（积分、微分归零）
  */
void PID_Reset(PID_Controller_t *pid);

/**
  * @brief  计算 PID 输出
  * @param  pid: PID 控制器句柄
  * @param  setpoint: 目标值
  * @param  measured: 当前测量值
  * @retval PID 输出
  */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured);


/* ========================================================================== */
/*  串级 PID 控制器                                                            */
/*  外环: 角度控制 → 输出角速度指令                                            */
/*  内环: 角速度控制 → 输出舵机位置偏移量                                      */
/* ========================================================================== */

/**
  * @brief  传感器融合数据源选择
  */
typedef enum {
    SENSOR_SOURCE_IMU_ONLY     = 0,   /* 仅使用 IMU */
    SENSOR_SOURCE_AX12A_ONLY   = 1,   /* 仅使用 AX-12A */
    SENSOR_SOURCE_FUSION       = 2    /* IMU + AX-12A 融合 */
} SensorSource_t;

/**
  * @brief  传感器反馈数据
  */
typedef struct {
    /* IMU 数据 (JY901S) */
    float imu_angle;          /* IMU 角度 (°) */
    float imu_angular_vel;    /* IMU 角速度 (°/s) */

    /* AX-12A 舵机反馈数据 */
    float ax12a_angle;        /* AX-12A 当前角度 (°) */
    float ax12a_speed;        /* AX-12A 当前角速度 (°/s) */
    uint8_t ax12a_valid;      /* AX-12A 数据有效标志 (1=有效) */

    /* 融合参数 */
    float imu_weight;         /* IMU 权重 (0.0~1.0)，AX-12A 权重 = 1 - imu_weight */
    SensorSource_t source;    /* 数据源选择 */
} SensorFeedback_t;

/**
  * @brief  串级 PID 控制器句柄
  */
typedef struct {
    /* 外环 PID（角度环） */
    PID_Controller_t outer;   /* 外环：角度 → 角速度指令 */
    float outer_output_limit; /* 外环输出绝对值限幅（即内环最大角速度指令） */

    /* 内环 PID（角速度环） */
    PID_Controller_t inner;   /* 内环：角速度 → 舵机位置偏移 */
    float inner_output_limit; /* 内环输出绝对值限幅（即最大舵机偏移量） */

    /* 中间量（外环输出 → 内环输入） */
    float velocity_cmd;       /* 角速度指令（外环输出，内环目标） */

    /* 速度输出（由内环 ComputeInner 计算） */
    uint16_t servo_speed;     /* 舵机运动速度 (0~1023) */
    float    speed_gain;      /* 速度增益 K (°/s → AX-12A 速度单位) */
    uint16_t speed_min;       /* 最低速度 */
    uint16_t speed_max;       /* 最高速度 */

    /* 融合后的测量值（用于调试） */
    float fused_angle;        /* 融合角度 */
    float fused_angular_vel;  /* 融合角速度 */
} CascadePID_t;

/**
  * @brief  初始化串级 PID 控制器
  * @param  cascade: 串级 PID 句柄
  * @param  outer_kp, outer_ki, outer_kd: 外环 PID 参数
  * @param  inner_kp, inner_ki, inner_kd: 内环 PID 参数
  */
void CascadePID_Init(CascadePID_t *cascade,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd);

/**
  * @brief  设置串级 PID 限幅参数
  * @param  cascade: 串级 PID 句柄
  * @param  outer_dead_zone: 外环死区 (°)
  * @param  outer_delta_limit: 外环输出增量限幅 (°/s 每次)
  * @param  outer_output_limit: 外环输出绝对值限幅 (°/s)
  * @param  inner_dead_zone: 内环死区 (°/s)
  * @param  inner_delta_limit: 内环输出增量限幅 (舵机偏移° 每次)
  * @param  inner_output_limit: 内环输出绝对值限幅 (舵机偏移°)
  */
void CascadePID_SetLimits(CascadePID_t *cascade,
                          float outer_dead_zone, float outer_delta_limit, float outer_output_limit,
                          float inner_dead_zone, float inner_delta_limit, float inner_output_limit);

/**
  * @brief  重置串级 PID 状态
  */
void CascadePID_Reset(CascadePID_t *cascade);

/**
  * @brief  计算串级 PID 输出（一步完成，外环+内环同频率）
  * @param  cascade: 串级 PID 句柄
  * @param  target_angle: 目标角度 (°)
  * @param  feedback: 传感器反馈数据
  * @retval 舵机位置偏移量
  */
float CascadePID_Compute(CascadePID_t *cascade, float target_angle,
                          const SensorFeedback_t *feedback);

/**
  * @brief  仅执行外环 PID（角度环），更新融合值和角速度指令
  * @note   用于双速率控制：外环低频（如 10Hz）
  * @param  cascade: 串级 PID 句柄
  * @param  target_angle: 目标角度 (°)
  * @param  feedback: 传感器反馈数据
  */
void CascadePID_ComputeOuter(CascadePID_t *cascade, float target_angle,
                              const SensorFeedback_t *feedback);

/**
  * @brief  仅执行内环 PID（角速度环），使用外环输出的角速度指令
  * @note   用于双速率控制：内环高频（如 100Hz）
  * @param  cascade: 串级 PID 句柄
  * @param  feedback: 传感器反馈数据
  * @retval 舵机位置偏移量
  */
float CascadePID_ComputeInner(CascadePID_t *cascade,
                               const SensorFeedback_t *feedback);

/**
  * @brief  传感器融合计算
  * @param  feedback: 传感器反馈数据
  * @param  fused_angle: 输出融合角度
  * @param  fused_angular_vel: 输出融合角速度
  */
void SensorFusion_Compute(const SensorFeedback_t *feedback,
                          float *fused_angle, float *fused_angular_vel);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
