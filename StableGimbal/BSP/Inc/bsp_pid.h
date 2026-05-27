/**
  ******************************************************************************
  * @file    bsp_pid.h
  * @brief   通用 PID 控制器模块
  ******************************************************************************
  */
#ifndef __BSP_PID_H
#define __BSP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* PID 参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 限幅参数 */
    float dead_zone;        /* 死区：误差在此范围内视为零 */
    float integral_limit;   /* 积分限幅 */
    float output_limit;     /* 输出限幅 */

    /* 内部状态 */
    float prev_error;
    float integral;
} PID_Controller_t;

/**
  * @brief  初始化 PID 控制器
  * @param  pid: PID 控制器句柄
  * @param  Kp, Ki, Kd: PID 参数
  */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/**
  * @brief  设置 PID 限幅参数
  */
void PID_SetLimits(PID_Controller_t *pid, float dead_zone,
                   float integral_limit, float output_limit);

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

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PID_H */
