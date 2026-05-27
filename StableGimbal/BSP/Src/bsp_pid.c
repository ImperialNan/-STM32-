/**
  ******************************************************************************
  * @file    bsp_pid.c
  * @brief   通用 PID 控制器实现
  ******************************************************************************
  */
#include "bsp_pid.h"
#include "main.h"
#include <math.h>

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->dead_zone      = 0.0f;
    pid->integral_limit  = 500.0f;
    pid->output_limit    = 1000.0f;

    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}

void PID_SetLimits(PID_Controller_t *pid, float dead_zone,
                   float integral_limit, float output_limit)
{
    pid->dead_zone      = dead_zone;
    pid->integral_limit  = integral_limit;
    pid->output_limit    = output_limit;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    float derivative;
    float output;

    /* 死区处理 */
    if (fabsf(error) < pid->dead_zone) {
        error = 0.0f;
    }

    /* 积分累加（仅在误差较小时积分，防止积分饱和） */
    if (fabsf(error) < pid->integral_limit) {
        pid->integral += error;

        /* 积分限幅 */
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    }

    /* 微分 */
    derivative = error - pid->prev_error;

    /* PID 输出 */
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    /* 输出限幅 */
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    pid->prev_error = error;

    return output;
}
