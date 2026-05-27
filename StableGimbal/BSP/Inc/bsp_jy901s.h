/**
  ******************************************************************************
  * @file    bsp_jy901s.h
  * @brief   JY901S 九轴 IMU 模块驱动（UART 模式）
  ******************************************************************************
  */
#ifndef __BSP_JY901S_H
#define __BSP_JY901S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* JY901S 协议帧定义 */
#define JY901S_FRAME_HEADER    0x55
#define JY901S_GYRO_TYPE       0x52    /* 角速度数据类型 */
#define JY901S_ANGLE_TYPE      0x53    /* 角度数据类型 */
#define JY901S_FRAME_LEN       11

/* 解析状态机 */
typedef enum {
    JY901S_STATE_WAIT_HEADER = 0,
    JY901S_STATE_WAIT_TYPE,
    JY901S_STATE_RECEIVING_DATA
} JY901S_State_t;

/* JY901S 数据结构 */
typedef struct {
    /* 角速度数据 (°/s) */
    float wx;       /* X轴角速度 */
    float wy;       /* Y轴角速度 */
    float wz;       /* Z轴角速度 */

    /* 角度数据 (°) */
    float pitch;    /* 俯仰角 -90~90 度 */
    float roll;     /* 横滚角 -180~180 度 */
    float yaw;      /* 航向角 -180~180 度 */

    /* 内部接收状态 */
    JY901S_State_t state;
    uint8_t  buffer[11];    /* 帧缓冲 */
    uint8_t  rx_index;      /* 接收索引 */
    uint8_t  frame_type;    /* 当前帧类型 */
    uint8_t  data_ready;    /* 数据就绪标志（新帧解析完成） */
} JY901S_t;

/**
  * @brief  初始化 JY901S 模块
  * @param  imu: JY901S 句柄
  * @param  huart: 连接 JY901S 的 UART 句柄（USART2）
  */
void JY901S_Init(JY901S_t *imu, UART_HandleTypeDef *huart);

/**
  * @brief  启动 UART 中断接收（在 main 初始化后调用一次）
  */
void JY901S_StartReceive(void);

/**
  * @brief  处理接收到的单字节（在 UART RXNE 回调中调用）
  * @param  imu: JY901S 句柄
  * @param  data: 接收到的字节
  */
void JY901S_ProcessByte(JY901S_t *imu, uint8_t data);

/**
  * @brief  检查是否有新数据就绪
  * @param  imu: JY901S 句柄
  * @retval 1=有新数据, 0=无
  */
uint8_t JY901S_IsDataReady(JY901S_t *imu);

/**
  * @brief  清除数据就绪标志
  */
void JY901S_ClearDataReady(JY901S_t *imu);

/**
  * @brief  获取 JY901S UART 句柄指针（用于中断回调）
  */
UART_HandleTypeDef* JY901S_GetUartHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_JY901S_H */
