/**
  ******************************************************************************
  * @file    jy901s.h
  * @brief   JY901S 九轴 IMU 模块驱动（DMA 循环 + IDLE 中断模式）
  ******************************************************************************
  */
#ifndef __JY901S_H
#define __JY901S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ========================== 协议帧定义 ========================== */
#define JY901S_FRAME_HEADER     0x55U
#define JY901S_GYRO_TYPE        0x52U   /* 角速度数据类型 */
#define JY901S_ANGLE_TYPE       0x53U   /* 角度数据类型 */
#define JY901S_FRAME_LEN        11U

/* DMA 环形缓冲区大小（2 的幂，便于位掩码取模） */
#define JY901S_DMA_BUF_SIZE     256U

/* ========================== 解析状态机 ========================== */
typedef enum {
    JY901S_STATE_WAIT_HEADER = 0,
    JY901S_STATE_WAIT_TYPE,
    JY901S_STATE_RECEIVING_DATA
} Jy901sState;

/* ========================== 数据结构 ========================== */
typedef struct {
    /* ---- 输出数据 ---- */
    float wx;           /* X 轴角速度 (°/s) */
    float wy;           /* Y 轴角速度 (°/s) */
    float wz;           /* Z 轴角速度 (°/s) */
    float pitch;        /* 俯仰角 (°)  -90 ~ 90 */
    float roll;         /* 横滚角 (°)  -180 ~ 180 */
    float yaw;          /* 航向角 (°)  -180 ~ 180 */

    /* ---- 接收状态（主循环独占，非 ISR） ---- */
    Jy901sState state;          /* 解析状态机当前状态 */
    uint8_t  frameBuf[11];      /* 帧组装缓冲 */
    uint8_t  frameIndex;        /* 帧内索引 */
    uint8_t  frameType;         /* 当前帧类型（0x52/0x53） */
    volatile uint8_t dataReady; /* 完整帧解析完成标志 */

    /* ---- DMA 环形缓冲（ISR 写 dmaHead / DMA 写 NDTR） ---- */
    uint8_t  dmaBuf[JY901S_DMA_BUF_SIZE];  /* DMA 循环缓冲区 */
    volatile uint16_t dmaHead;             /* 主循环读指针 */
    volatile uint16_t dmaPos;              /* ISR 记录的 DMA 写位置 */
    volatile uint8_t  idleFlag;            /* ISR 置位：IDLE 已发生 */
} Jy901s;

/* ========================== 公开 API ========================== */

/**
  * @brief  初始化 JY901S 句柄
  * @param  imu:   JY901S 句柄指针
  * @param  huart: 绑定的 UART 句柄（USART2）
  */
void jy901sInit(Jy901s *imu, UART_HandleTypeDef *huart);

/**
  * @brief  启动 DMA 循环接收 + IDLE 中断
  * @note   在 CubeMX 外设初始化完成后调用一次
  * @param  imu: JY901S 句柄指针
  */
void jy901sStartReceive(Jy901s *imu);

/**
  * @brief  USART2 IDLE 中断回调（ISR 上下文）
  * @note   仅清除 IDLE 标志 + 置 idleFlag，不做解析
  * @param  imu: JY901S 句柄指针
  */
void jy901sOnIdleIrq(Jy901s *imu);

/**
  * @brief  主循环轮询：消费环形缓冲区中的字节，驱动状态机
  * @note   必须在主循环中周期性调用
  * @param  imu: JY901S 句柄指针
  */
void jy901sPoll(Jy901s *imu);

/**
  * @brief  检查是否有新解析完成的帧
  * @param  imu: JY901S 句柄指针
  * @retval 0 = 无新数据, 非 0 = 有新数据
  */
uint8_t jy901sIsDataReady(const Jy901s *imu);

/**
  * @brief  清除数据就绪标志
  * @param  imu: JY901S 句柄指针
  */
void jy901sClearDataReady(Jy901s *imu);

#ifdef __cplusplus
}
#endif

#endif /* __JY901S_H */
