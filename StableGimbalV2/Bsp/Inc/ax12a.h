/**
 * @file    ax12a.h
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动（AX-12A / RX-28）
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 2.1
 */

#ifndef AX12A_H
#define AX12A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------------------------------- */
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ========================= 协议常量 ================================ */

/** @defgroup AX12A_Protocol 协议常量
 * @{ */
#define AX12A_HEADER_1              0xFFU
#define AX12A_HEADER_2              0xFFU
#define AX12A_BROADCAST_ID          0xFEU

/* 指令类型 */
#define AX12A_INST_WRITE            0x03U
#define AX12A_INST_SYNC_WRITE       0x83U

/* ---- 控制表地址 ---- */
#define AX12A_ADDR_GOAL_POSITION    0x1EU   /* 2 字节, 0~1023 */
#define AX12A_ADDR_MOVING_SPEED     0x20U   /* 2 字节, 0~1023 */

/* 协议参数 */
#define AX12A_TIMEOUT_MS            100U
#define AX12A_STATUS_PKT_BASE       6U
/** @} */

/* ========================= 类型定义 ================================ */

/**
 * @brief   舵机操作状态码
 */
typedef enum {
    AX12A_OK = 0,
    AX12A_ERR_TIMEOUT,
    AX12A_ERR_CHECKSUM,
    AX12A_ERR_SERVO,
    AX12A_ERR_PARAM
} Ax12aStatus;

/**
 * @brief   Dynamixel 总线句柄
 * @note    绑定一个 USART 和 RS485 方向控制引脚
 */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *txEnPort;
    uint16_t            txEnPin;
} Ax12aBus;

/* ========================= 公开 API ================================ */

/**
 * @brief   初始化总线句柄
 * @param   bus      总线句柄指针
 * @param   huart    已初始化的 UART 句柄 (USART3)
 * @param   txEnPort RS485 发送使能 GPIO 端口
 * @param   txEnPin  RS485 发送使能 GPIO 引脚
 */
void ax12aInit(Ax12aBus *bus, UART_HandleTypeDef *huart,
               GPIO_TypeDef *txEnPort, uint16_t txEnPin);

/**
 * @brief   设置舵机目标位置
 * @param   bus 总线句柄指针
 * @param   id  舵机 ID
 * @param   pos 目标位置值 (0~1023)
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aSetGoalPosition(Ax12aBus *bus, uint8_t id,
                                 uint16_t pos);

/**
 * @brief   设置舵机移动速度
 * @param   bus 总线句柄指针
 * @param   id  舵机 ID
 * @param   spd 移动速度值 (0~1023)
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aSetMovingSpeed(Ax12aBus *bus, uint8_t id,
                                uint16_t spd);

#ifdef __cplusplus
}
#endif

#endif /* AX12A_H */
