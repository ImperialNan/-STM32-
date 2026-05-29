/**
 ******************************************************************************
 * @file    bsp_dynamixel.h
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动（AX-12A / RX-28）
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 1.0
 ******************************************************************************
 */

#ifndef BSP_DYNAMIXEL_H
#define BSP_DYNAMIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/** @defgroup DYN_Macros 协议常量
 * @{
 */
#define DYN_HEADER_1            0xFFU
#define DYN_HEADER_2            0xFFU
#define DYN_BROADCAST_ID        0xFEU
#define DYN_INST_PING           0x01U
#define DYN_INST_READ           0x02U
#define DYN_INST_WRITE          0x03U
#define DYN_INST_SYNC_WRITE     0x83U

#define DYN_ADDR_GOAL_POSITION  0x1EU   /* 2 字节, 0~1023 */
#define DYN_ADDR_MOVING_SPEED   0x20U   /* 2 字节, 0~1023 */
#define DYN_ADDR_PRESENT_POSITION 0x24U /* 2 字节, 只读 */
#define DYN_ADDR_PRESENT_SPEED  0x26U   /* 2 字节, 只读 */

#define DYN_TIMEOUT_MS          100U    /* 阻塞收发超时 */
#define DYN_STATUS_PKT_BASE     6U      /* 状态包基础长度:
                                           头(2)+ID(1)+长度(1)+错误(1)+校验(1) */
/** @} */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵机操作状态码
 */
typedef enum {
    DYN_OK = 0,
    DYN_ERR_TIMEOUT,
    DYN_ERR_CHECKSUM,
    DYN_ERR_SERVO,
    DYN_ERR_PARAM
} DynStatus;

/**
 * @brief Dynamixel 总线句柄
 * @note  绑定一个 USART 和 RS485 方向控制引脚
 */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *txEnPort;
    uint16_t            txEnPin;
} DynamixelBus;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化总线句柄
 * @param  bus: 总线句柄指针
 * @param  huart: 已初始化的 UART 句柄 (USART3)
 * @param  txEnPort: RS485 发送使能 GPIO 端口
 * @param  txEnPin:  RS485 发送使能 GPIO 引脚
 */
void dynInit(DynamixelBus *bus, UART_HandleTypeDef *huart,
             GPIO_TypeDef *txEnPort, uint16_t txEnPin);

/* ---- L1: 通用协议操作 ---- */

/**
 * @brief  从舵机寄存器读取数据
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID (1~253)
 * @param  addr: 寄存器起始地址
 * @param  dataLen: 要读取的字节数
 * @param  data: 输出缓冲区 (长度 >= dataLen)
 * @return DynStatus
 */
DynStatus dynRead(DynamixelBus *bus, uint8_t id, uint16_t addr,
                  uint8_t dataLen, uint8_t *data);

/**
 * @brief  向舵机寄存器写入数据（不等待状态回包）
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID (1~253) 或 DYN_BROADCAST_ID 广播
 * @param  addr: 寄存器起始地址
 * @param  dataLen: 要写入的字节数
 * @param  data: 写入数据缓冲区
 * @return DynStatus
 */
DynStatus dynWrite(DynamixelBus *bus, uint8_t id, uint16_t addr,
                   uint8_t dataLen, const uint8_t *data);

/* ---- L2: 语义层操作 ---- */

/**
 * @brief  读取舵机当前位置
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID
 * @param  pos: 输出, 位置值 (0~1023)
 * @return DynStatus
 */
DynStatus dynGetPosition(DynamixelBus *bus, uint8_t id, uint16_t *pos);

/**
 * @brief  读取舵机当前速度
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID
 * @param  spd: 输出, 速度值 (0~1023)
 * @return DynStatus
 */
DynStatus dynGetSpeed(DynamixelBus *bus, uint8_t id, uint16_t *spd);

/**
 * @brief  设置舵机目标位置
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID
 * @param  pos: 目标位置值 (0~1023)
 * @return DynStatus
 */
DynStatus dynSetGoalPosition(DynamixelBus *bus, uint8_t id, uint16_t pos);

/**
 * @brief  设置舵机移动速度
 * @param  bus: 总线句柄指针
 * @param  id: 舵机 ID
 * @param  spd: 移动速度值 (0~1023)
 * @return DynStatus
 */
DynStatus dynSetMovingSpeed(DynamixelBus *bus, uint8_t id, uint16_t spd);

/* ---- L3: 组合层操作 ---- */

/**
 * @brief  同步写入三个舵机的目标位置和速度 (SYNC Write)
 * @note   使用广播 ID, 一次发送同时控制三个舵机
 * @param  bus: 总线句柄指针
 * @param  id1~id3: 三个舵机的 ID
 * @param  pos1~pos3: 三个舵机的目标位置
 * @param  spd1~spd3: 三个舵机的移动速度
 * @return DynStatus
 */
DynStatus dynSyncWritePosSpeed(DynamixelBus *bus,
                                uint8_t id1, uint16_t pos1, uint16_t spd1,
                                uint8_t id2, uint16_t pos2, uint16_t spd2,
                                uint8_t id3, uint16_t pos3, uint16_t spd3);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DYNAMIXEL_H */
