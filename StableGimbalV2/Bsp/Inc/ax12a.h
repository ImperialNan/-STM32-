/**
 * @file    ax12a.h
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动（AX-12A / RX-28）
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 2.0
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
 * @{
 */
#define AX12A_HEADER_1              0xFFU
#define AX12A_HEADER_2              0xFFU
#define AX12A_BROADCAST_ID          0xFEU

/* 指令类型 */
#define AX12A_INST_PING             0x01U
#define AX12A_INST_READ             0x02U
#define AX12A_INST_WRITE            0x03U
#define AX12A_INST_SYNC_WRITE       0x83U

/* ---- 控制表地址 ---- */
#define AX12A_ADDR_CW_ANGLE_LIMIT   0x06U   /* 2 字节 */
#define AX12A_ADDR_CCW_ANGLE_LIMIT  0x08U   /* 2 字节 */
#define AX12A_ADDR_MAX_TORQUE       0x0EU   /* 2 字节 */
#define AX12A_ADDR_TORQUE_ENABLE    0x18U   /* 1 字节 */
#define AX12A_ADDR_GOAL_POSITION    0x1EU   /* 2 字节, 0~1023 */
#define AX12A_ADDR_MOVING_SPEED     0x20U   /* 2 字节, 0~1023 */
#define AX12A_ADDR_TORQUE_LIMIT     0x22U   /* 2 字节, 0~1023 */
#define AX12A_ADDR_PRESENT_POSITION 0x24U   /* 2 字节, 只读 */
#define AX12A_ADDR_PRESENT_SPEED    0x26U   /* 2 字节, 只读 */
#define AX12A_ADDR_PRESENT_LOAD     0x28U   /* 2 字节, 只读 */
#define AX12A_ADDR_MOVING           0x2EU   /* 1 字节, 只读 */

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

/* ---- L1: 通用协议操作 ---- */

/**
 * @brief   从舵机寄存器读取数据
 * @param   bus     总线句柄指针
 * @param   id      舵机 ID (1~253)
 * @param   addr    寄存器起始地址
 * @param   dataLen 要读取的字节数
 * @param   data    输出缓冲区 (长度 >= dataLen)
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aRead(Ax12aBus *bus, uint8_t id, uint16_t addr,
                      uint8_t dataLen, uint8_t *data);

/**
 * @brief   向舵机寄存器写入数据
 * @param   bus     总线句柄指针
 * @param   id      舵机 ID (1~253) 或 AX12A_BROADCAST_ID 广播
 * @param   addr    寄存器起始地址
 * @param   dataLen 要写入的字节数
 * @param   data    写入数据缓冲区
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aWrite(Ax12aBus *bus, uint8_t id, uint16_t addr,
                       uint8_t dataLen, const uint8_t *data);

/* ---- L2: 语义层操作 ---- */

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

/**
 * @brief   启用/禁用舵机扭矩
 * @param   bus    总线句柄指针
 * @param   id     舵机 ID
 * @param   enable 1=启用, 0=禁用
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aSetTorqueEnable(Ax12aBus *bus, uint8_t id,
                                 uint8_t enable);

/**
 * @brief   设置舵机为伺服模式 (有限角度位置控制)
 * @note    AX-12A 伺服模式: CW=0, CCW=1023 (0~300°)
 *          设置前需先禁用扭矩，设置后重新使能
 * @param   bus 总线句柄指针
 * @param   id  舵机 ID
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aSetServoMode(Ax12aBus *bus, uint8_t id);

/* ---- L3: 组合层操作 ---- */

/**
 * @brief   同步写入三个舵机的目标位置和速度 (SYNC Write)
 * @note    使用广播 ID, 一次发送同时控制三个舵机
 * @return  Ax12aStatus
 */
Ax12aStatus ax12aSyncWritePosSpeed(Ax12aBus *bus,
    uint8_t id1, uint16_t pos1, uint16_t spd1,
    uint8_t id2, uint16_t pos2, uint16_t spd2,
    uint8_t id3, uint16_t pos3, uint16_t spd3);

#ifdef __cplusplus
}
#endif

#endif /* AX12A_H */
