/**
  ******************************************************************************
  * @file    bsp_servo_rx28.h
  * @brief   RX28 舵机驱动（Dynamixel 兼容协议，RS485 总线）
  ******************************************************************************
  */
#ifndef __BSP_SERVO_RX28_H
#define __BSP_SERVO_RX28_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 舵机参数 */
#define SERVO_ANGLE_MIN      0.0f      /* 最小角度（度） */
#define SERVO_ANGLE_MAX      300.0f    /* 最大角度（度） */
#define SERVO_POSITION_MAX   0x3FF     /* 最大位置值（1023） */
#define SERVO_DEG_TO_POS     3.41f     /* 角度→位置系数 (1023/300) */

/* 协议指令 */
#define RX28_INST_READ       0x02
#define RX28_INST_WRITE      0x03
#define RX28_INST_SYNC_WRITE 0x83

/* 控制表地址 */
#define RX28_ADDR_GOAL_POSITION  0x1E  /* 目标位置（2字节） */
#define RX28_ADDR_GOAL_SPEED     0x20  /* 目标速度（2字节） */
#define RX28_ADDR_PRESENT_POSITION 0x24 /* 当前位置（2字节） */

/* RS485 方向控制 */
#define RS485_TX_ENABLE()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define RS485_TX_DISABLE()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

/* 超时时间（ms） */
#define SERVO_TX_TIMEOUT    50
#define SERVO_RX_TIMEOUT    50

/* 舵机实例结构体 */
typedef struct {
    UART_HandleTypeDef *huart;  /* 连接的 UART 句柄（USART3） */
    uint8_t id;                 /* 舵机 ID */
    float   angle_min;          /* 软件角度下限 */
    float   angle_max;          /* 软件角度上限 */
} Servo_RX28_t;

/**
  * @brief  初始化舵机实例
  * @param  servo: 舵机句柄
  * @param  huart: UART 句柄
  * @param  id: 舵机 ID (1~254)
  * @param  angle_min: 软件角度下限
  * @param  angle_max: 软件角度上限
  */
void Servo_Init(Servo_RX28_t *servo, UART_HandleTypeDef *huart,
                uint8_t id, float angle_min, float angle_max);

/**
  * @brief  设置舵机目标位置
  * @param  servo: 舵机句柄
  * @param  angle: 目标角度（度）
  * @param  speed: 运动速度（0~1023）
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef Servo_SetPosition(Servo_RX28_t *servo, float angle, uint16_t speed);

/**
  * @brief  读取舵机当前位置
  * @param  servo: 舵机句柄
  * @param  position: 输出位置值（0~1023）
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef Servo_ReadPosition(Servo_RX28_t *servo, uint16_t *position);

/**
  * @brief  同时设置多个舵机位置（SYNC WRITE）
  * @param  huart: UART 句柄
  * @param  ids: 舵机 ID 数组
  * @param  positions: 目标位置数组
  * @param  speeds: 目标速度数组
  * @param  count: 舵机数量
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef Servo_SyncWrite(UART_HandleTypeDef *huart,
                                   const uint8_t *ids,
                                   const uint16_t *positions,
                                   const uint16_t *speeds,
                                   uint8_t count);

/**
  * @brief  设置舵机波特率
  * @param  servo: 舵机句柄
  * @param  bps_code: 波特率代码
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef Servo_SetBaudrate(Servo_RX28_t *servo, uint8_t bps_code);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_RX28_H */
