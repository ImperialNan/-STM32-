/**
  ******************************************************************************
  * @file    bsp_ax12a.h
  * @brief   AX-12A 舵机统一驱动（Dynamixel Protocol 1.0）
  *          整合了写入控制（目标位置/速度）和反馈读取（当前位置/速度/负载），
  *          通过同一个 AX12A_t 实例完成所有操作。
  ******************************************************************************
  */
#ifndef __BSP_AX12A_H
#define __BSP_AX12A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*  Dynamixel Protocol 1.0 指令                                               */
/* ========================================================================== */

#define AX12A_INST_PING         0x01
#define AX12A_INST_READ         0x02
#define AX12A_INST_WRITE        0x03
#define AX12A_INST_SYNC_WRITE   0x83

/* ========================================================================== */
/*  Dynamixel Protocol 1.0 控制表地址                                          */
/* ========================================================================== */

/* 写入地址（目标值） */
#define AX12A_ADDR_GOAL_POSITION    0x1E  /* 目标位置（2字节, R/W） */
#define AX12A_ADDR_GOAL_SPEED       0x20  /* 目标速度（2字节, R/W） */

/* 读取地址（反馈值） */
#define AX12A_ADDR_PRESENT_POSITION 0x24  /* 当前位置（2字节, R） */
#define AX12A_ADDR_PRESENT_SPEED    0x26  /* 当前速度（2字节, R） */
#define AX12A_ADDR_PRESENT_LOAD     0x28  /* 当前负载（2字节, R） */
#define AX12A_ADDR_PRESENT_VOLTAGE  0x2A  /* 当前电压（1字节, R） */
#define AX12A_ADDR_PRESENT_TEMP     0x2B  /* 当前温度（1字节, R） */

/* ========================================================================== */
/*  AX-12A 参数                                                                */
/* ========================================================================== */

#define AX12A_POSITION_MAX      1023      /* 最大位置值 */
#define AX12A_ANGLE_RANGE       300.0f    /* 角度范围 0~300° */
#define AX12A_POS_TO_DEG        0.293f    /* 位置→角度系数 (300/1023 ≈ 0.293°) */
#define AX12A_DEG_TO_POS        3.41f     /* 角度→位置系数 (1023/300 ≈ 3.41) */
#define AX12A_SPEED_MAX         1023      /* 最大速度值 (CCW) */
#define AX12A_SPEED_UNIT_RPM    0.111f    /* 速度单位 ≈ 0.111 rpm */

/* RS485 方向控制 */
#define RS485_TX_ENABLE()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define RS485_TX_DISABLE()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

/* 超时时间 (ms) */
#define AX12A_TX_TIMEOUT        50
#define AX12A_RX_TIMEOUT        50

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/**
  * @brief  AX-12A 反馈数据
  */
typedef struct {
    /* 原始数据 */
    uint16_t rawPosition;   /* 原始位置值 (0~1023) */
    uint16_t rawSpeed;      /* 原始速度值 (0~2047) */
    uint16_t rawLoad;       /* 原始负载值 (0~2047) */

    /* 物理量 */
    float angleDeg;         /* 当前角度 (°) */
    float speedDps;         /* 当前角速度 (°/s) */
    float loadPct;          /* 当前负载 (%) */

    /* 状态 */
    uint8_t dataValid;      /* 数据有效标志 (1=有效) */
    uint32_t timestamp;     /* 最后更新时间戳 (ms) */
    uint8_t errorCode;      /* Dynamixel 错误码 */
} Ax12aData;

/**
  * @brief  AX-12A 实例结构体（统一：控制 + 反馈）
  */
typedef struct {
    /* 总线配置 */
    UART_HandleTypeDef *huart;   /* UART 句柄 (USART3) */
    uint8_t id;                  /* 舵机 ID */

    /* 软件限位 */
    float angleMin;              /* 软件角度下限 (°) */
    float angleMax;              /* 软件角度上限 (°) */

    /* 反馈数据 */
    Ax12aData feedback;          /* 反馈数据 */
} Ax12a;

/* ========================================================================== */
/*  初始化                                                                     */
/* ========================================================================== */

/**
  * @brief  初始化 AX-12A 实例
  * @param  ax12a: AX-12A 句柄
  * @param  huart: UART 句柄 (USART3)
  * @param  id: 舵机 ID (1~254)
  * @param  angleMin: 软件角度下限
  * @param  angleMax: 软件角度上限
  */
void ax12aInit(Ax12a *ax12a, UART_HandleTypeDef *huart,
               uint8_t id, float angleMin, float angleMax);

/* ========================================================================== */
/*  写入控制（目标位置/速度）                                                   */
/* ========================================================================== */

/**
  * @brief  设置舵机目标位置
  * @param  ax12a: AX-12A 句柄
  * @param  angle: 目标角度（度）
  * @param  speed: 运动速度（0~1023）
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef ax12aSetPosition(Ax12a *ax12a, float angle, uint16_t speed);

/**
  * @brief  同时设置多个舵机位置（SYNC WRITE）
  * @param  huart: UART 句柄
  * @param  ids: 舵机 ID 数组
  * @param  positions: 目标位置数组
  * @param  speeds: 目标速度数组
  * @param  count: 舵机数量
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef ax12aSyncWrite(UART_HandleTypeDef *huart,
                                 const uint8_t *ids,
                                 const uint16_t *positions,
                                 const uint16_t *speeds,
                                 uint8_t count);

/**
  * @brief  设置舵机波特率
  * @param  ax12a: AX-12A 句柄
  * @param  bpsCode: 波特率代码
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef ax12aSetBaudrate(Ax12a *ax12a, uint8_t bpsCode);

/* ========================================================================== */
/*  反馈读取（当前位置/速度/负载）                                              */
/* ========================================================================== */

/**
  * @brief  读取 AX-12A 当前位置、速度和负载（单次 READ 指令读取 6 字节）
  * @param  ax12a: AX-12A 句柄
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  */
HAL_StatusTypeDef ax12aReadFeedback(Ax12a *ax12a);

/**
  * @brief  获取 AX-12A 反馈数据指针
  * @param  ax12a: AX-12A 句柄
  * @retval Ax12aData 指针
  */
Ax12aData* ax12aGetFeedback(Ax12a *ax12a);

/**
  * @brief  检查 AX-12A 数据是否有效（超时检测）
  * @param  ax12a: AX-12A 句柄
  * @param  timeoutMs: 超时时间 (ms)
  * @retval 1=有效, 0=超时
  */
uint8_t ax12aIsDataValid(Ax12a *ax12a, uint32_t timeoutMs);

/**
  * @brief  AX-12A 位置→角度转换
  * @param  position: 原始位置值 (0~1023)
  * @retval 角度 (°)
  */
float ax12aPositionToAngle(uint16_t position);

/**
  * @brief  AX-12A 速度→角速度转换
  * @param  speed: 原始速度值 (0~2047)
  * @retval 角速度 (°/s)，正值=CCW，负值=CW
  */
float ax12aSpeedToDps(uint16_t speed);

/* ========================================================================== */
/*  诊断                                                                       */
/* ========================================================================== */

/**
  * @brief  PING 指令（检测 AX-12A 是否在线）
  * @param  ax12a: AX-12A 句柄
  * @retval HAL_OK=在线, HAL_ERROR/HAL_TIMEOUT=离线
  */
HAL_StatusTypeDef ax12aPing(Ax12a *ax12a);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_AX12A_H */
