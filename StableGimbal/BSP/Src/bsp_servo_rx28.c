/**
  ******************************************************************************
  * @file    bsp_servo_rx28.c
  * @brief   RX28 舵机驱动实现
  ******************************************************************************
  */
#include "bsp_servo_rx28.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* 发送缓冲区 */
static uint8_t tx_buf[32];

/**
  * @brief  发送指令包并计算校验和
  */
static HAL_StatusTypeDef send_packet(UART_HandleTypeDef *huart,
                                      uint8_t *packet, uint8_t len)
{
    /* 计算校验和：~(ID + Length + Instruction + Parameters...) */
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        checksum += packet[i];
    }
    packet[len - 1] = ~checksum;

    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, packet, len, SERVO_TX_TIMEOUT);
    /* 等待最后一个字节发送完成（TC标志） */
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    return status;
}

void Servo_Init(Servo_RX28_t *servo, UART_HandleTypeDef *huart,
                uint8_t id, float angle_min, float angle_max)
{
    servo->huart     = huart;
    servo->id        = id;
    servo->angle_min = angle_min;
    servo->angle_max = angle_max;
}

HAL_StatusTypeDef Servo_SetPosition(Servo_RX28_t *servo, float angle, uint16_t speed)
{
    /* 角度限幅 */
    if (angle < servo->angle_min) angle = servo->angle_min;
    if (angle > servo->angle_max) angle = servo->angle_max;

    /* 角度→位置值 */
    uint16_t position = (uint16_t)(angle * SERVO_DEG_TO_POS);

    /* 构建 WRITE 指令包 */
    /* 格式: FF FF ID LEN INST P1 P2 P3 P4 CHK */
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = servo->id;
    tx_buf[3] = 0x07;                       /* Length = 7 */
    tx_buf[4] = RX28_INST_WRITE;            /* Write Data */
    tx_buf[5] = RX28_ADDR_GOAL_POSITION;    /* 起始地址 0x1E */
    tx_buf[6] = (uint8_t)(position & 0xFF);       /* 位置低字节 */
    tx_buf[7] = (uint8_t)((position >> 8) & 0xFF); /* 位置高字节 */
    tx_buf[8] = (uint8_t)(speed & 0xFF);           /* 速度低字节 */
    tx_buf[9] = (uint8_t)((speed >> 8) & 0xFF);    /* 速度高字节 */

    return send_packet(servo->huart, tx_buf, 11);
}

HAL_StatusTypeDef Servo_ReadPosition(Servo_RX28_t *servo, uint16_t *position)
{
    uint8_t rx_buf[8];
    HAL_StatusTypeDef status;

    /* 构建 READ 指令包 */
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = servo->id;
    tx_buf[3] = 0x04;                            /* Length = 4 */
    tx_buf[4] = RX28_INST_READ;                  /* Read Data */
    tx_buf[5] = RX28_ADDR_PRESENT_POSITION;      /* 起始地址 0x24 */
    tx_buf[6] = 0x02;                            /* 读取 2 字节 */

    /* 计算校验和 */
    uint8_t checksum = ~(tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6]);
    tx_buf[7] = checksum;

    RS485_TX_ENABLE();
    status = HAL_UART_Transmit(servo->huart, tx_buf, 8, SERVO_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(servo->huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    if (status != HAL_OK) return status;

    /* 接收响应 (8字节: FF FF ID LEN ERR POS_L POS_H CHK) */
    status = HAL_UART_Receive(servo->huart, rx_buf, 8, SERVO_RX_TIMEOUT);
    if (status != HAL_OK) return status;

    /* 验证帧头 */
    if (rx_buf[0] != 0xFF || rx_buf[1] != 0xFF) {
        return HAL_ERROR;
    }

    /* 验证校验和 */
    uint8_t calc_chk = ~(rx_buf[2] + rx_buf[3] + rx_buf[4] + rx_buf[5] + rx_buf[6]);
    if (calc_chk != rx_buf[7]) {
        return HAL_ERROR;
    }

    /* 检查错误码 */
    if (rx_buf[4] != 0) {
        return HAL_ERROR;
    }

    *position = (uint16_t)((rx_buf[6] << 8) | rx_buf[5]);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SyncWrite(UART_HandleTypeDef *huart,
                                   const uint8_t *ids,
                                   const uint16_t *positions,
                                   const uint16_t *speeds,
                                   uint8_t count)
{
    if (count == 0 || count > 5) return HAL_ERROR;

    uint8_t idx = 0;
    uint8_t data_len = count * 4;  /* 每个舵机 4 字节数据 */

    tx_buf[idx++] = 0xFF;
    tx_buf[idx++] = 0xFF;
    tx_buf[idx++] = 0xFE;                       /* 广播 ID */
    tx_buf[idx++] = data_len + 4;               /* Length */
    tx_buf[idx++] = RX28_INST_SYNC_WRITE;       /* Sync Write */
    tx_buf[idx++] = RX28_ADDR_GOAL_POSITION;    /* 起始地址 */
    tx_buf[idx++] = 0x04;                       /* 每个舵机写 4 字节 */

    for (uint8_t i = 0; i < count; i++) {
        tx_buf[idx++] = ids[i];
        tx_buf[idx++] = (uint8_t)(positions[i] & 0xFF);
        tx_buf[idx++] = (uint8_t)((positions[i] >> 8) & 0xFF);
        tx_buf[idx++] = (uint8_t)(speeds[i] & 0xFF);
        tx_buf[idx++] = (uint8_t)((speeds[i] >> 8) & 0xFF);
    }

    return send_packet(huart, tx_buf, idx + 1);
}

HAL_StatusTypeDef Servo_SetBaudrate(Servo_RX28_t *servo, uint8_t bps_code)
{
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = servo->id;
    tx_buf[3] = 0x04;                    /* Length = 4 */
    tx_buf[4] = RX28_INST_WRITE;         /* Write */
    tx_buf[5] = 0x04;                    /* 地址 0x04 = Baud Rate */
    tx_buf[6] = bps_code;

    return send_packet(servo->huart, tx_buf, 8);
}
