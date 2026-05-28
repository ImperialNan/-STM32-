/**
  ******************************************************************************
  * @file    bsp_ax12a.c
  * @brief   AX-12A 舵机统一驱动实现（Dynamixel Protocol 1.0）
  *          整合了写入控制和反馈读取功能。
  ******************************************************************************
  */
#include "bsp_ax12a.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* 发送/接收缓冲区（静态分配，避免栈开销） */
static uint8_t s_txBuf[32];
static uint8_t s_rxBuf[16];

/* ========================================================================== */
/*  内部辅助函数                                                               */
/* ========================================================================== */

/**
  * @brief  发送 Dynamixel 指令包并计算校验和
  */
static HAL_StatusTypeDef sendPacket(UART_HandleTypeDef *huart,
                                    uint8_t *packet, uint8_t len)
{
    /* 校验和：~(ID + Length + Instruction + Parameters...) */
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        checksum += packet[i];
    }
    packet[len - 1] = ~checksum;

    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, packet, len, AX12A_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    return status;
}

/* ========================================================================== */
/*  初始化                                                                     */
/* ========================================================================== */

void ax12aInit(Ax12a *ax12a, UART_HandleTypeDef *huart,
               uint8_t id, float angleMin, float angleMax)
{
    ax12a->huart    = huart;
    ax12a->id       = id;
    ax12a->angleMin = angleMin;
    ax12a->angleMax = angleMax;

    memset(&ax12a->feedback, 0, sizeof(Ax12aData));
    ax12a->feedback.dataValid = 0;
}

/* ========================================================================== */
/*  写入控制（目标位置/速度）                                                   */
/* ========================================================================== */

HAL_StatusTypeDef ax12aSetPosition(Ax12a *ax12a, float angle, uint16_t speed)
{
    /* 角度限幅 */
    if (angle < ax12a->angleMin) angle = ax12a->angleMin;
    if (angle > ax12a->angleMax) angle = ax12a->angleMax;

    /* 角度→位置值 */
    uint16_t position = (uint16_t)(angle * AX12A_DEG_TO_POS);

    /* 构建 WRITE 指令包: FF FF ID LEN INST P1 P2 P3 P4 CHK */
    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x07;                              /* Length = 7 */
    s_txBuf[4] = AX12A_INST_WRITE;                  /* Write Data */
    s_txBuf[5] = AX12A_ADDR_GOAL_POSITION;          /* 起始地址 0x1E */
    s_txBuf[6] = (uint8_t)(position & 0xFF);        /* 位置低字节 */
    s_txBuf[7] = (uint8_t)((position >> 8) & 0xFF); /* 位置高字节 */
    s_txBuf[8] = (uint8_t)(speed & 0xFF);           /* 速度低字节 */
    s_txBuf[9] = (uint8_t)((speed >> 8) & 0xFF);    /* 速度高字节 */

    return sendPacket(ax12a->huart, s_txBuf, 11);
}

HAL_StatusTypeDef ax12aSyncWrite(UART_HandleTypeDef *huart,
                                 const uint8_t *ids,
                                 const uint16_t *positions,
                                 const uint16_t *speeds,
                                 uint8_t count)
{
    if (count == 0 || count > 5) return HAL_ERROR;

    uint8_t idx = 0;
    uint8_t dataLen = count * 4;  /* 每个舵机 4 字节数据 */

    s_txBuf[idx++] = 0xFF;
    s_txBuf[idx++] = 0xFF;
    s_txBuf[idx++] = 0xFE;                          /* 广播 ID */
    s_txBuf[idx++] = dataLen + 4;                    /* Length */
    s_txBuf[idx++] = AX12A_INST_SYNC_WRITE;          /* Sync Write */
    s_txBuf[idx++] = AX12A_ADDR_GOAL_POSITION;       /* 起始地址 */
    s_txBuf[idx++] = 0x04;                           /* 每个舵机写 4 字节 */

    for (uint8_t i = 0; i < count; i++) {
        s_txBuf[idx++] = ids[i];
        s_txBuf[idx++] = (uint8_t)(positions[i] & 0xFF);
        s_txBuf[idx++] = (uint8_t)((positions[i] >> 8) & 0xFF);
        s_txBuf[idx++] = (uint8_t)(speeds[i] & 0xFF);
        s_txBuf[idx++] = (uint8_t)((speeds[i] >> 8) & 0xFF);
    }

    return sendPacket(huart, s_txBuf, idx + 1);
}

HAL_StatusTypeDef ax12aSetBaudrate(Ax12a *ax12a, uint8_t bpsCode)
{
    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x04;                    /* Length = 4 */
    s_txBuf[4] = AX12A_INST_WRITE;        /* Write */
    s_txBuf[5] = 0x04;                    /* 地址 0x04 = Baud Rate */
    s_txBuf[6] = bpsCode;

    return sendPacket(ax12a->huart, s_txBuf, 8);
}

HAL_StatusTypeDef ax12aReadFeedback(Ax12a *ax12a)
{
    /* 读取 Present Position (addr 0x24, 2字节) + Present Speed (addr 0x26, 2字节) + Present Load (addr 0x28, 2字节) */
    /* 总共读取 6 字节，从地址 0x24 开始 */
    /* 响应格式: FF FF ID LEN ERR POS_L POS_H SPD_L SPD_H LOAD_L LOAD_H CHK */
    /* 响应长度 = 4(header+id+len+err) + 6(data) + 1(checksum) = 11 */

    uint8_t rxBuf[12];
    HAL_StatusTypeDef status;

    /* 构建 READ 指令包: FF FF ID LEN INST ADDR LEN CHK */
    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x04;           /* Length = 4 */
    s_txBuf[4] = AX12A_INST_READ;
    s_txBuf[5] = AX12A_ADDR_PRESENT_POSITION;  /* 起始地址 0x24 */
    s_txBuf[6] = 0x06;           /* 读取 6 字节 (Position + Speed + Load) */

    /* 计算校验和 */
    uint8_t checksum = ~(s_txBuf[2] + s_txBuf[3] + s_txBuf[4] +
                          s_txBuf[5] + s_txBuf[6]);
    s_txBuf[7] = checksum;

    /* 发送指令 */
    RS485_TX_ENABLE();
    status = HAL_UART_Transmit(ax12a->huart, s_txBuf, 8, AX12A_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(ax12a->huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    if (status != HAL_OK) {
        ax12a->feedback.dataValid = 0;
        return status;
    }

    /* 接收响应 (12字节: FF FF ID LEN ERR POS_L POS_H SPD_L SPD_H LOAD_L LOAD_H CHK) */
    status = HAL_UART_Receive(ax12a->huart, rxBuf, 12, AX12A_RX_TIMEOUT);
    if (status != HAL_OK) {
        ax12a->feedback.dataValid = 0;
        return status;
    }

    /* 验证帧头 */
    if (rxBuf[0] != 0xFF || rxBuf[1] != 0xFF) {
        ax12a->feedback.dataValid = 0;
        return HAL_ERROR;
    }

    /* 验证校验和 */
    checksum = 0;
    for (uint8_t i = 2; i < 11; i++) {
        checksum += rxBuf[i];
    }
    checksum = ~checksum;
    if (checksum != rxBuf[11]) {
        ax12a->feedback.dataValid = 0;
        return HAL_ERROR;
    }

    /* 检查错误码 */
    ax12a->feedback.errorCode = rxBuf[4];
    if (rxBuf[4] != 0) {
        ax12a->feedback.dataValid = 0;
        return HAL_ERROR;
    }

    /* 解析数据 (Little Endian) */
    ax12a->feedback.rawPosition = (uint16_t)((rxBuf[6] << 8) | rxBuf[5]);
    ax12a->feedback.rawSpeed    = (uint16_t)((rxBuf[8] << 8) | rxBuf[7]);
    ax12a->feedback.rawLoad     = (uint16_t)((rxBuf[10] << 8) | rxBuf[9]);

    /* 转换为物理量 */
    ax12a->feedback.angleDeg = ax12aPositionToAngle(ax12a->feedback.rawPosition);
    ax12a->feedback.speedDps = ax12aSpeedToDps(ax12a->feedback.rawSpeed);

    /* 负载转换 (0~1023 = CCW 0~100%, 1024~2047 = CW 0~100%) */
    if (ax12a->feedback.rawLoad <= 1023) {
        ax12a->feedback.loadPct = (float)ax12a->feedback.rawLoad / 1023.0f * 100.0f;
    } else {
        ax12a->feedback.loadPct = (float)(ax12a->feedback.rawLoad - 1024) / 1023.0f * 100.0f;
    }

    /* 更新状态 */
    ax12a->feedback.dataValid = 1;
    ax12a->feedback.timestamp = HAL_GetTick();

    return HAL_OK;
}

Ax12aData* ax12aGetFeedback(Ax12a *ax12a)
{
    return &ax12a->feedback;
}

uint8_t ax12aIsDataValid(Ax12a *ax12a, uint32_t timeoutMs)
{
    if (!ax12a->feedback.dataValid) {
        return 0;
    }
    /* 检查是否超时 */
    if ((HAL_GetTick() - ax12a->feedback.timestamp) > timeoutMs) {
        ax12a->feedback.dataValid = 0;
        return 0;
    }
    return 1;
}

float ax12aPositionToAngle(uint16_t position)
{
    /* AX-12A: 0~1023 对应 0~300° */
    return (float)position * AX12A_POS_TO_DEG;
}

float ax12aSpeedToDps(uint16_t speed)
{
    /*
     * AX-12A Present Speed:
     *   0~1023  = CCW 方向，单位 ≈ 0.111 rpm
     *   1024~2047 = CW 方向，单位 ≈ 0.111 rpm (值 - 1024)
     *
     * 转换为 °/s: rpm * 360 / 60 = rpm * 6
     */
    float rpm;
    if (speed <= 1023) {
        /* CCW: 正值 */
        rpm = (float)speed * AX12A_SPEED_UNIT_RPM;
    } else {
        /* CW: 负值 */
        rpm = -((float)(speed - 1024) * AX12A_SPEED_UNIT_RPM);
    }
    return rpm * 6.0f;  /* rpm → °/s */
}

HAL_StatusTypeDef ax12aPing(Ax12a *ax12a)
{
    /* PING 指令: FF FF ID 01 CHK */
    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x02;  /* Length = 2 */
    s_txBuf[4] = AX12A_INST_PING;

    /* 校验和 */
    s_txBuf[5] = ~(ax12a->id + 0x02 + AX12A_INST_PING);

    /* 发送 */
    RS485_TX_ENABLE();
    HAL_StatusTypeDef status = HAL_UART_Transmit(ax12a->huart, s_txBuf, 6, AX12A_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(ax12a->huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    if (status != HAL_OK) return status;

    /* 接收响应: FF FF ID LEN ERR CHK */
    status = HAL_UART_Receive(ax12a->huart, s_rxBuf, 6, AX12A_RX_TIMEOUT);
    if (status != HAL_OK) return status;

    /* 验证帧头 */
    if (s_rxBuf[0] != 0xFF || s_rxBuf[1] != 0xFF) {
        return HAL_ERROR;
    }

    /* 验证校验和 */
    uint8_t calcChk = ~(s_rxBuf[2] + s_rxBuf[3] + s_rxBuf[4]);
    if (calcChk != s_rxBuf[5]) {
        return HAL_ERROR;
    }

    return HAL_OK;
}
