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
    uint8_t dataLen = count * 5;  /* 每舵机: ID(1) + Position(2) + Speed(2) = 5 */

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

/* ========================================================================== */
/*  非阻塞反馈读取状态机（DMA+IDLE，替代阻塞式 ax12aReadFeedback）                */
/* ========================================================================== */

/* 反馈读取上下文（单例） */
static Ax12aFbContext g_fbCtx;
static UART_HandleTypeDef *g_fbUart;  /* 快速引用，避免每次通过 servo->huart */

/* 内部：构建 READ 指令包 (8字节) 到 s_txBuf */
static void buildReadPacket(Ax12a *ax12a)
{
    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x04;                              /* Length = 4 */
    s_txBuf[4] = AX12A_INST_READ;
    s_txBuf[5] = AX12A_ADDR_PRESENT_POSITION;        /* 起始地址 0x24 */
    s_txBuf[6] = 0x06;                               /* 读取 6 字节 */
    s_txBuf[7] = ~(s_txBuf[2] + s_txBuf[3] + s_txBuf[4]
                  + s_txBuf[5] + s_txBuf[6]);        /* 校验和 */
}

/* 内部：解析 READ 响应帧，更新舵机 feedback */
static void parseResponseFrame(Ax12a *ax12a, uint8_t *buf, uint16_t len)
{
    /* 最小长度检查: FF FF ID LEN ERR ... CHK = 至少 8 字节 */
    if (len < 8) return;
    if (buf[0] != 0xFF || buf[1] != 0xFF) return;

    uint8_t frameLen = buf[3];  /* LEN = Error + Params + Checksum */

    /* Status Packet 总长 = 4(header+id+len) + frameLen */
    if (len < (uint16_t)(4 + frameLen)) return;

    /* 校验和 */
    uint8_t chk = 0;
    for (uint8_t i = 2; i < 4 + frameLen - 1; i++) {
        chk += buf[i];
    }
    if ((uint8_t)~chk != buf[4 + frameLen - 1]) return;

    /* 检查错误码 */
    ax12a->feedback.errorCode = buf[4];
    if (buf[4] != 0) {
        ax12a->feedback.dataValid = 0;
        return;
    }

    /* 验证响应 ID 匹配请求的舵机 */
    if (buf[2] != ax12a->id) {
        ax12a->feedback.dataValid = 0;
        return;
    }

    /* 解析数据（至少需要 Position + Speed + Load = 6 字节参数） */
    if (frameLen < 8) {  /* Error(1) + Params(6) + Checksum(1) = 8 */
        ax12a->feedback.dataValid = 0;
        return;
    }

    ax12a->feedback.rawPosition = (uint16_t)((buf[6] << 8) | buf[5]);
    ax12a->feedback.rawSpeed    = (uint16_t)((buf[8] << 8) | buf[7]);
    ax12a->feedback.rawLoad     = (uint16_t)((buf[10] << 8) | buf[9]);
    ax12a->feedback.angleDeg    = ax12aPositionToAngle(ax12a->feedback.rawPosition);
    ax12a->feedback.speedDps    = ax12aSpeedToDps(ax12a->feedback.rawSpeed);

    if (ax12a->feedback.rawLoad <= 1023) {
        ax12a->feedback.loadPct = (float)ax12a->feedback.rawLoad / 1023.0f * 100.0f;
    } else {
        ax12a->feedback.loadPct = (float)(ax12a->feedback.rawLoad - 1024) / 1023.0f * 100.0f;
    }

    ax12a->feedback.dataValid = 1;
    ax12a->feedback.timestamp = HAL_GetTick();
}

/* 内部：启动下一个舵机的 READ 发送（ISR 安全，使用 IT 模式） */
static void sendNextRead(void)
{
    if (g_fbCtx.curIdx >= g_fbCtx.servoCount) {
        g_fbCtx.state = AX12A_FB_DONE;
        g_fbCtx.done = 1;
        return;
    }

    Ax12a *servo = g_fbCtx.servos[g_fbCtx.curIdx];
    buildReadPacket(servo);

    RS485_TX_ENABLE();
    /* 使用中断式发送，完成后 HAL_UART_TxCpltCallback 会被触发 */
    HAL_UART_Transmit_IT(g_fbUart, s_txBuf, 8);
    g_fbCtx.state = AX12A_FB_WAIT_TC;
}

/* ========================================================================== */
/*  HAL 回调覆盖                                                               */
/* ========================================================================== */

/**
  * @brief  UART TX 完成回调（由 HAL_UART_IRQHandler 触发）
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_fbUart && g_fbCtx.state == AX12A_FB_WAIT_TC) {
        /* TX 完成 → 等待最后一位移出 → 切到接收模式 */
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {}

        RS485_TX_DISABLE();
        HAL_HalfDuplex_EnableReceiver(huart);

        /* 启动 DMA+IDLE 接收，响应到达后 HAL_UARTEx_RxEventCallback 被调用 */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_fbCtx.rxBuf, sizeof(g_fbCtx.rxBuf));
        g_fbCtx.state = AX12A_FB_WAIT_RX;
        g_fbCtx.timeoutTick = HAL_GetTick();
    }
}

/**
  * @brief  UART DMA+IDLE 接收完成回调
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != g_fbUart || g_fbCtx.state != AX12A_FB_WAIT_RX) return;

    /* 解析当前舵机响应 */
    Ax12a *servo = g_fbCtx.servos[g_fbCtx.curIdx];
    parseResponseFrame(servo, g_fbCtx.rxBuf, Size);

    /* 推进到下一个舵机 */
    g_fbCtx.curIdx++;
    sendNextRead();
}

/* ========================================================================== */
/*  公开 API                                                                   */
/* ========================================================================== */

void ax12aStartFeedbackRead(Ax12a **servos, uint8_t count)
{
    if (count == 0 || count > 5) return;

    /* 如果上一次读取还在进行中，强制终止 */
    if (g_fbCtx.state != AX12A_FB_IDLE && g_fbCtx.state != AX12A_FB_DONE
        && g_fbCtx.state != AX12A_FB_TIMEOUT) {
        HAL_UART_AbortReceive(g_fbUart);
    }

    memset(&g_fbCtx, 0, sizeof(g_fbCtx));
    for (uint8_t i = 0; i < count; i++) {
        g_fbCtx.servos[i] = servos[i];
    }
    g_fbCtx.servoCount = count;
    g_fbCtx.curIdx     = 0;
    g_fbCtx.done       = 0;
    g_fbCtx.state      = AX12A_FB_IDLE;
    g_fbUart           = servos[0]->huart;

    /* 发送第一个 READ（阻塞发送，因为此时在主循环上下文） */
    Ax12a *first = g_fbCtx.servos[0];
    buildReadPacket(first);

    RS485_TX_ENABLE();
    HAL_UART_Transmit(g_fbUart, s_txBuf, 8, AX12A_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(g_fbUart, UART_FLAG_TC) == RESET) {}

    RS485_TX_DISABLE();
    HAL_HalfDuplex_EnableReceiver(g_fbUart);

    HAL_UARTEx_ReceiveToIdle_DMA(g_fbUart, g_fbCtx.rxBuf, sizeof(g_fbCtx.rxBuf));
    g_fbCtx.state = AX12A_FB_WAIT_RX;
    g_fbCtx.timeoutTick = HAL_GetTick();
}

uint8_t ax12aIsFeedbackDone(void)
{
    return g_fbCtx.done;
}

void ax12aFeedbackPoll(void)
{
    /* 仅检查超时：WAIT_RX 状态下超过 50ms 无响应则中止 */
    if (g_fbCtx.state == AX12A_FB_WAIT_RX) {
        if ((HAL_GetTick() - g_fbCtx.timeoutTick) > AX12A_RX_TIMEOUT) {
            /* 清除当前舵机的无效数据 */
            Ax12a *servo = g_fbCtx.servos[g_fbCtx.curIdx];
            servo->feedback.dataValid = 0;

            g_fbCtx.timeoutServo++;
            g_fbCtx.curIdx++;

            /* 中止 DMA 接收 */
            HAL_UART_AbortReceive(g_fbUart);

            /* 尝试继续读取下一个舵机 */
            if (g_fbCtx.curIdx < g_fbCtx.servoCount) {
                sendNextRead();
            } else {
                g_fbCtx.state = AX12A_FB_DONE;
                g_fbCtx.done = 1;
            }
        }
    }
}

/* ========================================================================== */
/*  保留旧 API（内部兼容，不推荐直接调用）                                      */
/* ========================================================================== */

HAL_StatusTypeDef ax12aReadFeedback(Ax12a *ax12a)
{
    uint8_t rxBuf[12];
    HAL_StatusTypeDef status;

    s_txBuf[0] = 0xFF;
    s_txBuf[1] = 0xFF;
    s_txBuf[2] = ax12a->id;
    s_txBuf[3] = 0x04;
    s_txBuf[4] = AX12A_INST_READ;
    s_txBuf[5] = AX12A_ADDR_PRESENT_POSITION;
    s_txBuf[6] = 0x06;
    s_txBuf[7] = ~(s_txBuf[2] + s_txBuf[3] + s_txBuf[4]
                  + s_txBuf[5] + s_txBuf[6]);

    RS485_TX_ENABLE();
    status = HAL_UART_Transmit(ax12a->huart, s_txBuf, 8, AX12A_TX_TIMEOUT);
    while (__HAL_UART_GET_FLAG(ax12a->huart, UART_FLAG_TC) == RESET) {}
    RS485_TX_DISABLE();

    if (status != HAL_OK) {
        ax12a->feedback.dataValid = 0;
        return status;
    }

    status = HAL_UART_Receive(ax12a->huart, rxBuf, 12, AX12A_RX_TIMEOUT);
    if (status != HAL_OK) {
        ax12a->feedback.dataValid = 0;
        return status;
    }

    parseResponseFrame(ax12a, rxBuf, 12);
    return (ax12a->feedback.dataValid) ? HAL_OK : HAL_ERROR;
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
