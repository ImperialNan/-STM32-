/**
 * @file    ax12a.c
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动实现
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 2.0
 */

#include "ax12a.h"

/* ========================= 私有宏定义 ============================== */

/* RS485 方向控制：高电平 = 发送, 低电平 = 接收 */
#define RS485_TX_MODE(bus) \
    HAL_GPIO_WritePin((bus)->txEnPort, (bus)->txEnPin, GPIO_PIN_SET)
#define RS485_RX_MODE(bus) \
    HAL_GPIO_WritePin((bus)->txEnPort, (bus)->txEnPin, GPIO_PIN_RESET)

/* 校验和计算: ~ (ID + Length + Instruction + Params...) */
#define CALC_CHECKSUM(pkt, start, end) \
    do { \
        uint8_t _sum = 0; \
        for (uint8_t _i = (start); _i <= (end); _i++) { \
            _sum += (pkt)[_i]; \
        } \
        (pkt)[(end) + 1] = ~_sum; \
    } while (0)

/* ========================= 私有函数声明 ============================ */

static Ax12aStatus sendPacket(Ax12aBus *bus, const uint8_t *pkt,
                              uint8_t len);
static Ax12aStatus recvPacket(Ax12aBus *bus, uint8_t *buf, uint8_t len);
static Ax12aStatus verifyStatus(const uint8_t *buf, uint8_t expectedId,
                                uint8_t dataLen);

/* ========================= 公开 API 实现 =========================== */

/**
 * @brief   初始化总线句柄
 */
void ax12aInit(Ax12aBus *bus, UART_HandleTypeDef *huart,
               GPIO_TypeDef *txEnPort, uint16_t txEnPin)
{
    bus->huart    = huart;
    bus->txEnPort = txEnPort;
    bus->txEnPin  = txEnPin;
    RS485_RX_MODE(bus);
}

/**
 * @brief   从舵机寄存器读取数据
 */
Ax12aStatus ax12aRead(Ax12aBus *bus, uint8_t id, uint16_t addr,
                      uint8_t dataLen, uint8_t *data)
{
    uint8_t cmdPkt[8];
    uint8_t rspPkt[16];
    uint8_t rspLen;
    Ax12aStatus status;

    if (bus == NULL || data == NULL || dataLen == 0) {
        return AX12A_ERR_PARAM;
    }

    /* 构建 READ 指令包 */
    cmdPkt[0] = AX12A_HEADER_1;
    cmdPkt[1] = AX12A_HEADER_2;
    cmdPkt[2] = id;
    cmdPkt[3] = 0x04;
    cmdPkt[4] = AX12A_INST_READ;
    cmdPkt[5] = (uint8_t)(addr & 0xFF);
    cmdPkt[6] = dataLen;
    CALC_CHECKSUM(cmdPkt, 2, 6);

    status = sendPacket(bus, cmdPkt, sizeof(cmdPkt));
    if (status != AX12A_OK) {
        return status;
    }

    rspLen = AX12A_STATUS_PKT_BASE + dataLen;
    status = recvPacket(bus, rspPkt, rspLen);
    if (status != AX12A_OK) {
        return status;
    }

    status = verifyStatus(rspPkt, id, dataLen);
    if (status != AX12A_OK) {
        return status;
    }

    for (uint8_t i = 0; i < dataLen; i++) {
        data[i] = rspPkt[5 + i];
    }

    return AX12A_OK;
}

/**
 * @brief   向舵机寄存器写入数据
 */
Ax12aStatus ax12aWrite(Ax12aBus *bus, uint8_t id, uint16_t addr,
                       uint8_t dataLen, const uint8_t *data)
{
    uint8_t cmdPkt[16];
    uint8_t cmdLen;

    if (bus == NULL || data == NULL || dataLen == 0) {
        return AX12A_ERR_PARAM;
    }

    cmdPkt[0] = AX12A_HEADER_1;
    cmdPkt[1] = AX12A_HEADER_2;
    cmdPkt[2] = id;
    cmdPkt[3] = dataLen + 3;
    cmdPkt[4] = AX12A_INST_WRITE;
    cmdPkt[5] = (uint8_t)(addr & 0xFF);

    for (uint8_t i = 0; i < dataLen; i++) {
        cmdPkt[6 + i] = data[i];
    }

    cmdLen = 6 + dataLen;
    CALC_CHECKSUM(cmdPkt, 2, cmdLen - 1);

    return sendPacket(bus, cmdPkt, cmdLen + 1);
}

/**
 * @brief   设置舵机目标位置
 */
Ax12aStatus ax12aSetGoalPosition(Ax12aBus *bus, uint8_t id,
                                 uint16_t pos)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(pos & 0xFF);
    buf[1] = (uint8_t)(pos >> 8);

    return ax12aWrite(bus, id, AX12A_ADDR_GOAL_POSITION, 2, buf);
}

/**
 * @brief   设置舵机移动速度
 */
Ax12aStatus ax12aSetMovingSpeed(Ax12aBus *bus, uint8_t id,
                                uint16_t spd)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(spd & 0xFF);
    buf[1] = (uint8_t)(spd >> 8);

    return ax12aWrite(bus, id, AX12A_ADDR_MOVING_SPEED, 2, buf);
}

/**
 * @brief   启用/禁用舵机扭矩
 */
Ax12aStatus ax12aSetTorqueEnable(Ax12aBus *bus, uint8_t id,
                                 uint8_t enable)
{
    uint8_t val = enable ? 1 : 0;

    return ax12aWrite(bus, id, AX12A_ADDR_TORQUE_ENABLE, 1, &val);
}

/**
 * @brief   设置舵机为伺服模式 (有限角度位置控制)
 */
Ax12aStatus ax12aSetServoMode(Ax12aBus *bus, uint8_t id)
{
    uint8_t buf[2];
    Ax12aStatus status;

    /* 1. 禁用扭矩 (修改角度限制前必须禁用) */
    status = ax12aSetTorqueEnable(bus, id, 0);
    if (status != AX12A_OK) {
        return status;
    }

    /* 2. 设置 CW Angle Limit = 0 */
    buf[0] = 0x00;
    buf[1] = 0x00;
    status = ax12aWrite(bus, id, AX12A_ADDR_CW_ANGLE_LIMIT, 2, buf);
    if (status != AX12A_OK) {
        return status;
    }

    /* 3. 设置 CCW Angle Limit = 1023 (300°) */
    buf[0] = 0xFF;
    buf[1] = 0x03;
    status = ax12aWrite(bus, id, AX12A_ADDR_CCW_ANGLE_LIMIT, 2, buf);
    if (status != AX12A_OK) {
        return status;
    }

    /* 4. 重新使能扭矩 */
    return ax12aSetTorqueEnable(bus, id, 1);
}

/**
 * @brief   同步写入三个舵机的目标位置和速度 (SYNC Write)
 */
Ax12aStatus ax12aSyncWritePosSpeed(Ax12aBus *bus,
    uint8_t id1, uint16_t pos1, uint16_t spd1,
    uint8_t id2, uint16_t pos2, uint16_t spd2,
    uint8_t id3, uint16_t pos3, uint16_t spd3)
{
    /* SYNC Write: [FF FF FE 13 83 1E 04 ID1 P1L P1H S1L S1H
     *              ID2 P2L P2H S2L S2H ID3 P3L P3H S3L S3H CKSM]
     * 每舵机数据: ID(1) + Position(2) + Speed(2) = 5 字节
     * Length = 2 + 3*5 + 2 = 19 = 0x13 */
    uint8_t cmdPkt[23];

    cmdPkt[0] = AX12A_HEADER_1;
    cmdPkt[1] = AX12A_HEADER_2;
    cmdPkt[2] = AX12A_BROADCAST_ID;
    cmdPkt[3] = 0x13;
    cmdPkt[4] = AX12A_INST_SYNC_WRITE;
    cmdPkt[5] = AX12A_ADDR_GOAL_POSITION;
    cmdPkt[6] = 0x04;

    /* 舵机 1 */
    cmdPkt[7]  = id1;
    cmdPkt[8]  = (uint8_t)(pos1 & 0xFF);
    cmdPkt[9]  = (uint8_t)(pos1 >> 8);
    cmdPkt[10] = (uint8_t)(spd1 & 0xFF);
    cmdPkt[11] = (uint8_t)(spd1 >> 8);

    /* 舵机 2 */
    cmdPkt[12] = id2;
    cmdPkt[13] = (uint8_t)(pos2 & 0xFF);
    cmdPkt[14] = (uint8_t)(pos2 >> 8);
    cmdPkt[15] = (uint8_t)(spd2 & 0xFF);
    cmdPkt[16] = (uint8_t)(spd2 >> 8);

    /* 舵机 3 */
    cmdPkt[17] = id3;
    cmdPkt[18] = (uint8_t)(pos3 & 0xFF);
    cmdPkt[19] = (uint8_t)(pos3 >> 8);
    cmdPkt[20] = (uint8_t)(spd3 & 0xFF);
    cmdPkt[21] = (uint8_t)(spd3 >> 8);

    CALC_CHECKSUM(cmdPkt, 2, 21);

    return sendPacket(bus, cmdPkt, sizeof(cmdPkt));
}

/* ========================= 私有函数实现 ============================ */

/**
 * @brief   发送指令包 (阻塞)
 */
static Ax12aStatus sendPacket(Ax12aBus *bus, const uint8_t *pkt,
                              uint8_t len)
{
    HAL_StatusTypeDef halStatus;

    /* 清除溢出标志 (读SR+读DR序列, 否则ORE会阻塞后续收发) */
    if (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_ORE) != RESET) {
        volatile uint32_t dummy = bus->huart->Instance->DR;
        (void)dummy;
    }

    RS485_TX_MODE(bus);

    halStatus = HAL_UART_Transmit(bus->huart, (uint8_t *)pkt, len,
                                  AX12A_TIMEOUT_MS);
    if (halStatus != HAL_OK) {
        RS485_RX_MODE(bus);
        return AX12A_ERR_TIMEOUT;
    }

    /* 等待发送完成 (TC), 带超时保护 */
    {
        uint32_t tcStart = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - tcStart) >= AX12A_TIMEOUT_MS) {
                RS485_RX_MODE(bus);
                return AX12A_ERR_TIMEOUT;
            }
        }
    }

    RS485_RX_MODE(bus);

    /* MAX485 方向切换延时 (~20us @ 72MHz) */
    for (volatile uint32_t i = 0; i < 1440; i++) {
        /* nop */
    }

    return AX12A_OK;
}

/**
 * @brief   接收状态包 (逐字节阻塞接收)
 */
static Ax12aStatus recvPacket(Ax12aBus *bus, uint8_t *buf, uint8_t len)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t recvCount = 0;

    while (recvCount < len) {
        if ((HAL_GetTick() - startTick) >= AX12A_TIMEOUT_MS) {
            return AX12A_ERR_TIMEOUT;
        }

        if (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_RXNE) != RESET) {
            buf[recvCount] = (uint8_t)(
                bus->huart->Instance->DR & 0xFF);
            recvCount++;
        }
    }

    return AX12A_OK;
}

/**
 * @brief   校验状态包
 */
static Ax12aStatus verifyStatus(const uint8_t *buf,
                                uint8_t expectedId,
                                uint8_t dataLen)
{
    uint8_t checksum;
    uint8_t sum;
    uint8_t pktLen;

    if (buf[0] != AX12A_HEADER_1 || buf[1] != AX12A_HEADER_2) {
        return AX12A_ERR_CHECKSUM;
    }

    if (buf[2] != expectedId) {
        return AX12A_ERR_CHECKSUM;
    }

    pktLen = AX12A_STATUS_PKT_BASE + dataLen;

    sum = 0;
    for (uint8_t i = 2; i < pktLen - 1; i++) {
        sum += buf[i];
    }
    checksum = ~sum;

    if (checksum != buf[pktLen - 1]) {
        return AX12A_ERR_CHECKSUM;
    }

    if (buf[4] != 0x00) {
        return AX12A_ERR_SERVO;
    }

    return AX12A_OK;
}
