/**
 * @file    ax12a.c
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动实现
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 2.1
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
static Ax12aStatus ax12aWrite(Ax12aBus *bus, uint8_t id, uint16_t addr,
                              uint8_t dataLen, const uint8_t *data);

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

/* ========================= 私有函数实现 ============================ */

/**
 * @brief   向舵机寄存器写入数据 (内部)
 */
static Ax12aStatus ax12aWrite(Ax12aBus *bus, uint8_t id, uint16_t addr,
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
