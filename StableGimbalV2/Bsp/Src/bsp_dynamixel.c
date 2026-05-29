/**
 ******************************************************************************
 * @file    bsp_dynamixel.c
 * @brief   Dynamixel Protocol 1.0 舵机总线驱动实现
 * @note    RS485 半双工, USART3 @ 1Mbps, 阻塞收发
 * @version 1.0
 ******************************************************************************
 */

#include "bsp_dynamixel.h"

/* Private macros -----------------------------------------------------------*/

/* RS485 方向控制：高电平 = 发送, 低电平 = 接收 */
#define DYN_RS485_TX_MODE(bus) \
    HAL_GPIO_WritePin((bus)->txEnPort, (bus)->txEnPin, GPIO_PIN_SET)
#define DYN_RS485_RX_MODE(bus) \
    HAL_GPIO_WritePin((bus)->txEnPort, (bus)->txEnPin, GPIO_PIN_RESET)

/* 校验和：~ (ID + Length + Instruction + Param0 + ... + ParamN) */
#define DYN_CHECKSUM_CALC(pkt, start, end) \
    do { \
        uint8_t _sum = 0; \
        for (uint8_t _i = (start); _i <= (end); _i++) { \
            _sum += (pkt)[_i]; \
        } \
        (pkt)[(end) + 1] = ~_sum; \
    } while (0)

/* Private function prototypes ----------------------------------------------*/

/**
 * @brief  发送指令包
 * @param  bus: 总线句柄指针
 * @param  pkt: 指令包缓冲区
 * @param  len: 指令包总长度
 * @return DYN_OK 或 DYN_ERR_TIMEOUT
 */
static DynStatus dynSendPacket(DynamixelBus *bus, const uint8_t *pkt,
                               uint8_t len);

/**
 * @brief  接收状态包
 * @param  bus: 总线句柄指针
 * @param  buf: 接收缓冲区
 * @param  len: 期望接收的字节数
 * @return DYN_OK 或 DYN_ERR_TIMEOUT
 */
static DynStatus dynRecvPacket(DynamixelBus *bus, uint8_t *buf, uint8_t len);

/**
 * @brief  校验状态包
 * @param  buf: 接收缓冲区 (长度 >= DYN_STATUS_PKT_BASE + dataLen)
 * @param  expectedId: 期望的舵机 ID
 * @param  dataLen: 数据区长度
 * @return DYN_OK / DYN_ERR_CHECKSUM / DYN_ERR_SERVO
 */
static DynStatus dynVerifyStatus(const uint8_t *buf, uint8_t expectedId,
                                 uint8_t dataLen);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  初始化总线句柄
 */
void dynInit(DynamixelBus *bus, UART_HandleTypeDef *huart,
             GPIO_TypeDef *txEnPort, uint16_t txEnPin)
{
    bus->huart = huart;
    bus->txEnPort = txEnPort;
    bus->txEnPin = txEnPin;
    DYN_RS485_RX_MODE(bus); /* 默认接收模式 */
}

/**
 * @brief  从舵机寄存器读取数据
 */
DynStatus dynRead(DynamixelBus *bus, uint8_t id, uint16_t addr,
                  uint8_t dataLen, uint8_t *data)
{
    uint8_t cmdPkt[8];
    uint8_t rspPkt[16]; /* 足够容纳常规回包 */
    uint8_t rspLen;
    DynStatus status;

    if (bus == NULL || data == NULL || dataLen == 0) {
        return DYN_ERR_PARAM;
    }

    /* 构建 READ 指令包: [0xFF, 0xFF, ID, 0x04, 0x02, Addr, DataLen, CKSM] */
    cmdPkt[0] = DYN_HEADER_1;
    cmdPkt[1] = DYN_HEADER_2;
    cmdPkt[2] = id;
    cmdPkt[3] = 0x04;                    /* Length */
    cmdPkt[4] = DYN_INST_READ;           /* READ */
    cmdPkt[5] = (uint8_t)(addr & 0xFF);  /* 起始地址低字节 */
    cmdPkt[6] = dataLen;                 /* 读取字节数 */
    DYN_CHECKSUM_CALC(cmdPkt, 2, 6);

    /* 发送指令包 */
    status = dynSendPacket(bus, cmdPkt, sizeof(cmdPkt));
    if (status != DYN_OK) {
        return status;
    }

    /* 接收状态包: 头(2) + ID(1) + Length(1) + Error(1) + Data(N) + CKSM(1) */
    rspLen = DYN_STATUS_PKT_BASE + dataLen;
    status = dynRecvPacket(bus, rspPkt, rspLen);
    if (status != DYN_OK) {
        return status;
    }

    /* 校验状态包 */
    status = dynVerifyStatus(rspPkt, id, dataLen);
    if (status != DYN_OK) {
        return status;
    }

    /* 复制数据 */
    for (uint8_t i = 0; i < dataLen; i++) {
        data[i] = rspPkt[5 + i];
    }

    return DYN_OK;
}

/**
 * @brief  向舵机寄存器写入数据
 */
DynStatus dynWrite(DynamixelBus *bus, uint8_t id, uint16_t addr,
                   uint8_t dataLen, const uint8_t *data)
{
    uint8_t cmdPkt[16]; /* 写指令最多 8 + dataLen 字节 */
    uint8_t cmdLen;

    if (bus == NULL || data == NULL || dataLen == 0) {
        return DYN_ERR_PARAM;
    }

    /* 构建 WRITE 指令包:
     * [0xFF, 0xFF, ID, Length, 0x03, Addr, Data0...DataN, CKSM]
     * Length = (1 + dataLen) + 2 = dataLen + 3, 其中 1=地址, +2 是协议要求 */
    cmdPkt[0] = DYN_HEADER_1;
    cmdPkt[1] = DYN_HEADER_2;
    cmdPkt[2] = id;
    cmdPkt[3] = dataLen + 3;             /* Length */
    cmdPkt[4] = DYN_INST_WRITE;          /* WRITE */
    cmdPkt[5] = (uint8_t)(addr & 0xFF);  /* 起始地址低字节 */
    for (uint8_t i = 0; i < dataLen; i++) {
        cmdPkt[6 + i] = data[i];
    }
    cmdLen = 6 + dataLen;                /* 不含校验的指令长度 */
    DYN_CHECKSUM_CALC(cmdPkt, 2, cmdLen - 1);

    return dynSendPacket(bus, cmdPkt, cmdLen + 1);
}

/**
 * @brief  读取舵机当前位置
 */
DynStatus dynGetPosition(DynamixelBus *bus, uint8_t id, uint16_t *pos)
{
    uint8_t buf[2];
    DynStatus status;

    if (pos == NULL) {
        return DYN_ERR_PARAM;
    }

    status = dynRead(bus, id, DYN_ADDR_PRESENT_POSITION, 2, buf);
    if (status != DYN_OK) {
        *pos = 0;
        return status;
    }

    *pos = ((uint16_t)buf[1] << 8) | buf[0];
    return DYN_OK;
}

/**
 * @brief  读取舵机当前速度
 */
DynStatus dynGetSpeed(DynamixelBus *bus, uint8_t id, uint16_t *spd)
{
    uint8_t buf[2];
    DynStatus status;

    if (spd == NULL) {
        return DYN_ERR_PARAM;
    }

    status = dynRead(bus, id, DYN_ADDR_PRESENT_SPEED, 2, buf);
    if (status != DYN_OK) {
        *spd = 0;
        return status;
    }

    *spd = ((uint16_t)buf[1] << 8) | buf[0];
    return DYN_OK;
}

/**
 * @brief  设置舵机目标位置
 */
DynStatus dynSetGoalPosition(DynamixelBus *bus, uint8_t id, uint16_t pos)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(pos & 0xFF);
    buf[1] = (uint8_t)(pos >> 8);

    return dynWrite(bus, id, DYN_ADDR_GOAL_POSITION, 2, buf);
}

/**
 * @brief  设置舵机移动速度
 */
DynStatus dynSetMovingSpeed(DynamixelBus *bus, uint8_t id, uint16_t spd)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(spd & 0xFF);
    buf[1] = (uint8_t)(spd >> 8);

    return dynWrite(bus, id, DYN_ADDR_MOVING_SPEED, 2, buf);
}

/**
 * @brief  同步写入三个舵机的目标位置和速度 (SYNC Write)
 */
DynStatus dynSyncWritePosSpeed(DynamixelBus *bus,
                                uint8_t id1, uint16_t pos1, uint16_t spd1,
                                uint8_t id2, uint16_t pos2, uint16_t spd2,
                                uint8_t id3, uint16_t pos3, uint16_t spd3)
{
    /*
     * SYNC Write 指令包结构:
     * [0xFF, 0xFF, 0xFE, Length, 0x83, StartAddr, DataPerServo,
     *  ID1, Data1_0...Data1_N, ID2, Data2_0...Data2_N, ID3, Data3_0...Data3_N,
     *  CKSM]
     *
     * 每个舵机数据块: ID(1) + Position(2) + Speed(2) = 5 字节
     * Length = 2 (StartAddr + DataPerServo) + 3 * 5 (三个舵机数据) + 2
     *        = 2 + 15 + 2 = 19 = 0x13
     */
    uint8_t cmdPkt[23];

    cmdPkt[0] = DYN_HEADER_1;
    cmdPkt[1] = DYN_HEADER_2;
    cmdPkt[2] = DYN_BROADCAST_ID;
    cmdPkt[3] = 0x13;                       /* Length = 19 */
    cmdPkt[4] = DYN_INST_SYNC_WRITE;        /* SYNC Write */
    cmdPkt[5] = DYN_ADDR_GOAL_POSITION;     /* 起始地址 */
    cmdPkt[6] = 0x04;                       /* 每舵机数据长度: pos(2)+spd(2) */

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

    DYN_CHECKSUM_CALC(cmdPkt, 2, 21);

    return dynSendPacket(bus, cmdPkt, sizeof(cmdPkt));
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  发送指令包
 */
static DynStatus dynSendPacket(DynamixelBus *bus, const uint8_t *pkt,
                               uint8_t len)
{
    HAL_StatusTypeDef halStatus;

    /* 清除残留接收数据，避免干扰 */
    __HAL_UART_CLEAR_OREFLAG(bus->huart);

    DYN_RS485_TX_MODE(bus);

    halStatus = HAL_UART_Transmit(bus->huart, (uint8_t *)pkt, len,
                                  DYN_TIMEOUT_MS);
    if (halStatus != HAL_OK) {
        DYN_RS485_RX_MODE(bus);
        return DYN_ERR_TIMEOUT;
    }

    /* 等待发送完成 (TC: Transmission Complete) */
    while (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_TC) == RESET) {
        /* wait */
    }

    DYN_RS485_RX_MODE(bus);

    /* 给 MAX485 切换方向留一小段延时 (~5us) */
    for (volatile uint32_t i = 0; i < 360; i++) {
        /* 72MHz 下约 5us */
    }

    return DYN_OK;
}

/**
 * @brief  接收状态包（逐字节接收，带超时）
 * @note   Dynamixel 回包字节间间隔极短，逐字节接收比定长更可靠
 */
static DynStatus dynRecvPacket(DynamixelBus *bus, uint8_t *buf, uint8_t len)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t recvCount = 0;

    while (recvCount < len) {
        if ((HAL_GetTick() - startTick) >= DYN_TIMEOUT_MS) {
            return DYN_ERR_TIMEOUT;
        }

        if (__HAL_UART_GET_FLAG(bus->huart, UART_FLAG_RXNE) != RESET) {
            buf[recvCount] = (uint8_t)(bus->huart->Instance->DR & 0xFF);
            recvCount++;
        }
    }

    return DYN_OK;
}

/**
 * @brief  校验状态包
 */
static DynStatus dynVerifyStatus(const uint8_t *buf, uint8_t expectedId,
                                 uint8_t dataLen)
{
    uint8_t checksum;
    uint8_t sum;
    uint8_t pktLen;

    /* 检查帧头 */
    if (buf[0] != DYN_HEADER_1 || buf[1] != DYN_HEADER_2) {
        return DYN_ERR_CHECKSUM;
    }

    /* 检查 ID */
    if (buf[2] != expectedId) {
        return DYN_ERR_CHECKSUM;
    }

    pktLen = DYN_STATUS_PKT_BASE + dataLen;

    /* 校验和: ~(ID + Length + Error + Data[0..N-1]) */
    sum = 0;
    for (uint8_t i = 2; i < pktLen - 1; i++) {
        sum += buf[i];
    }
    checksum = ~sum;

    if (checksum != buf[pktLen - 1]) {
        return DYN_ERR_CHECKSUM;
    }

    /* 检查舵机错误位 (buf[4]) */
    if (buf[4] != 0x00) {
        return DYN_ERR_SERVO;
    }

    return DYN_OK;
}
