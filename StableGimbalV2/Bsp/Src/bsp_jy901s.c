/**
  ******************************************************************************
  * @file    bsp_jy901s.c
  * @brief   JY901S 九轴 IMU 模块驱动（DMA 循环 + IDLE 中断模式）
  * @note    ISR 仅操作状态标志，解析在 jy901sPoll() 中由主循环完成
  ******************************************************************************
  */
#include "bsp_jy901s.h"
#include <string.h>

/* ========================== 模块级静态变量 ========================== */
static UART_HandleTypeDef *sHuart = NULL;

/* ========================== 内部函数声明 ========================== */
static void processByte(Jy901s *imu, uint8_t data);

/* ========================== 公开 API 实现 ========================== */

/**
  * @brief  初始化 JY901S 句柄
  */
void jy901sInit(Jy901s *imu, UART_HandleTypeDef *huart)
{
    sHuart = huart;
    memset(imu, 0, sizeof(Jy901s));
    imu->state = JY901S_STATE_WAIT_HEADER;
}

/**
  * @brief  启动 DMA 循环接收 + IDLE 中断
  */
void jy901sStartReceive(Jy901s *imu)
{
    if (sHuart == NULL) {
        return;
    }
    (void)imu;
    HAL_UART_Receive_DMA(sHuart, imu->dmaBuf, JY901S_DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(sHuart, UART_IT_IDLE);
}

/**
  * @brief  USART2 IDLE 中断回调（ISR 上下文）
  * @note   STM32F1 清零 IDLE 需读 SR+DR，在 DMA 模式下读 DR
  *         可能抢走 DMA 未传输的字节。故先暂停 DMA 通道，
  *         清零 IDLE 后记录 CNDTR，再恢复 DMA。
  */
void jy901sOnIdleIrq(Jy901s *imu)
{
    if (__HAL_UART_GET_FLAG(sHuart, UART_FLAG_IDLE) != RESET) {
        /* 暂停 DMA 通道，防止读 DR 清零 IDLE 时干扰 DMA */
        __HAL_DMA_DISABLE(sHuart->hdmarx);

        /* 清零 IDLE：读 SR 再读 DR */
        __HAL_UART_CLEAR_IDLEFLAG(sHuart);

        /* 记录 DMA 当前写入位置（DMA 已暂停，CNDTR 稳定） */
        imu->dmaPos = JY901S_DMA_BUF_SIZE
                    - (uint16_t)__HAL_DMA_GET_COUNTER(sHuart->hdmarx);

        /* 恢复 DMA 继续接收 */
        __HAL_DMA_ENABLE(sHuart->hdmarx);

        imu->idleFlag = 1;
    }
}

/**
  * @brief  主循环轮询：消费环形缓冲区中的字节
  * @note   由 idleFlag 驱动，使用 ISR 中记录的 dmaPos
  */
void jy901sPoll(Jy901s *imu)
{
    uint16_t head;
    uint16_t pos;

    if (imu->idleFlag == 0) {
        return;
    }

    head = imu->dmaHead;
    pos  = imu->dmaPos;

    /* 逐字节消费 */
    while (head != pos) {
        processByte(imu, imu->dmaBuf[head]);
        head = (head + 1U) & (JY901S_DMA_BUF_SIZE - 1U);
    }

    imu->dmaHead = head;
    imu->idleFlag = 0;
}

/**
  * @brief  检查是否有新解析完成的帧
  */
uint8_t jy901sIsDataReady(const Jy901s *imu)
{
    return imu->dataReady;
}

/**
  * @brief  清除数据就绪标志
  */
void jy901sClearDataReady(Jy901s *imu)
{
    imu->dataReady = 0;
}

/* ========================== 内部函数实现 ========================== */

/**
  * @brief  单字节状态机解析
  * @note   不依赖 ISR，可从主循环安全调用
  */
static void processByte(Jy901s *imu, uint8_t data)
{
    switch (imu->state) {
    case JY901S_STATE_WAIT_HEADER:
        if (data == JY901S_FRAME_HEADER) {
            imu->frameBuf[0] = data;
            imu->state = JY901S_STATE_WAIT_TYPE;
        }
        break;

    case JY901S_STATE_WAIT_TYPE:
        if (data == JY901S_GYRO_TYPE || data == JY901S_ANGLE_TYPE) {
            imu->frameBuf[1] = data;
            imu->frameType = data;
            imu->frameIndex = 2;
            imu->state = JY901S_STATE_RECEIVING_DATA;
        } else if (data == JY901S_FRAME_HEADER) {
            /* 可能是新的帧头，保持在 WAIT_TYPE */
            imu->frameBuf[0] = data;
        } else {
            imu->state = JY901S_STATE_WAIT_HEADER;
        }
        break;

    case JY901S_STATE_RECEIVING_DATA:
        imu->frameBuf[imu->frameIndex] = data;
        imu->frameIndex++;

        if (imu->frameIndex >= JY901S_FRAME_LEN) {
            /* 帧接收完成，校验 */
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 10; i++) {
                checksum += imu->frameBuf[i];
            }

            if (checksum == imu->frameBuf[10]) {
                /* 解析小端序 int16 */
                int16_t raw1 = (int16_t)(((uint16_t)imu->frameBuf[3] << 8)
                                         | imu->frameBuf[2]);
                int16_t raw2 = (int16_t)(((uint16_t)imu->frameBuf[5] << 8)
                                         | imu->frameBuf[4]);
                int16_t raw3 = (int16_t)(((uint16_t)imu->frameBuf[7] << 8)
                                         | imu->frameBuf[6]);

                if (imu->frameType == JY901S_GYRO_TYPE) {
                    /* 量程 32768 = 2000°/s */
                    imu->wx = (float)raw1 / 32768.0f * 2000.0f;
                    imu->wy = (float)raw2 / 32768.0f * 2000.0f;
                    imu->wz = (float)raw3 / 32768.0f * 2000.0f;
                } else {
                    /* 量程 32768 = 180° */
                    imu->roll  = (float)raw1 / 32768.0f * 180.0f;
                    imu->pitch = (float)raw2 / 32768.0f * 180.0f;
                    imu->yaw   = (float)raw3 / 32768.0f * 180.0f;
                }
                imu->dataReady = 1;
            }

            /* 无论校验是否通过，重置状态机 */
            imu->state = JY901S_STATE_WAIT_HEADER;
            imu->frameIndex = 0;
        }
        break;

    default:
        imu->state = JY901S_STATE_WAIT_HEADER;
        break;
    }
}
