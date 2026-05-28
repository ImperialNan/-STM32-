/**
  ******************************************************************************
  * @file    bsp_jy901s.c
  * @brief   JY901S 九轴 IMU 模块驱动实现（DMA + IDLE 中断模式）
  ******************************************************************************
  */
#include "bsp_jy901s.h"
#include "main.h"
#include <string.h>

/* 模块级 UART 句柄 */
static UART_HandleTypeDef *s_huart = NULL;

void jy901sInit(Jy901s *imu, UART_HandleTypeDef *huart)
{
    s_huart = huart;

    memset(imu, 0, sizeof(Jy901s));
    imu->state = JY901S_STATE_WAIT_HEADER;
    imu->dmaHead = 0;
}

void jy901sStartReceive(Jy901s *imu)
{
    if (s_huart != NULL) {
        /* 启动 DMA 循环接收 */
        HAL_UART_Receive_DMA(s_huart, imu->dmaBuf, JY901S_DMA_BUF_SIZE);
        /* 使能 IDLE 中断 */
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
    }
}

void jy901sIdleIrqHandler(Jy901s *imu)
{
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(s_huart);

        /* 计算 DMA 写指针位置 */
        uint16_t dmaPos = JY901S_DMA_BUF_SIZE
                        - __HAL_DMA_GET_COUNTER(s_huart->hdmarx);

        /* 从 dmaHead 到 dmaPos 逐字节送入解析状态机 */
        while (imu->dmaHead != dmaPos) {
            jy901sProcessByte(imu, imu->dmaBuf[imu->dmaHead]);
            imu->dmaHead = (imu->dmaHead + 1) % JY901S_DMA_BUF_SIZE;
        }
    }
}

void jy901sProcessByte(Jy901s *imu, uint8_t data)
{
    switch (imu->state) {
    case JY901S_STATE_WAIT_HEADER:
        if (data == JY901S_FRAME_HEADER) {
            imu->buffer[0] = data;
            imu->state = JY901S_STATE_WAIT_TYPE;
        }
        break;

    case JY901S_STATE_WAIT_TYPE:
        if (data == JY901S_GYRO_TYPE || data == JY901S_ANGLE_TYPE) {
            imu->buffer[1] = data;
            imu->frameType = data;
            imu->rxIndex = 2;
            imu->state = JY901S_STATE_RECEIVING_DATA;
        } else if (data == JY901S_FRAME_HEADER) {
            /* 可能是新的帧头，保持在 WAIT_TYPE 状态 */
            imu->buffer[0] = data;
        } else {
            imu->state = JY901S_STATE_WAIT_HEADER;
        }
        break;

    case JY901S_STATE_RECEIVING_DATA:
        imu->buffer[imu->rxIndex] = data;
        imu->rxIndex++;

        if (imu->rxIndex >= JY901S_FRAME_LEN) {
            /* 帧接收完成，进行校验 */
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 10; i++) {
                checksum += imu->buffer[i];
            }

            if (checksum == imu->buffer[10]) {
                /* 校验通过，解析数据（小端序） */
                int16_t rawData1 = (int16_t)((imu->buffer[3] << 8) | imu->buffer[2]);
                int16_t rawData2 = (int16_t)((imu->buffer[5] << 8) | imu->buffer[4]);
                int16_t rawData3 = (int16_t)((imu->buffer[7] << 8) | imu->buffer[6]);

                if (imu->frameType == JY901S_GYRO_TYPE) {
                    /* 角速度数据：单位 32768 = 2000°/s */
                    imu->wx = (float)rawData1 / 32768.0f * 2000.0f;
                    imu->wy = (float)rawData2 / 32768.0f * 2000.0f;
                    imu->wz = (float)rawData3 / 32768.0f * 2000.0f;
                } else if (imu->frameType == JY901S_ANGLE_TYPE) {
                    /* 角度数据：单位 32768 = 180° */
                    imu->roll  = (float)rawData1 / 32768.0f * 180.0f;
                    imu->pitch = (float)rawData2 / 32768.0f * 180.0f;
                    imu->yaw   = (float)rawData3 / 32768.0f * 180.0f;
                }

                imu->dataReady = 1;
            }

            /* 无论校验是否通过，都重置状态机 */
            imu->state = JY901S_STATE_WAIT_HEADER;
            imu->rxIndex = 0;
        }
        break;

    default:
        imu->state = JY901S_STATE_WAIT_HEADER;
        break;
    }
}

uint8_t jy901sIsDataReady(Jy901s *imu)
{
    return imu->dataReady;
}

void jy901sClearDataReady(Jy901s *imu)
{
    imu->dataReady = 0;
}

UART_HandleTypeDef* jy901sGetUartHandle(void)
{
    return s_huart;
}

/* HAL 回调函数已移至 main.c USER CODE 区域 */
