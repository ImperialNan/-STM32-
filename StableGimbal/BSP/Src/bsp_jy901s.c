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
static UART_HandleTypeDef *s_huart_jy901s = NULL;

void JY901S_Init(JY901S_t *imu, UART_HandleTypeDef *huart)
{
    s_huart_jy901s = huart;

    memset(imu, 0, sizeof(JY901S_t));
    imu->state = JY901S_STATE_WAIT_HEADER;
    imu->dma_head = 0;
}

void JY901S_StartReceive(JY901S_t *imu)
{
    if (s_huart_jy901s != NULL) {
        /* 启动 DMA 循环接收 */
        HAL_UART_Receive_DMA(s_huart_jy901s, imu->dma_buf, JY901S_DMA_BUF_SIZE);
        /* 使能 IDLE 中断 */
        __HAL_UART_ENABLE_IT(s_huart_jy901s, UART_IT_IDLE);
    }
}

void JY901S_IDLE_IRQHandler(JY901S_t *imu)
{
    if (__HAL_UART_GET_FLAG(s_huart_jy901s, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(s_huart_jy901s);

        /* 计算 DMA 写指针位置 */
        uint16_t dma_pos = JY901S_DMA_BUF_SIZE
                         - __HAL_DMA_GET_COUNTER(s_huart_jy901s->hdmarx);

        /* 从 dma_head 到 dma_pos 逐字节送入解析状态机 */
        while (imu->dma_head != dma_pos) {
            JY901S_ProcessByte(imu, imu->dma_buf[imu->dma_head]);
            imu->dma_head = (imu->dma_head + 1) % JY901S_DMA_BUF_SIZE;
        }
    }
}

void JY901S_ProcessByte(JY901S_t *imu, uint8_t data)
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
            imu->frame_type = data;
            imu->rx_index = 2;
            imu->state = JY901S_STATE_RECEIVING_DATA;
        } else if (data == JY901S_FRAME_HEADER) {
            /* 可能是新的帧头，保持在 WAIT_TYPE 状态 */
            imu->buffer[0] = data;
        } else {
            imu->state = JY901S_STATE_WAIT_HEADER;
        }
        break;

    case JY901S_STATE_RECEIVING_DATA:
        imu->buffer[imu->rx_index] = data;
        imu->rx_index++;

        if (imu->rx_index >= JY901S_FRAME_LEN) {
            /* 帧接收完成，进行校验 */
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 10; i++) {
                checksum += imu->buffer[i];
            }

            if (checksum == imu->buffer[10]) {
                /* 校验通过，解析数据（小端序） */
                int16_t raw_data1 = (int16_t)((imu->buffer[3] << 8) | imu->buffer[2]);
                int16_t raw_data2 = (int16_t)((imu->buffer[5] << 8) | imu->buffer[4]);
                int16_t raw_data3 = (int16_t)((imu->buffer[7] << 8) | imu->buffer[6]);

                if (imu->frame_type == JY901S_GYRO_TYPE) {
                    /* 角速度数据：单位 32768 = 2000°/s */
                    imu->wx = (float)raw_data1 / 32768.0f * 2000.0f;
                    imu->wy = (float)raw_data2 / 32768.0f * 2000.0f;
                    imu->wz = (float)raw_data3 / 32768.0f * 2000.0f;
                } else if (imu->frame_type == JY901S_ANGLE_TYPE) {
                    /* 角度数据：单位 32768 = 180° */
                    imu->roll  = (float)raw_data1 / 32768.0f * 180.0f;
                    imu->pitch = (float)raw_data2 / 32768.0f * 180.0f;
                    imu->yaw   = (float)raw_data3 / 32768.0f * 180.0f;
                }

                imu->data_ready = 1;
            }

            /* 无论校验是否通过，都重置状态机 */
            imu->state = JY901S_STATE_WAIT_HEADER;
            imu->rx_index = 0;
        }
        break;

    default:
        imu->state = JY901S_STATE_WAIT_HEADER;
        break;
    }
}

uint8_t JY901S_IsDataReady(JY901S_t *imu)
{
    return imu->data_ready;
}

void JY901S_ClearDataReady(JY901S_t *imu)
{
    imu->data_ready = 0;
}

UART_HandleTypeDef* JY901S_GetUartHandle(void)
{
    return s_huart_jy901s;
}

/* HAL 回调函数已移至 main.c USER CODE 区域 */
