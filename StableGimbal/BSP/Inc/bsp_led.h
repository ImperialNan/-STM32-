/**
  ******************************************************************************
  * @file    bsp_led.h
  * @brief   LED 控制模块
  ******************************************************************************
  */
#ifndef __BSP_LED_H
#define __BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* LED 引脚定义（PB8，开漏输出） */
#define LED_GPIO_PORT   GPIOB
#define LED_GPIO_PIN    GPIO_PIN_8

#define LED_ON()    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET)
#define LED_Toggle() HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN)

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H */
