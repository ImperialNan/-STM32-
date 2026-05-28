/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_jy901s.h"
#include "pid_controller.h"
#include "bsp_ax12a.h"
#include "bsp_led.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* 全局实例 */
Jy901s g_jy901s;

/* AX-12A 舵机 */
Ax12a g_ax12aPitch;    /* ID=1, 俯仰 */
Ax12a g_ax12aRoll;     /* ID=2, 横滚 */
Ax12a g_ax12aYaw;      /* ID=3, 航向 */

/* 串级 PID 控制器 (传感器数据内嵌在 CascadePid.sensor 中) */
CascadePid g_cascadePitch;
CascadePid g_cascadeRoll;
CascadePid g_cascadeYaw;

/* printf 重定向到 USART1 */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 控制环标志（由 TIM2 中断置位） */
volatile uint8_t g_controlFlag = 0;

/* 双速率分频 */
#define OUTER_DIV  10   /* 100Hz / 10 = 10Hz 外环 */
#define FB_DIV      5   /* 100Hz / 5  = 20Hz 舵机反馈读取 */

/* 控制周期 (s) */
#define DT_OUTER  0.1f  /* 外环 dt = 100ms */
#define DT_INNER  0.01f /* 内环 dt = 10ms  */

/* 舵机中立位 (AX-12A 角度值) */
#define PITCH_NEUTRAL  240.0f
#define ROLL_NEUTRAL   150.0f
#define YAW_NEUTRAL    150.0f

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* ══════════════════════════════════════════════════════════════ */
  /*  BSP 初始化                                                    */
  /* ══════════════════════════════════════════════════════════════ */

  /* JY901S IMU */
  jy901sInit(&g_jy901s, &huart2);

  /* AX-12A 舵机 */
  ax12aInit(&g_ax12aPitch, &huart3, 1, 0.0f, 300.0f);
  ax12aInit(&g_ax12aRoll,  &huart3, 2, 0.0f, 300.0f);
  ax12aInit(&g_ax12aYaw,   &huart3, 3, 100.0f, 200.0f);

  /* ══════════════════════════════════════════════════════════════ */
  /*  串级 PID 初始化                                               */
  /*  外环 (位置型): 角度 → 角速度指令, 10Hz                         */
  /*  内环 (增量型): 角速度 → 舵机偏移, 100Hz                        */
  /* ══════════════════════════════════════════════════════════════ */
  /*
   *  cascadePidInit(c,
   *      外环 kp, ki(/s), kd, 死区°, absMax(°/s), incMax(°/s), iMax(°/s), dfAlpha,
   *      内环 kp, ki(/s), kd, 死区(°/s), absMax(°), incMax(°), dfAlpha,
   *      kff,  speedK, spdMin, spdMax);
   */

  /* PITCH */
  cascadePidInit(&g_cascadePitch,
      2.0f, 0.5f, 0.8f, 0.3f, 150.0f, 10.0f, 50.0f, 0.3f,
      0.5f, 2.0f, 0.1f, 0.3f,  40.0f,  5.0f,        0.3f,
      0.05f, 2.0f, 50, 1023);

  /* ROLL */
  cascadePidInit(&g_cascadeRoll,
      2.0f, 0.5f, 0.8f, 0.5f, 150.0f, 10.0f, 50.0f, 0.3f,
      0.5f, 2.0f, 0.1f, 0.3f,  40.0f,  5.0f,        0.3f,
      0.05f, 2.0f, 50, 1023);

  /* YAW */
  cascadePidInit(&g_cascadeYaw,
      1.5f, 0.3f, 0.6f, 0.5f, 100.0f,  8.0f, 40.0f, 0.3f,
      0.4f, 2.0f, 0.08f,0.3f,  30.0f,  4.0f,        0.3f,
      0.04f, 2.0f, 50, 1023);

  /* 传感器融合配置 */
  g_cascadePitch.sensor.source     = SENSOR_SRC_FUSION;
  g_cascadePitch.sensor.imuWeight  = 0.7f;

  g_cascadeRoll.sensor.source      = SENSOR_SRC_FUSION;
  g_cascadeRoll.sensor.imuWeight   = 0.7f;

  g_cascadeYaw.sensor.source       = SENSOR_SRC_FUSION;
  g_cascadeYaw.sensor.imuWeight    = 0.6f;

  /* 启动 IMU DMA+IDLE 接收 */
  jy901sStartReceive(&g_jy901s);

  /* 启动 TIM2 定时器中断（100Hz 控制节拍） */
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_Delay(500);
  LED_ON();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_controlFlag) {
      g_controlFlag = 0;

      /* ═══════════════════════════════════════════════════════ */
      /*  舵机反馈读取 (非阻塞 DMA+IDLE, 降频至 20Hz)            */
      /* ═══════════════════════════════════════════════════════ */
      static uint8_t fbCnt = 0;
      static uint8_t fbPending = 0;

      if (!fbPending && ++fbCnt >= FB_DIV) {
          fbCnt = 0;
          Ax12a *servos[] = {&g_ax12aPitch, &g_ax12aRoll, &g_ax12aYaw};
          ax12aStartFeedbackRead(servos, 3);
          fbPending = 1;
      }

      if (fbPending &ax12aIsFeedbackDone()) {
          fbPending = 0;
      }

      if (fbPending) {
          ax12aFeedbackPoll();
      }

      /* ═══════════════════════════════════════════════════════ */
      /*  填充传感器原始数据 (关中断防止 IMU 数据竞态)            */
      /* ═══════════════════════════════════════════════════════ */
      __disable_irq();
      float imuPitch  = g_jy901s.pitch;
      float imuRoll   = g_jy901s.roll;
      float imuYaw    = g_jy901s.yaw;
      float imuWx     = g_jy901s.wx;
      float imuWy     = g_jy901s.wy;
      float imuWz     = g_jy901s.wz;
      __enable_irq();

      g_cascadePitch.sensor.imuAngle    = imuPitch;
      g_cascadePitch.sensor.imuW        = imuWx;
      g_cascadePitch.sensor.ax12aAngle  = g_ax12aPitch.feedback.angleDeg;
      g_cascadePitch.sensor.ax12aSpeed  = g_ax12aPitch.feedback.speedDps;
      g_cascadePitch.sensor.ax12aValid  = g_ax12aPitch.feedback.dataValid;

      g_cascadeRoll.sensor.imuAngle     = imuRoll;
      g_cascadeRoll.sensor.imuW         = imuWy;
      g_cascadeRoll.sensor.ax12aAngle   = g_ax12aRoll.feedback.angleDeg;
      g_cascadeRoll.sensor.ax12aSpeed   = g_ax12aRoll.feedback.speedDps;
      g_cascadeRoll.sensor.ax12aValid   = g_ax12aRoll.feedback.dataValid;

      g_cascadeYaw.sensor.imuAngle      = imuYaw;
      g_cascadeYaw.sensor.imuW          = imuWz;
      g_cascadeYaw.sensor.ax12aAngle    = g_ax12aYaw.feedback.angleDeg;
      g_cascadeYaw.sensor.ax12aSpeed    = g_ax12aYaw.feedback.speedDps;
      g_cascadeYaw.sensor.ax12aValid    = g_ax12aYaw.feedback.dataValid;

      /* ═══════════════════════════════════════════════════════ */
      /*  双速率串级 PID 控制                                     */
      /*  外环: 10Hz | 内环: 100Hz                                */
      /* ═══════════════════════════════════════════════════════ */
      static uint8_t outerDivCnt = 0;

      if (++outerDivCnt >= OUTER_DIV) {
          outerDivCnt = 0;

          /* 外环 PID (位置型, 10Hz) */
          cascadePidComputeOuter(&g_cascadePitch,  0.0f,   DT_OUTER);
          cascadePidComputeOuter(&g_cascadeRoll,   0.0f,   DT_OUTER);
          cascadePidComputeOuter(&g_cascadeYaw,  -143.0f,  DT_OUTER);
      }

      /* 内环 PID (增量型, 100Hz) */
      float pitchOff = cascadePidComputeInner(&g_cascadePitch, DT_INNER);
      float rollOff  = cascadePidComputeInner(&g_cascadeRoll,  DT_INNER);
      float yawOff   = cascadePidComputeInner(&g_cascadeYaw,   DT_INNER);

      /* ═══════════════════════════════════════════════════════ */
      /*  输出到舵机 (SyncWrite 批量下发)                         */
      /* ═══════════════════════════════════════════════════════ */
      {
          uint8_t  ids[3]  = {1, 2, 3};
          uint16_t pos[3]  = {
              (uint16_t)((PITCH_NEUTRAL + pitchOff) * AX12A_DEG_TO_POS),
              (uint16_t)((ROLL_NEUTRAL  + rollOff)  * AX12A_DEG_TO_POS),
              (uint16_t)((YAW_NEUTRAL   - yawOff)   * AX12A_DEG_TO_POS)
          };
          uint16_t spd[3]  = {
              g_cascadePitch.servoSpeed,
              g_cascadeRoll.servoSpeed,
              g_cascadeYaw.servoSpeed
          };
          ax12aSyncWrite(&huart3, ids, pos, spd, 3);
      }

      /* ═══════════════════════════════════════════════════════ */
      /*  调试输出 (每 10 帧, 约 10Hz)，release可删除             */
      /* ═══════════════════════════════════════════════════════ */
      static uint16_t dbgCnt = 0;
      if (++dbgCnt >= 10) {
          dbgCnt = 0;
          printf("P:%.1f/%.1f R:%.1f/%.1f Y:%.1f/%.1f"
                 " | off:%+.1f %+.1f %+.1f"
                 " | sat_o:%d%d%d sat_i:%d%d%d\r\n",
                 g_cascadePitch.sensor.fusedAngle, imuPitch,
                 g_cascadeRoll.sensor.fusedAngle,  imuRoll,
                 g_cascadeYaw.sensor.fusedAngle,   imuYaw,
                 pitchOff, rollOff, yawOff,
                 g_cascadePitch.outerSat, g_cascadeRoll.outerSat, g_cascadeYaw.outerSat,
                 g_cascadePitch.innerSat, g_cascadeRoll.innerSat, g_cascadeYaw.innerSat);
      }

      LED_Toggle();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  TIM2 更新中断回调（100Hz）
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        g_controlFlag = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
