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

/* USER CODE BEGIN PV */
/* 全局实例 */
JY901S_t g_jy901s;

/* AX-12A 舵机（统一驱动：执行控制 + 反馈读取） */
AX12A_t g_ax12a_pitch;        /* ID=1, 俯仰 */
AX12A_t g_ax12a_roll;         /* ID=2, 横滚 */
AX12A_t g_ax12a_yaw;          /* ID=3, 航向 */

/* 串级 PID 控制器 */
CascadePID_t g_cascade_pitch;
CascadePID_t g_cascade_roll;
CascadePID_t g_cascade_yaw;

/* 传感器反馈数据 */
SensorFeedback_t g_feedback_pitch;
SensorFeedback_t g_feedback_roll;
SensorFeedback_t g_feedback_yaw;

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
volatile uint8_t g_control_flag = 0;

/* 外环分频计数器（100Hz / 10 = 10Hz 外环） */
#define OUTER_LOOP_DIVIDER  10

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
  JY901S_Init(&g_jy901s, &huart2);

  /* AX-12A 舵机（统一驱动：同时具备执行控制和反馈读取能力） */
  AX12A_Init(&g_ax12a_pitch, &huart3, 1, 0.0f, 300.0f);
  AX12A_Init(&g_ax12a_roll,  &huart3, 2, 0.0f, 300.0f);
  AX12A_Init(&g_ax12a_yaw,   &huart3, 3, 100.0f, 200.0f);

  /* ══════════════════════════════════════════════════════════════ */
  /*  串级 PID 初始化                                               */
  /*  外环: 角度环 (目标角度 → 角速度指令)                           */
  /*  内环: 角速度环 (角速度指令 → 舵机偏移)                         */
  /* ══════════════════════════════════════════════════════════════ */

  /*                        外环(Kp,Ki,Kd)      内环(Kp,Ki,Kd)    */
  CascadePID_Init(&g_cascade_pitch, 2.0f, 0.05f, 0.8f,  0.5f, 0.02f, 0.1f);
  CascadePID_Init(&g_cascade_roll,  2.0f, 0.05f, 0.8f,  0.5f, 0.02f, 0.1f);
  CascadePID_Init(&g_cascade_yaw,   1.5f, 0.03f, 0.6f,  0.4f, 0.02f, 0.08f);

  /* 限幅: 外环(死区°, 增量限幅°/s, 输出绝对值限幅°/s)  内环(死区°/s, 增量限幅°, 输出绝对值限幅°) */
  CascadePID_SetLimits(&g_cascade_pitch, 0.3f, 10.0f, 150.0f,  2.0f,  5.0f, 40.0f);
  CascadePID_SetLimits(&g_cascade_roll,  0.5f, 10.0f, 150.0f,  2.0f,  5.0f, 40.0f);
  CascadePID_SetLimits(&g_cascade_yaw,   0.5f,  8.0f, 100.0f,  2.0f,  4.0f, 30.0f);

  /* 内环微分滤波（抑制舵机反馈噪声） */
  PID_SetDerivativeFilter(&g_cascade_pitch.inner, 0.3f);
  PID_SetDerivativeFilter(&g_cascade_roll.inner,  0.3f);
  PID_SetDerivativeFilter(&g_cascade_yaw.inner,   0.3f);

  /* 传感器融合配置 */
  g_feedback_pitch.source     = SENSOR_SOURCE_FUSION;
  g_feedback_pitch.imu_weight = 0.7f;

  g_feedback_roll.source      = SENSOR_SOURCE_FUSION;
  g_feedback_roll.imu_weight  = 0.7f;

  g_feedback_yaw.source       = SENSOR_SOURCE_FUSION;
  g_feedback_yaw.imu_weight   = 0.6f;

  /* 启动 IMU DMA+IDLE 接收 */
  JY901S_StartReceive(&g_jy901s);

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
    if (g_control_flag) {
      g_control_flag = 0;

      /* 读取 AX-12A 反馈 */
      AX12A_ReadFeedback(&g_ax12a_pitch);
      AX12A_ReadFeedback(&g_ax12a_roll);
      AX12A_ReadFeedback(&g_ax12a_yaw);

      /* 填充传感器反馈 */
      g_feedback_pitch.imu_angle       = g_jy901s.pitch;
      g_feedback_pitch.imu_angular_vel = g_jy901s.wx;
      g_feedback_pitch.ax12a_angle     = g_ax12a_pitch.feedback.angle_deg;
      g_feedback_pitch.ax12a_speed     = g_ax12a_pitch.feedback.speed_dps;
      g_feedback_pitch.ax12a_valid     = g_ax12a_pitch.feedback.data_valid;

      g_feedback_roll.imu_angle       = g_jy901s.roll;
      g_feedback_roll.imu_angular_vel = g_jy901s.wy;
      g_feedback_roll.ax12a_angle     = g_ax12a_roll.feedback.angle_deg;
      g_feedback_roll.ax12a_speed     = g_ax12a_roll.feedback.speed_dps;
      g_feedback_roll.ax12a_valid     = g_ax12a_roll.feedback.data_valid;

      g_feedback_yaw.imu_angle       = g_jy901s.yaw;
      g_feedback_yaw.imu_angular_vel = g_jy901s.wz;
      g_feedback_yaw.ax12a_angle     = g_ax12a_yaw.feedback.angle_deg;
      g_feedback_yaw.ax12a_speed     = g_ax12a_yaw.feedback.speed_dps;
      g_feedback_yaw.ax12a_valid     = g_ax12a_yaw.feedback.data_valid;

      /* ═══════════════════════════════════════════════════════ */
      /*  双速率串级 PID 控制                                     */
      /*  外环: 10Hz (每 OUTER_LOOP_DIVIDER 次内环执行一次)       */
      /*  内环: 100Hz (每次 TIM2 中断都执行)                      */
      /* ═══════════════════════════════════════════════════════ */
      static uint8_t outer_div_cnt = 0;

      if (++outer_div_cnt >= OUTER_LOOP_DIVIDER) {
          outer_div_cnt = 0;

          /* 外环 PID（角度环，10Hz） */
          CascadePID_ComputeOuter(&g_cascade_pitch, 0.0f,   &g_feedback_pitch);
          CascadePID_ComputeOuter(&g_cascade_roll,  0.0f,   &g_feedback_roll);
          CascadePID_ComputeOuter(&g_cascade_yaw,  -143.0f, &g_feedback_yaw);
      }

      /* 内环 PID（角速度环，100Hz） */
      float pitch_offset = CascadePID_ComputeInner(&g_cascade_pitch, &g_feedback_pitch);
      float roll_offset  = CascadePID_ComputeInner(&g_cascade_roll,  &g_feedback_roll);
      float yaw_offset   = CascadePID_ComputeInner(&g_cascade_yaw,   &g_feedback_yaw);

      /* 输出到舵机（SyncWrite 批量下发，速度由 PID 动态计算） */
      {
          uint8_t  ids[3]       = {1, 2, 3};
          uint16_t positions[3] = {
              (uint16_t)((240.0f + pitch_offset) * AX12A_DEG_TO_POS),
              (uint16_t)((150.0f + roll_offset)  * AX12A_DEG_TO_POS),
              (uint16_t)((150.0f - yaw_offset)   * AX12A_DEG_TO_POS)
          };
          uint16_t speeds[3] = {
              g_cascade_pitch.servo_speed,
              g_cascade_roll.servo_speed,
              g_cascade_yaw.servo_speed
          };
          AX12A_SyncWrite(&huart3, ids, positions, speeds, 3);
      }

      /* 调试输出（每 10 帧打印一次） */
      static uint16_t print_div = 0;
      if (++print_div >= 10) {
          print_div = 0;
          printf("P:%6.1f/%6.1f R:%6.1f/%6.1f Y:%6.1f/%6.1f | out: %5.1f %5.1f %5.1f\r\n",
                 g_cascade_pitch.fused_angle, g_jy901s.pitch,
                 g_cascade_roll.fused_angle,  g_jy901s.roll,
                 g_cascade_yaw.fused_angle,   g_jy901s.yaw,
                 pitch_offset, roll_offset, yaw_offset);
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
        g_control_flag = 1;
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
