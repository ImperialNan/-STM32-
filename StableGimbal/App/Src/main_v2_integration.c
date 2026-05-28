/**
  ******************************************************************************
  * @file    main_v2_integration.c
  * @brief   V2 串级 PID 在 main.c 中的集成示例
  *
  *  用法: 将以下代码段替换 main.c 中对应的 USER CODE 区域即可。
  *  注: 本文件仅作参考，不直接编译。
  ******************************************************************************
  *
  * ═══════════════════════════════════════════════════════════════════════════
  *  需要在 main.c 顶部 #include 中添加:
  *    #include "pid_controller_v2.h"
  * ═══════════════════════════════════════════════════════════════════════════
  */

/* ========================================================================== */
/*  USER CODE BEGIN PV   (替换全局变量区)                                      */
/* ========================================================================== */

/* 全局实例 */
JY901S_t g_jy901s;

AX12A_t g_ax12a_pitch;
AX12A_t g_ax12a_roll;
AX12A_t g_ax12a_yaw;

/* V2 串级 PID 控制器 (替代旧的 CascadePID_t) */
CascadePID_v2_t g_c_pitch;
CascadePID_v2_t g_c_roll;
CascadePID_v2_t g_c_yaw;

/* printf 重定向 (保持不变) */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ========================================================================== */
/*  USER CODE BEGIN 0   (替换私有代码区)                                       */
/* ========================================================================== */

volatile uint8_t g_control_flag = 0;

/* 双速率分频 */
#define OUTER_DIV    10    /* 100Hz / 10 = 10Hz 外环 */
#define FB_DIV        5    /* 100Hz / 5  = 20Hz 舵机反馈读取 */

/* 控制参数 (集中管理, 方便整定) */
#define DT_OUTER  0.1f     /* 外环 dt = 100ms */
#define DT_INNER  0.01f    /* 内环 dt = 10ms  */

/* ========================================================================== */
/*  USER CODE BEGIN 2   (替换初始化区, 在 MX_TIM2_Init 之后)                    */
/* ========================================================================== */

  /* ── BSP 初始化 ── */
  JY901S_Init(&g_jy901s, &huart2);

  AX12A_Init(&g_ax12a_pitch, &huart3, 1, 0.0f, 300.0f);
  AX12A_Init(&g_ax12a_roll,  &huart3, 2, 0.0f, 300.0f);
  AX12A_Init(&g_ax12a_yaw,   &huart3, 3, 100.0f, 200.0f);

  /* ── 串级 PID 初始化 ──
   *
   *  CascadePID_v2_Init(c,
   *      // 外环 (位置型)
   *      Kp,  Ki(/s),  Kd,  死区, 输出abs_max, 增量max, 积分abs_max, 微分滤波α,
   *      // 内环 (增量型)
   *      Kp,  Ki(/s),  Kd,  死区, 输出abs_max, 增量max, 微分滤波α,
   *      // 前馈 & 速度
   *      Kff,  speed_K,  speed_min,  speed_max);
   */

  /* PITCH */
  CascadePID_v2_Init(&g_c_pitch,
      2.0f, 0.5f, 0.8f,     /* 外环 Kp, Ki(/s)=0.5, Kd       */
      0.3f, 150.0f, 10.0f, 50.0f, 0.3f,
      0.5f, 2.0f, 0.1f,     /* 内环 Kp, Ki(/s)=2.0, Kd       */
      0.3f, 40.0f, 5.0f, 0.3f,
      0.05f,                 /* Kff 前馈 */
      2.0f, 50, 1023);       /* 速度映射 */

  /* ROLL */
  CascadePID_v2_Init(&g_c_roll,
      2.0f, 0.5f, 0.8f,
      0.5f, 150.0f, 10.0f, 50.0f, 0.3f,
      0.5f, 2.0f, 0.1f,
      0.3f, 40.0f, 5.0f, 0.3f,
      0.05f,
      2.0f, 50, 1023);

  /* YAW */
  CascadePID_v2_Init(&g_c_yaw,
      1.5f, 0.3f, 0.6f,
      0.5f, 100.0f, 8.0f, 40.0f, 0.3f,
      0.4f, 2.0f, 0.08f,
      0.3f, 30.0f, 4.0f, 0.3f,
      0.04f,
      2.0f, 50, 1023);

  /* ── 传感器融合配置 ── */
  g_c_pitch.sensor.source     = SENSOR_SRC_FUSION;
  g_c_pitch.sensor.imu_weight = 0.7f;

  g_c_roll.sensor.source      = SENSOR_SRC_FUSION;
  g_c_roll.sensor.imu_weight  = 0.7f;

  g_c_yaw.sensor.source       = SENSOR_SRC_FUSION;
  g_c_yaw.sensor.imu_weight   = 0.6f;

  /* 启动 */
  JY901S_StartReceive(&g_jy901s);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(500);
  LED_ON();

/* ========================================================================== */
/*  USER CODE BEGIN 3   (替换主循环 while(1) 内部)                              */
/* ========================================================================== */

  if (g_control_flag) {
      g_control_flag = 0;

      /* ── 舵机反馈读取 (降频至 20Hz, 与 PID 异步) ── */
      static uint8_t fb_cnt = 0;
      if (++fb_cnt >= FB_DIV) {
          fb_cnt = 0;
          AX12A_ReadFeedback(&g_ax12a_pitch);
          AX12A_ReadFeedback(&g_ax12a_roll);
          AX12A_ReadFeedback(&g_ax12a_yaw);
      }

      /* ── 填充传感器原始数据 (IMU 批量读取前关中断, 防止竞态) ── */
      __disable_irq();
      float pitch_imu = g_jy901s.pitch;
      float pitch_w   = g_jy901s.wx;
      float roll_imu  = g_jy901s.roll;
      float roll_w    = g_jy901s.wy;
      float yaw_imu   = g_jy901s.yaw;
      float yaw_w     = g_jy901s.wz;
      __enable_irq();

      g_c_pitch.sensor.imu_angle = pitch_imu;
      g_c_pitch.sensor.imu_w     = pitch_w;
      g_c_pitch.sensor.ax12a_angle = g_ax12a_pitch.feedback.angle_deg;
      g_c_pitch.sensor.ax12a_speed = g_ax12a_pitch.feedback.speed_dps;
      g_c_pitch.sensor.ax12a_valid = g_ax12a_pitch.feedback.data_valid;

      g_c_roll.sensor.imu_angle = roll_imu;
      g_c_roll.sensor.imu_w     = roll_w;
      g_c_roll.sensor.ax12a_angle = g_ax12a_roll.feedback.angle_deg;
      g_c_roll.sensor.ax12a_speed = g_ax12a_roll.feedback.speed_dps;
      g_c_roll.sensor.ax12a_valid = g_ax12a_roll.feedback.data_valid;

      g_c_yaw.sensor.imu_angle = yaw_imu;
      g_c_yaw.sensor.imu_w     = yaw_w;
      g_c_yaw.sensor.ax12a_angle = g_ax12a_yaw.feedback.angle_deg;
      g_c_yaw.sensor.ax12a_speed = g_ax12a_yaw.feedback.speed_dps;
      g_c_yaw.sensor.ax12a_valid = g_ax12a_yaw.feedback.data_valid;

      /* ── 双速率串级 PID ── */
      static uint8_t outer_div = 0;
      if (++outer_div >= OUTER_DIV) {
          outer_div = 0;

          /* 外环 10Hz */
          CascadePID_v2_ComputeOuter(&g_c_pitch,  0.0f,   DT_OUTER);
          CascadePID_v2_ComputeOuter(&g_c_roll,   0.0f,   DT_OUTER);
          CascadePID_v2_ComputeOuter(&g_c_yaw,  -143.0f,  DT_OUTER);
      }

      /* 内环 100Hz */
      float p_off = CascadePID_v2_ComputeInner(&g_c_pitch, DT_INNER);
      float r_off = CascadePID_v2_ComputeInner(&g_c_roll,  DT_INNER);
      float y_off = CascadePID_v2_ComputeInner(&g_c_yaw,   DT_INNER);

      /* ── 输出到舵机 ── */
      {
          uint8_t  ids[3]       = {1, 2, 3};
          uint16_t pos[3] = {
              (uint16_t)((240.0f + p_off) * AX12A_DEG_TO_POS),
              (uint16_t)((150.0f + r_off) * AX12A_DEG_TO_POS),
              (uint16_t)((150.0f - y_off) * AX12A_DEG_TO_POS)
          };
          uint16_t spd[3] = {
              g_c_pitch.servo_speed,
              g_c_roll.servo_speed,
              g_c_yaw.servo_speed
          };
          AX12A_SyncWrite(&huart3, ids, pos, spd, 3);
      }

      /* ── 调试输出 (含饱和标志) ── */
      static uint16_t dbg = 0;
      if (++dbg >= 10) {
          dbg = 0;
          printf("P:%.1f/%.1f R:%.1f/%.1f Y:%.1f/%.1f"
                 " | o:%+.1f %+.1f %+.1f"
                 " | sat_o:%d%d%d sat_i:%d%d%d"
                 " | cmd:%.1f %.1f %.1f\r\n",
                 g_c_pitch.sensor.fused_angle, pitch_imu,
                 g_c_roll.sensor.fused_angle,  roll_imu,
                 g_c_yaw.sensor.fused_angle,   yaw_imu,
                 p_off, r_off, y_off,
                 g_c_pitch.outer_sat, g_c_roll.outer_sat, g_c_yaw.outer_sat,
                 g_c_pitch.inner_sat, g_c_roll.inner_sat, g_c_yaw.inner_sat,
                 g_c_pitch.velocity_cmd, g_c_roll.velocity_cmd, g_c_yaw.velocity_cmd);
      }

      LED_Toggle();
  }
