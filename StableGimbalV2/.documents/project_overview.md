# StableGimbalV2 — 三轴稳定云台工程文档

> **MCU**: STM32F103C8T6 (Cortex-M3, 72MHz)
> **IDE**: STM32CubeMX + GCC
> **版本**: 2.1

---

## 1. 工程总览

### 1.1 功能概述

三轴 (Pitch/Roll/Yaw) 姿态稳定云台。JY901S 九轴 IMU 实时测量姿态角和角速度，串级 PID 控制器计算修正量，驱动三个 AX-12A 舵机保持目标角度。

### 1.2 目录结构

```
StableGimbalV2/
├── Core/
│   ├── Inc/                    # CubeMX 生成的头文件 (main.h, stm32f1xx_it.h ...)
│   └── Src/
│       ├── main.c              # 主程序：初始化 + 主循环
│       ├── stm32f1xx_it.c      # 中断服务程序
│       ├── stm32f1xx_hal_msp.c # HAL MSP 初始化 (GPIO/UART/DMA 底层配置)
│       ├── system_stm32f1xx.c  # 系统初始化 (CubeMX 生成)
│       ├── syscalls.c / sysmem.c # newlib 桩函数
├── App/
│   ├── Inc/
│   │   └── pid.h               # 串级 PID 控制器头文件
│   └── Src/
│       └── pid.c               # 串级 PID 控制器实现
├── Bsp/
│   ├── Inc/
│   │   ├── ax12a.h             # AX-12A 舵机驱动头文件
│   │   └── jy901s.h           # JY901S IMU 驱动头文件
│   └── Src/
│       ├── ax12a.c             # AX-12A 舵机驱动实现
│       └── jy901s.c            # JY901S IMU 驱动实现
└── Drivers/                    # STM32 HAL 库 + CMSIS (CubeMX 生成)
```

### 1.3 硬件接口分配

| 外设 | 引脚 | 波特率 | 用途 |
|------|------|--------|------|
| USART1 | PA9/PA10 | 115200 | printf 调试输出 (VOFA+) |
| USART2 | PA2/PA3 | 115200 | JY901S IMU 数据接收 (DMA) |
| USART3 | PB10/PB11 | 1000000 | AX-12A 舵机总线 (RS485) |
| TIM2 | — | 100Hz | PID 控制周期定时中断 |
| GPIO PB5 | RX485_TX_EN | — | RS485 方向控制 (高=发/低=收) |
| DMA1_CH6 | — | — | USART2 RX 循环 DMA |

### 1.4 软件架构

```
┌─────────────────────────────────────────────────────────┐
│                    main.c (主循环)                       │
│                                                         │
│  ┌──────────┐   ┌──────────┐   ┌───────────────────┐   │
│  │ JY901S   │   │  PID     │   │  AX-12A           │   │
│  │ IMU 驱动 │──→│ 串级控制 │──→│ 舵机驱动          │   │
│  │ (BSP)    │   │ (App)    │   │ (BSP)             │   │
│  └──────────┘   └──────────┘   └───────────────────┘   │
│       ↑                              ↓                  │
│  USART2 DMA+IDLE              USART3 RS485             │
│  (115200 baud)                (1Mbps)                   │
└─────────────────────────────────────────────────────────┘
         ↑                              ↓
    ┌─────────┐                   ┌──────────┐
    │ JY901S  │                   │ AX-12A   │
    │ 九轴IMU │                   │ ×3 舵机  │
    └─────────┘                   └──────────┘
```

### 1.5 中断与调度

| 中断源 | 频率 | 处理 |
|--------|------|------|
| TIM2_UP | 100Hz | `g_pidFlag++` (仅置标志) |
| USART2 IDLE | 事件驱动 | `jy901sOnIdleIrq()` 记录 DMA 写位置 |
| DMA1_CH6 | 持续 | USART2 RX 循环 DMA (HAL 管理) |
| SysTick | 1kHz | `HAL_IncTick()` 系统时钟 |

主循环轮询 `g_pidFlag`，每收到一个标志执行一次完整控制周期。

---

## 2. 模块详解

---

### 2.1 JY901S IMU 驱动 (jy901s)

> **文件**: `Bsp/Inc/jy901s.h` + `Bsp/Src/jy901s.c`
> **功能**: 九轴 IMU 姿态数据接收与解析
> **通信**: USART2 115200baud, DMA 循环接收 + IDLE 中断

#### 2.1.1 协议帧格式

```
帧头(0x55) + 类型(1B) + 数据(8B) + 校验(1B) = 11 字节
```

| 类型码 | 含义 | 数据内容 |
|--------|------|----------|
| `0x52` (GYRO) | 角速度 | wx, wy, wz (int16, 小端, 量程 2000°/s) |
| `0x53` (ANGLE) | 姿态角 | roll, pitch, yaw (int16, 小端, 量程 180°) |

#### 2.1.2 数据结构

```c
typedef struct {
    /* 输出数据 */
    float wx, wy, wz;           /* 角速度 (°/s) */
    float pitch, roll, yaw;     /* 姿态角 (°) */

    /* 接收状态 (主循环独占) */
    Jy901sState state;           /* 解析状态机 */
    uint8_t  frameBuf[11];      /* 帧组装缓冲 */
    uint8_t  frameIndex;        /* 帧内索引 */
    uint8_t  frameType;         /* 当前帧类型 */
    volatile uint8_t dataReady; /* 帧解析完成标志 */

    /* DMA 环形缓冲 (ISR 写 dmaPos, 主循环读 dmaHead) */
    uint8_t  dmaBuf[256];       /* DMA 循环缓冲区 (2的幂) */
    volatile uint16_t dmaHead;  /* 主循环读指针 */
    volatile uint16_t dmaPos;   /* ISR 记录的 DMA 写位置 */
    volatile uint8_t  idleFlag; /* IDLE 已发生标志 */
} Jy901s;
```

#### 2.1.3 公开 API

| 函数 | 上下文 | 说明 |
|------|--------|------|
| `jy901sInit(imu, huart)` | main 初始化 | 清零句柄，绑定 UART |
| `jy901sStartReceive(imu)` | main 初始化 | 启动 DMA 循环接收 + 使能 IDLE 中断 |
| `jy901sOnIdleIrq(imu)` | **ISR** (USART2) | 暂停 DMA → 清 IDLE → 记录写位置 → 恢复 DMA |
| `jy901sPoll(imu)` | main 循环 | 消费环形缓冲区字节，驱动状态机解析 |
| `jy901sIsDataReady(imu)` | main 循环 | 查询是否有新帧 |
| `jy901sClearDataReady(imu)` | main 循环 | 清除数据就绪标志 |

#### 2.1.4 内部函数

| 函数 | 说明 |
|------|------|
| `processByte(imu, data)` | 单字节状态机：WAIT_HEADER → WAIT_TYPE → RECEIVING_DATA → 校验 → 解析 |

#### 2.1.5 接收流程

```
USART2 DMA 循环接收 ──→ dmaBuf[256] 环形缓冲
        │
    IDLE 中断触发
        │
    jy901sOnIdleIrq():
        ① 暂停 DMA 通道
        ② 清除 IDLE 标志 (读 SR+DR)
        ③ 记录 DMA 写位置 (dmaPos = BUF_SIZE - CNDTR)
        ④ 恢复 DMA 通道
        ⑤ 置 idleFlag = 1
        │
    主循环 jy901sPoll():
        ① 检查 idleFlag
        ② 逐字节消费 dmaBuf[head → pos]
        ③ processByte() 状态机解析
        ④ 校验通过 → 更新 wx/wy/wz 或 pitch/roll/yaw
        ⑤ 置 dataReady = 1
```

#### 2.1.6 解析状态机

```
                  ┌──────────────┐
                  │ WAIT_HEADER  │←─────────────────────┐
                  └──────┬───────┘                      │
                    0x55 │                              │
                         ↓                              │
                  ┌──────────────┐     非法字节         │
                  │  WAIT_TYPE   │──────────────────────┘
                  └──────┬───────┘
            0x52/0x53    │  0x55 → 保持 (新帧头)
                         ↓
                  ┌──────────────┐
                  │ RECEIVING    │  接收 8 字节数据
                  │ DATA         │  → 帧满 11 字节
                  └──────┬───────┘
                         │
                    校验通过?
                   ╱          ╲
                 是             否
                 ↓              ↓
           解析 int16       丢弃
           更新输出         重置状态机
                 ↓              ↓
                  └──────┬───────┘
                         │
                    WAIT_HEADER (重置)
```

---

### 2.2 AX-12A 舵机驱动 (ax12a)

> **文件**: `Bsp/Inc/ax12a.h` + `Bsp/Src/ax12a.c`
> **功能**: Dynamixel Protocol 1.0 舵机通信
> **通信**: USART3 1Mbps, RS485 半双工 (GPIO PB5 方向控制)

#### 2.2.1 协议格式

```
指令包: [0xFF][0xFF][ID][Length][Instruction][Params...][Checksum]
状态包: [0xFF][0xFF][ID][Length][Error][Params...][Checksum]
```

#### 2.2.2 公开 API

| 函数 | 说明 |
|------|------|
| `ax12aInit(bus, huart, txEnPort, txEnPin)` | 初始化总线句柄，绑定 UART + RS485 方向引脚 |
| `ax12aSetGoalPosition(bus, id, pos)` | 设置目标位置 (0~1023, 对应 0°~300°) |
| `ax12aSetMovingSpeed(bus, id, spd)` | 设置移动速度 (0~1023) |

#### 2.2.3 内部函数

| 函数 | 说明 |
|------|------|
| `ax12aWrite(bus, id, addr, dataLen, data)` | [static] 向舵机寄存器写入数据 |
| `sendPacket(bus, pkt, len)` | [static] 阻塞发送指令包 (含 RS485 方向切换 + TC 等待) |

#### 2.2.4 发送流程

```
ax12aSetGoalPosition()
    │
    ├→ 组装 buf[2] = {pos低字节, pos高字节}
    │
    └→ ax12aWrite(bus, id, GOAL_POSITION, 2, buf)
        │
        ├→ 构建指令包: [FF FF ID Len=W+3 INST_WRITE ADDR DATA... CKSM]
        ├→ CALC_CHECKSUM: ~(ID + Len + Inst + Params...)
        │
        └→ sendPacket(bus, pkt, len)
            │
            ├① 清除 ORE 溢出标志 (读 SR+DR)
            ├② RS485_TX_MODE (GPIO 高)
            ├③ HAL_UART_Transmit (阻塞, 100ms 超时)
            ├④ 等待 TC 标志 (发送完成, 100ms 超时)
            ├⑤ RS485_RX_MODE (GPIO 低)
            └⑥ 延时 ~20us (MAX485 方向切换稳定)
```

---

### 2.3 串级 PID 控制器 (pid)

> **文件**: `App/Inc/pid.h` + `App/Src/pid.c`
> **功能**: 三轴串级 PID 控制 (外环 20Hz + 内环 100Hz)

#### 2.3.1 控制架构

```
目标角度 ──→ [外环 PID 20Hz] ──→ 目标角速度 ──→ [内环 PID 100Hz] ──→ 舵机位置
               角度误差                               角速度误差
               死区 0.5°                               死区 1.0°/s
               积分分离 20°                             积分限幅 100
               输出限幅 120°/s                          输出限幅 60°
                                                        │
                                                   低通滤波
                                                   PI项: 15Hz
                                                   D项:  53Hz
```

#### 2.3.2 数据结构

```c
typedef struct {
    /* 外环 (角度→目标角速度) */
    float outerKp, outerKi, outerKd;    /* PID 增益 */
    float outerIntegral;                 /* 积分累加 */
    float outerIntegThreshold;           /* 积分分离阈值 (度) */
    float outerOutputMax;                /* 输出限幅 (°/s) */
    float outerDeadband;                 /* 死区 (度) */

    /* 内环 (角速度→输出) */
    float innerKp, innerKi, innerKd;    /* PID 增益 */
    float innerIntegral;                 /* 积分累加 */
    float innerIntegMax;                 /* 积分上限 */
    float innerDeadband;                 /* 死区 (°/s) */
    float innerOutputMax;                /* 输出限幅 (度) */

    /* 运行时状态 */
    float filteredGyro;                  /* PI项低通滤波值 (15Hz) */
    float dFilteredGyro;                 /* D项巴特沃斯滤波值 (53Hz) */
    float lastGyro;                      /* 上一拍D项滤波值 */
    float lastOuterError;                /* 上一拍外环误差 */
    float targetAngle;                   /* 目标角度 (度) */
    float targetAngularVel;              /* 外环输出目标角速度 (°/s) */
} CascadedPid;
```

#### 2.3.3 公开 API

| 函数 | 频率 | 说明 |
|------|------|------|
| `cascadedPidInit(cp, target)` | 初始化 | 参数清零，加载默认限幅/死区 |
| `cascadedPidInitTuned(cp, target, axis)` | 初始化 | 加载三轴调试好的 PID 增益 |
| `cascadedPidOuterUpdate(cp, currentAngle)` | 20Hz | 外环：角度误差 → 目标角速度 |
| `cascadedPidInnerUpdate(cp, gyro)` | 100Hz | 内环：角速度误差 → 输出角度 |
| `pidAx12aPitchOutput(cp, gyro, goalPos, bus)` | 100Hz | Pitch 舵机输出封装 |
| `pidAx12aRollOutput(cp, gyro, goalPos, bus)` | 100Hz | Roll 舵机输出封装 (取反) |
| `pidAx12aYawOutput(cp, gyro, goalPos, bus)` | 100Hz | Yaw 舵机输出封装 |

#### 2.3.4 内部函数

| 函数 | 说明 |
|------|------|
| `pidShortestAngleDiff(target, current)` | [static] 最短角度差，结果范围 (-180°, +180°] |

#### 2.3.5 外环计算流程 (cascadedPidOuterUpdate)

```
输入: currentAngle (来自 IMU)
    │
    ├① angleError = pidShortestAngleDiff(target, current)
    │     取 [-180, +180] 范围内的最短弧
    │
    ├② 死区判断: |angleError| < outerDeadband(0.5°) → 归零
    │
    ├③ P 项: targetAngularVel = outerKp × angleError
    │
    ├④ I 项: 仅当 |angleError| < outerIntegThreshold(20°) 时累加
    │        outerIntegral += angleError × PID_OUTER_DT(0.05s)
    │        targetAngularVel += outerKi × outerIntegral
    │
    ├⑤ D 项: (angleError - lastOuterError) / PID_OUTER_DT
    │        targetAngularVel += outerKd × dError/dt
    │        lastOuterError = angleError
    │
    └⑥ 输出限幅: targetAngularVel = clamp(±outerOutputMax)
```

#### 2.3.6 内环计算流程 (cascadedPidInnerUpdate)

```
输入: gyro (来自 IMU 角速度)
    │
    ├① 陀螺低通滤波 (15Hz, PI项用)
    │   filteredGyro += α × (gyro - filteredGyro)
    │
    ├② 角速度误差 = targetAngularVel - filteredGyro
    │
    ├③ 死区判断: |velError| < innerDeadband(1.0°/s) → 归零
    │
    ├④ P 项: output = innerKp × velError
    │
    ├⑤ I 项: innerIntegral += velError × PID_TS(0.01s)
    │        积分限幅: ±innerIntegMax
    │        output += innerKi × innerIntegral
    │
    ├⑥ D 项: 巴特沃斯低通 (53Hz)
    │        dFilteredGyro += α_d × (gyro - dFilteredGyro)
    │        output += innerKd × (dFilteredGyro - lastGyro) / PID_TS
    │        lastGyro = dFilteredGyro
    │
    └⑦ 输出限幅: output = clamp(±innerOutputMax)

返回: output (度), 正值 = 向正方向移动舵机
```

#### 2.3.7 舵机输出封装 (pidAx12a*Output)

```
输入: cp (PID状态), gyro (角速度), goalPos[3] (舵机位置), bus (总线)
    │
    ├① pidOut = cascadedPidInnerUpdate(cp, gyro)
    │
    ├② deltaPos = pidOut × PID_POS_SCALE (1024/300 步/度)
    │
    ├③ 增量限幅: ±PID_OUT_DELTA_MAX (15步)
    │
    ├④ newPos = goalPos[i] + deltaPos
    │
    ├⑤ 位置限幅: [SERVO_{AXIS}_POS_MIN, SERVO_{AXIS}_POS_MAX]
    │
    └⑥ ax12aSetGoalPosition(bus, servoId, newPos)

注: Roll 输出取反 (-pidOut), 因舵机安装方向相反
```

#### 2.3.8 三轴调试参数

| 参数 | Pitch | Roll | Yaw |
|------|-------|------|-----|
| innerKp | 0.016 | 0.016 | 0.015 |
| innerKi | 0.0 | 0.0 | 0.0 |
| innerKd | 0.0 | 0.0 | 0.0 |
| innerOutputMax | 60.0 | 60.0 | 60.0 |
| outerKp | 10.0 | 6.0 | 6.0 |
| outerKi | 0.1 | 0.1 | 1.5 |
| outerKd | 0.3 | 0.3 | 0.5 |

---

### 2.4 主程序 (main.c)

> **文件**: `Core/Src/main.c`

#### 2.4.1 全局变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `g_jy901s` | `Jy901s` | IMU 句柄 |
| `g_ax12aBus` | `Ax12aBus` | 舵机总线句柄 |
| `g_cascadedPitch` | `CascadedPid` | Pitch PID 控制器 |
| `g_cascadedRoll` | `CascadedPid` | Roll PID 控制器 |
| `g_cascadedYaw` | `CascadedPid` | Yaw PID 控制器 (当前未启用) |
| `g_goalPos[3]` | `uint16_t` | 三轴舵机目标位置 |
| `g_pidFlag` | `volatile uint32_t` | TIM2 中断计数标志 |

#### 2.4.2 初始化顺序

```
HAL_Init()
SystemClock_Config()          // HSE 8MHz × PLL9 = 72MHz
MX_GPIO_Init()                // RS485 TX EN (PB5)
MX_DMA_Init()                 // DMA1_CH6 (USART2 RX)
MX_USART1_UART_Init()         // 115200 (调试)
MX_USART2_UART_Init()         // 115200 (IMU)
MX_USART3_UART_Init()         // 1Mbps (舵机)
MX_TIM2_Init()                // 72MHz/(71+1)/(9999+1) = 100Hz

jy901sInit() + jy901sStartReceive()   // IMU DMA 接收启动
ax12aInit()                           // 舵机总线初始化
HAL_Delay(2000)                       // 等待舵机就绪
ax12aSetMovingSpeed(1/2/3, 200)       // 设置三轴速度
ax12aSetGoalPosition(1/2/3, 512)      // 归中 (150°)
HAL_Delay(2000)                       // 等待归中完成
cascadedPidInitTuned(Pitch/Roll/Yaw, 0.0f)  // PID 目标=0°
HAL_TIM_Base_Start_IT(&htim2)         // 启动 100Hz 定时中断
```

#### 2.4.3 主循环逻辑

```
while (1) {
    ① 等待 g_pidFlag > 0 (TIM2 100Hz 中断驱动)
       g_pidFlag--

    ② jy901sPoll() — 消费 DMA 缓冲区，解析 IMU 数据

    ③ 检查 jy901sIsDataReady()
       若无新数据 → continue

    ④ 读取姿态: pitch, roll, yaw, wx, wy, wz

    ⑤ Pitch 控制 (内环 100Hz):
       pidAx12aPitchOutput(gyro=wy)
       每 5 拍执行一次外环 (20Hz):
         cascadedPidOuterUpdate(currentPitch)

    ⑥ Roll 控制 (内环 100Hz):
       pidAx12aRollOutput(gyro=wx)
       每 5 拍执行一次外环 (20Hz):
         cascadedPidOuterUpdate(currentRoll)

    ⑦ Yaw 控制: [当前已注释禁用]

    ⑧ VOFA+ 调试输出 (每 10 拍 = 10Hz):
       printf("%.2f,%.2f,%.2f", pitch, roll, yaw)
}
```

#### 2.4.4 printf 重定向

```c
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
```

输出到 USART1 (115200), VOFA+ Firewater 协议格式 (逗号分隔 + `\r\n`)。

---

### 2.5 中断服务 (stm32f1xx_it.c)

> **文件**: `Core/Src/stm32f1xx_it.c`

#### 2.5.1 用户中断

| 中断 | 处理 |
|------|------|
| `TIM2_IRQHandler` | HAL_TIM_IRQHandler → **g_pidFlag++** |
| `USART2_IRQHandler` | HAL_UART_IRQHandler → **jy901sOnIdleIrq()** |
| `DMA1_Channel6_IRQHandler` | HAL_DMA_IRQHandler (USART2 RX DMA) |

#### 2.5.2 系统异常

NMI / HardFault / MemManage / BusFault / UsageFault: 死循环。
SysTick: `HAL_IncTick()` 系统时钟递增。

---

## 3. 完整调用链

### 3.1 初始化调用链

```
main()
 ├→ HAL_Init()
 ├→ SystemClock_Config()
 ├→ MX_GPIO_Init / MX_DMA_Init / MX_USART*_Init / MX_TIM2_Init
 ├→ jy901sInit(&g_jy901s, &huart2)
 ├→ jy901sStartReceive(&g_jy901s)
 │   └→ HAL_UART_Receive_DMA() + __HAL_UART_ENABLE_IT(IDLE)
 ├→ ax12aInit(&g_ax12aBus, &huart3, ...)
 ├→ ax12aSetMovingSpeed(&g_ax12aBus, 1/2/3, 200)
 │   └→ ax12aWrite() → sendPacket()
 ├→ ax12aSetGoalPosition(&g_ax12aBus, 1/2/3, 512)
 │   └→ ax12aWrite() → sendPacket()
 ├→ cascadedPidInitTuned(&g_cascadedPitch/Roll/Yaw, 0.0f, 1/2/3)
 │   ├→ cascadedPidInit()   (清零 + 默认限幅)
 │   └→ 覆盖三轴调试参数
 └→ HAL_TIM_Base_Start_IT(&htim2)
```

### 3.2 运行时调用链 (每个 10ms 周期)

```
[TIM2 ISR]  g_pidFlag++
     │
[主循环]    检测 g_pidFlag > 0
     │
     ├→ jy901sPoll(&g_jy901s)
     │   └→ processByte() × N  (状态机解析 DMA 数据)
     │       └→ 更新 pitch/roll/yaw/wx/wy/wz
     │
     ├→ [Pitch 内环 100Hz]
     │   pidAx12aPitchOutput()
     │   ├→ cascadedPidInnerUpdate(gyro=wy)
     │   │   ├→ filteredGyro 低通滤波 (15Hz)
     │   │   ├→ velError = targetAngularVel - filteredGyro
     │   │   ├→ P + I + D 计算
     │   │   └→ 输出限幅 ±innerOutputMax
     │   ├→ deltaPos = pidOut × PID_POS_SCALE
     │   ├→ 增量限幅 ±15
     │   ├→ 位置限幅 [SERVO_PITCH_POS_MIN, MAX]
     │   └→ ax12aSetGoalPosition(bus, 1, newPos)
     │       └→ ax12aWrite() → sendPacket()
     │
     ├→ [Pitch 外环 20Hz] (每5拍)
     │   cascadedPidOuterUpdate(currentPitch)
     │   ├→ pidShortestAngleDiff()
     │   ├→ 死区 + P + I(积分分离) + D
     │   └→ 输出限幅 → targetAngularVel
     │
     ├→ [Roll 内环 100Hz] (同 Pitch, gyro=wx, 取反)
     ├→ [Roll 外环 20Hz]  (同 Pitch)
     │
     └→ [VOFA+ 输出 10Hz] (每10拍)
         printf("%.2f,%.2f,%.2f\r\n", pitch, roll, yaw)
```

### 3.3 ISR 调用链

```
[USART2 IDLE 中断]
 └→ USART2_IRQHandler()
     ├→ HAL_UART_IRQHandler(&huart2)
     └→ jy901sOnIdleIrq(&g_jy901s)
         ├→ __HAL_DMA_DISABLE(hdma_usart2_rx)
         ├→ __HAL_UART_CLEAR_IDLEFLAG()
         ├→ dmaPos = BUF_SIZE - CNDTR
         ├→ __HAL_DMA_ENABLE(hdma_usart2_rx)
         └→ idleFlag = 1

[TIM2 更新中断]
 └→ TIM2_IRQHandler()
     ├→ HAL_TIM_IRQHandler(&htim2)
     └→ g_pidFlag++
```

---

## 4. 宏定义汇总

### 4.1 PID 模块 (pid.h) — 45 个宏

| 分类 | 宏名 | 值 |
|------|------|-----|
| **时序** | `PID_TS` | 0.01s (100Hz) |
| | `PID_OUTER_PERIOD_RATIO` | 5.0 |
| | `PID_OUTER_DT` | 0.05s (20Hz) |
| **滤波器** | `PID_GYRO_FC` | 15.0Hz |
| | `PID_ALPHA` | 推导 (PI项低通) |
| | `PID_D_FC` | 53.0Hz |
| | `PID_D_ALPHA` | 推导 (D项低通) |
| **角度** | `PID_ANGLE_WRAP` | 360.0° |
| | `PID_HALF_ANGLE` | 180.0° |
| **外环默认** | `PID_OUTER_INTEG_THRESHOLD` | 20.0° |
| | `PID_OUTER_OUTPUT_MAX` | 120.0°/s |
| | `PID_OUTER_DEADBAND` | 0.5° |
| **内环默认** | `PID_INNER_INTEG_MAX` | 100.0 |
| | `PID_INNER_DEADBAND` | 1.0°/s |
| | `PID_INNER_OUTPUT_MAX` | 60.0° |
| **输出** | `PID_OUT_DELTA_MAX` | 15.0 步 |
| | `PID_POS_SCALE` | 1024/300 步/度 |
| **轴编号** | `PID_AXIS_PITCH / ROLL / YAW` | 1 / 2 / 3 |
| **Pitch** | `PID_PITCH_INNER_KP` | 0.016 |
| | `PID_PITCH_INNER_KI / KD` | 0.0 |
| | `PID_PITCH_INNER_OUTPUT_MAX` | 60.0 |
| | `PID_PITCH_OUTER_KP` | 10.0 |
| | `PID_PITCH_OUTER_KI` | 0.1 |
| | `PID_PITCH_OUTER_KD` | 0.3 |
| **Roll** | `PID_ROLL_INNER_KP` | 0.016 |
| | `PID_ROLL_INNER_KI / KD` | 0.0 |
| | `PID_ROLL_INNER_OUTPUT_MAX` | 60.0 |
| | `PID_ROLL_OUTER_KP` | 6.0 |
| | `PID_ROLL_OUTER_KI` | 0.1 |
| | `PID_ROLL_OUTER_KD` | 0.3 |
| **Yaw** | `PID_YAW_INNER_KP` | 0.015 |
| | `PID_YAW_INNER_KI / KD` | 0.0 |
| | `PID_YAW_INNER_OUTPUT_MAX` | 60.0 |
| | `PID_YAW_OUTER_KP` | 6.0 |
| | `PID_YAW_OUTER_KI` | 1.5 |
| | `PID_YAW_OUTER_KD` | 0.5 |
| **舵机限位** | `SERVO_PITCH_POS_MIN / MAX` | 0 / 1023 |
| | `SERVO_ROLL_POS_MIN / MAX` | 0 / 1023 |
| | `SERVO_YAW_POS_MIN / MAX` | 0 / 1023 |

### 4.2 AX-12A 驱动 (ax12a.h) — 9 个宏

| 宏名 | 值 | 说明 |
|------|-----|------|
| `AX12A_HEADER_1` | 0xFF | 帧头字节1 |
| `AX12A_HEADER_2` | 0xFF | 帧头字节2 |
| `AX12A_BROADCAST_ID` | 0xFE | 广播 ID |
| `AX12A_INST_WRITE` | 0x03 | 写指令 |
| `AX12A_INST_SYNC_WRITE` | 0x83 | 同步写指令 |
| `AX12A_ADDR_GOAL_POSITION` | 0x1E | 目标位置寄存器 |
| `AX12A_ADDR_MOVING_SPEED` | 0x20 | 移动速度寄存器 |
| `AX12A_TIMEOUT_MS` | 100 | 通信超时 (ms) |
| `AX12A_STATUS_PKT_BASE` | 6 | 状态包基础长度 |

### 4.3 JY901S 驱动 (jy901s.h) — 4 个宏

| 宏名 | 值 | 说明 |
|------|-----|------|
| `JY901S_FRAME_HEADER` | 0x55 | 帧头 |
| `JY901S_GYRO_TYPE` | 0x52 | 角速度帧类型 |
| `JY901S_ANGLE_TYPE` | 0x53 | 姿态角帧类型 |
| `JY901S_FRAME_LEN` | 11 | 帧长度 |
| `JY901S_DMA_BUF_SIZE` | 256 | DMA 缓冲区大小 |

---

## 5. 函数索引

### 5.1 按模块

| 模块 | 函数 | 可见性 | 说明 |
|------|------|--------|------|
| **jy901s** | `jy901sInit` | public | 初始化 IMU 句柄 |
| | `jy901sStartReceive` | public | 启动 DMA + IDLE |
| | `jy901sOnIdleIrq` | public | IDLE 中断处理 (ISR) |
| | `jy901sPoll` | public | 主循环轮询解析 |
| | `jy901sIsDataReady` | public | 查询数据就绪 |
| | `jy901sClearDataReady` | public | 清除就绪标志 |
| | `processByte` | static | 单字节状态机 |
| **ax12a** | `ax12aInit` | public | 初始化总线句柄 |
| | `ax12aSetGoalPosition` | public | 设置目标位置 |
| | `ax12aSetMovingSpeed` | public | 设置移动速度 |
| | `ax12aWrite` | static | 寄存器写入 |
| | `sendPacket` | static | 阻塞发送 |
| **pid** | `cascadedPidInit` | public | PID 初始化 |
| | `cascadedPidInitTuned` | public | 加载调试参数 |
| | `cascadedPidOuterUpdate` | public | 外环更新 (20Hz) |
| | `cascadedPidInnerUpdate` | public | 内环更新 (100Hz) |
| | `pidAx12aPitchOutput` | public | Pitch 舵机输出 |
| | `pidAx12aRollOutput` | public | Roll 舵机输出 |
| | `pidAx12aYawOutput` | public | Yaw 舵机输出 |
| | `pidShortestAngleDiff` | static | 最短角度差 |
| **main** | `main` | public | 主程序入口 |
| | `SystemClock_Config` | public | 时钟配置 |
| | `MX_GPIO_Init` | static | GPIO 初始化 |
| | `MX_DMA_Init` | static | DMA 初始化 |
| | `MX_USART1/2/3_UART_Init` | static | UART 初始化 |
| | `MX_TIM2_Init` | static | TIM2 初始化 |
| | `__io_putchar` | public | printf 重定向 |
| | `Error_Handler` | public | 错误处理 |
| **stm32f1xx_it** | `TIM2_IRQHandler` | public | TIM2 中断 → g_pidFlag++ |
| | `USART2_IRQHandler` | public | USART2 中断 → IDLE 处理 |
| | `DMA1_Channel6_IRQHandler` | public | DMA 中断 |
| | `SysTick_Handler` | public | 系统时钟 |

### 5.2 公开 API 速查

```
/* IMU */
void    jy901sInit(Jy901s *imu, UART_HandleTypeDef *huart);
void    jy901sStartReceive(Jy901s *imu);
void    jy901sOnIdleIrq(Jy901s *imu);
void    jy901sPoll(Jy901s *imu);
uint8_t jy901sIsDataReady(const Jy901s *imu);
void    jy901sClearDataReady(Jy901s *imu);

/* 舵机 */
void       ax12aInit(Ax12aBus *bus, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *txEnPort, uint16_t txEnPin);
Ax12aStatus ax12aSetGoalPosition(Ax12aBus *bus, uint8_t id, uint16_t pos);
Ax12aStatus ax12aSetMovingSpeed(Ax12aBus *bus, uint8_t id, uint16_t spd);

/* PID */
void  cascadedPidInit(CascadedPid *cp, float target);
void  cascadedPidInitTuned(CascadedPid *cp, float target, uint8_t axis);
void  cascadedPidOuterUpdate(CascadedPid *cp, float currentAngle);
float cascadedPidInnerUpdate(CascadedPid *cp, float gyro);
void  pidAx12aPitchOutput(CascadedPid *cp, float gyro,
                          uint16_t *goalPos, void *bus);
void  pidAx12aRollOutput(CascadedPid *cp, float gyro,
                         uint16_t *goalPos, void *bus);
void  pidAx12aYawOutput(CascadedPid *cp, float gyro,
                        uint16_t *goalPos, void *bus);
```
