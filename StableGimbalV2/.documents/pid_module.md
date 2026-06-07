# PID 模块文档

> **文件**: `App/Inc/pid.h` + `App/Src/pid.c`
> **版本**: 1.1
> **功能**: 三轴稳定云台串级 PID 控制器
> **架构**: 外环 (角度→目标角速度) 20Hz + 内环 (角速度→舵机输出) 100Hz

---

## 1. 模块总览

```
目标角度 ──→ [外环 PID] ──→ 目标角速度 ──→ [内环 PID] ──→ 舵机位置
  20Hz         角度误差         100Hz         角速度误差       AX-12A
               死区+积分分离                    低通滤波+死区
```

### 1.1 依赖关系

| 依赖 | 说明 |
|------|------|
| `<stdint.h>` | 标准整型 |
| `ax12a.h` | AX-12A 舵机总线驱动 (BSP 层) |

### 1.2 公开 API 一览

| 函数 | 频率 | 说明 |
|------|------|------|
| `cascadedPidInit()` | 初始化 | 参数清零，加载默认限幅 |
| `cascadedPidInitTuned()` | 初始化 | 加载三轴调试好的 PID 增益 |
| `cascadedPidOuterUpdate()` | 20Hz | 外环：角度误差 → 目标角速度 |
| `cascadedPidInnerUpdate()` | 100Hz | 内环：角速度误差 → 输出角度 |
| `pidAx12aPitchOutput()` | 100Hz | Pitch 舵机输出封装 |
| `pidAx12aRollOutput()` | 100Hz | Roll 舵机输出封装 |
| `pidAx12aYawOutput()` | 100Hz | Yaw 舵机输出封装 |

---

## 2. 头文件 (pid.h)

### 2.1 控制时序

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_TS` | 0.01s | 内环控制周期 (100Hz) |
| `PID_OUTER_PERIOD_RATIO` | 5.0 | 外环分频比 |
| `PID_OUTER_DT` | 0.05s | 外环控制周期 (20Hz) = 5 × PID_TS |

### 2.2 滤波器参数

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_GYRO_FC` | 15.0Hz | 陀螺低通截止频率 (PI 项用) |
| `PID_ALPHA` | 推导值 | PI 项低通滤波系数 |
| `PID_D_FC` | 53.0Hz | D 项巴特沃斯低通截止频率 (τ≈3ms) |
| `PID_D_ALPHA` | 推导值 | D 项低通滤波系数 |

### 2.3 角度常量

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_ANGLE_WRAP` | 360.0° | 全圆角度 |
| `PID_HALF_ANGLE` | 180.0° | 半圆角度 |

### 2.4 通用默认参数 (cascadedPidInit)

**外环**

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_OUTER_INTEG_THRESHOLD` | 20.0° | 积分分离阈值 |
| `PID_OUTER_OUTPUT_MAX` | 120.0°/s | 输出限幅 |
| `PID_OUTER_DEADBAND` | 0.5° | 死区 |

**内环**

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_INNER_INTEG_MAX` | 100.0 | 积分上限 |
| `PID_INNER_DEADBAND` | 1.0°/s | 死区 |
| `PID_INNER_OUTPUT_MAX` | 60.0° | 输出限幅 (通用默认) |

### 2.5 输出增量限幅

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_OUT_DELTA_MAX` | 15.0 步 | 单次位置增量限幅 |
| `PID_POS_SCALE` | 1024/300 步/度 | PID 输出→舵机位置转换系数 |

### 2.6 轴编号

| 宏 | 值 | 说明 |
|----|-----|------|
| `PID_AXIS_PITCH` | 1 | 俯仰轴 |
| `PID_AXIS_ROLL` | 2 | 横滚轴 |
| `PID_AXIS_YAW` | 3 | 偏航轴 |

### 2.7 三轴舵机硬件限制

| 宏 | 值 | 说明 |
|----|-----|------|
| `SERVO_PITCH_POS_MIN` | 0 | Pitch 舵机最小位置 (0°) |
| `SERVO_PITCH_POS_MAX` | 1023 | Pitch 舵机最大位置 (300°) |
| `SERVO_ROLL_POS_MIN` | 0 | Roll 舵机最小位置 (0°) |
| `SERVO_ROLL_POS_MAX` | 1023 | Roll 舵机最大位置 (300°) |
| `SERVO_YAW_POS_MIN` | 0 | Yaw 舵机最小位置 (0°) |
| `SERVO_YAW_POS_MAX` | 1023 | Yaw 舵机最大位置 (300°) |

### 2.8 三轴 PID 调试参数

| 宏 | Pitch | Roll | Yaw |
|----|-------|------|-----|
| `*_INNER_KP` | 0.016 | 0.016 | 0.015 |
| `*_INNER_KI` | 0.0 | 0.0 | 0.0 |
| `*_INNER_KD` | 0.0 | 0.0 | 0.0 |
| `*_INNER_OUTPUT_MAX` | 60.0 | 60.0 | 60.0 |
| `*_OUTER_KP` | 10.0 | 6.0 | 6.0 |
| `*_OUTER_KI` | 0.1 | 0.1 | 1.5 |
| `*_OUTER_KD` | 0.3 | 0.3 | 0.5 |

---

## 3. 类型定义

### 3.1 CascadedPid 结构体

```c
typedef struct {
    /* ---- 外环 (角度→目标角速度) ---- */
    float outerKp;              /* 外环 P */
    float outerKi;              /* 外环 I */
    float outerKd;              /* 外环 D (角速度前馈) */
    float outerIntegral;        /* 外环积分累加 */
    float outerIntegThreshold;  /* 外环积分分离阈值 (度) */
    float outerOutputMax;       /* 外环输出限幅 (°/s) */
    float outerDeadband;        /* 外环死区 (度) */

    /* ---- 内环 (角速度→输出) ---- */
    float innerKp;              /* 内环 P */
    float innerKi;              /* 内环 I */
    float innerKd;              /* 内环 D (角加速度阻尼) */
    float innerIntegral;        /* 内环积分累加 */
    float innerIntegMax;        /* 内环积分上限 */
    float innerDeadband;        /* 内环死区 (°/s) */
    float innerOutputMax;       /* 内环输出限幅 (度) */

    /* ---- 运行时状态 ---- */
    float filteredGyro;         /* 陀螺低通滤波值 (15Hz, PI项用) */
    float dFilteredGyro;        /* D项巴特沃斯滤波值 (53Hz, τ≈3ms) */
    float lastGyro;             /* 上一拍D项滤波值 (内环D差分用) */
    float lastOuterError;       /* 上一拍外环角度误差 (外环D用) */
    float targetAngle;          /* 目标角度 (度) */
    float targetAngularVel;     /* 外环输出的目标角速度 (°/s) */
} CascadedPid;
```

---

## 4. 源文件 (pid.c) 函数详解

### 4.1 pidShortestAngleDiff (内部辅助)

```
static float pidShortestAngleDiff(float target, float current)
```

计算两个角度之间的最短差值，结果范围 (-180°, +180°]。

```
示例: target=10°, current=350° → diff=-340° → +20° (取短弧)
```

### 4.2 cascadedPidInit

```
void cascadedPidInit(CascadedPid *cp, float target)
```

初始化串级 PID 状态：

- 外环增益 Kp/Ki/Kd 清零，加载默认限幅/死区/积分阈值
- 内环增益 Kp/Ki/Kd 清零，加载默认积分上限/死区/输出限幅
- 运行时滤波器、积分器、误差记录清零
- 设置目标角度

### 4.3 cascadedPidOuterUpdate (20Hz)

```
void cascadedPidOuterUpdate(CascadedPid *cp, float currentAngle)
```

外环 PID 计算流程：

```
1. 角度误差 = 最短角度差(目标, 当前)
2. 死区判断: |误差| < outerDeadband → 误差归零
3. P 项: targetAngularVel = Kp × 误差
4. I 项: 仅当 |误差| < 积分分离阈值时累加
         outerIntegral += 误差 × PID_OUTER_DT
         targetAngularVel += Ki × outerIntegral
5. D 项: (误差 - 上次误差) / PID_OUTER_DT
         targetAngularVel += Kd × dError/dt
6. 输出限幅: ±outerOutputMax
```

### 4.4 cascadedPidInnerUpdate (100Hz)

```
float cascadedPidInnerUpdate(CascadedPid *cp, float gyro)
```

内环 PID 计算流程：

```
1. 陀螺低通滤波: filteredGyro += α × (gyro - filteredGyro)   [15Hz]
2. 角速度误差 = 目标角速度 - 滤波后角速度
3. 死区判断: |误差| < innerDeadband → 误差归零
4. P 项: output = Kp × 误差
5. I 项: innerIntegral += 误差 × PID_TS
         积分限幅: ±innerIntegMax
         output += Ki × innerIntegral
6. D 项: dFilteredGyro 巴特沃斯低通 [53Hz]
         output += Kd × (dFilteredGyro - lastGyro) / PID_TS
7. 输出限幅: ±innerOutputMax
```

**返回值**: PID 输出 (度)，正值 = 向正方向移动舵机

### 4.5 cascadedPidInitTuned

```
void cascadedPidInitTuned(CascadedPid *cp, float target, uint8_t axis)
```

先调用 `cascadedPidInit()` 加载默认值，再按轴号覆盖调试好的增益参数：

| axis | 宏前缀 | 说明 |
|------|--------|------|
| `PID_AXIS_PITCH` (1) | `PID_PITCH_*` | 俯仰轴 |
| `PID_AXIS_ROLL` (2) | `PID_ROLL_*` | 横滚轴 |
| `PID_AXIS_YAW` (3) | `PID_YAW_*` | 偏航轴 |

### 4.6 舵机输出封装

三个函数结构相同，分别对应 Pitch / Roll / Yaw：

```
void pidAx12a{Axis}Output(CascadedPid *cp, float gyro,
                          uint16_t *goalPos, void *bus)
```

处理流程：

```
1. 调用 cascadedPidInnerUpdate() 获取 PID 输出
2. 转换为位置增量: deltaPos = pidOut × PID_POS_SCALE
3. 增量限幅: ±PID_OUT_DELTA_MAX
4. 叠加到当前目标位置: newPos = goalPos[i] + deltaPos
5. 位置限幅: [SERVO_{AXIS}_POS_MIN, SERVO_{AXIS}_POS_MAX]
6. 写入舵机: ax12aSetGoalPosition(bus, servoId, newPos)
```

| 函数 | goalPos 索引 | 舵机 ID | 限幅宏 |
|------|-------------|---------|--------|
| `pidAx12aPitchOutput` | [0] | 1 | `SERVO_PITCH_POS_*` |
| `pidAx12aRollOutput` | [1] | 2 | `SERVO_ROLL_POS_*` |
| `pidAx12aYawOutput` | [2] | 3 | `SERVO_YAW_POS_*` |

> **注意**: Roll 输出取反 (`-cascadedPidInnerUpdate`)，因舵机安装方向相反。

---

## 5. 宏速查表

按字母序排列，便于查找：

| 宏名 | 值 | 分类 |
|------|-----|------|
| `PID_ALPHA` | 推导 | 滤波器 |
| `PID_ANGLE_WRAP` | 360.0f | 角度常量 |
| `PID_AXIS_PITCH` | 1 | 轴编号 |
| `PID_AXIS_ROLL` | 2 | 轴编号 |
| `PID_AXIS_YAW` | 3 | 轴编号 |
| `PID_D_ALPHA` | 推导 | 滤波器 |
| `PID_D_FC` | 53.0f | 滤波器参数 |
| `PID_GYRO_FC` | 15.0f | 滤波器参数 |
| `PID_HALF_ANGLE` | 180.0f | 角度常量 |
| `PID_INNER_DEADBAND` | 1.0f | 内环默认 |
| `PID_INNER_INTEG_MAX` | 100.0f | 内环默认 |
| `PID_INNER_OUTPUT_MAX` | 60.0f | 内环默认 |
| `PID_OUT_DELTA_MAX` | 15.0f | 输出限幅 |
| `PID_OUTER_DEADBAND` | 0.5f | 外环默认 |
| `PID_OUTER_DT` | 推导 | 时序 |
| `PID_OUTER_INTEG_THRESHOLD` | 20.0f | 外环默认 |
| `PID_OUTER_OUTPUT_MAX` | 120.0f | 外环默认 |
| `PID_OUTER_PERIOD_RATIO` | 5.0f | 时序参数 |
| `PID_PITCH_INNER_KD` | 0.0f | Pitch 增益 |
| `PID_PITCH_INNER_KI` | 0.0f | Pitch 增益 |
| `PID_PITCH_INNER_KP` | 0.016f | Pitch 增益 |
| `PID_PITCH_INNER_OUTPUT_MAX` | 60.0f | Pitch 限幅 |
| `PID_PITCH_OUTER_KD` | 0.3f | Pitch 增益 |
| `PID_PITCH_OUTER_KI` | 0.1f | Pitch 增益 |
| `PID_PITCH_OUTER_KP` | 10.0f | Pitch 增益 |
| `PID_POS_SCALE` | 3.413f | 输出转换 |
| `PID_ROLL_INNER_KD` | 0.0f | Roll 增益 |
| `PID_ROLL_INNER_KI` | 0.0f | Roll 增益 |
| `PID_ROLL_INNER_KP` | 0.016f | Roll 增益 |
| `PID_ROLL_INNER_OUTPUT_MAX` | 60.0f | Roll 限幅 |
| `PID_ROLL_OUTER_KD` | 0.3f | Roll 增益 |
| `PID_ROLL_OUTER_KI` | 0.1f | Roll 增益 |
| `PID_ROLL_OUTER_KP` | 6.0f | Roll 增益 |
| `PID_TS` | 0.01s | 时序 |
| `PID_YAW_INNER_KD` | 0.0f | Yaw 增益 |
| `PID_YAW_INNER_KI` | 0.0f | Yaw 增益 |
| `PID_YAW_INNER_KP` | 0.015f | Yaw 增益 |
| `PID_YAW_INNER_OUTPUT_MAX` | 60.0f | Yaw 限幅 |
| `PID_YAW_OUTER_KD` | 0.5f | Yaw 增益 |
| `PID_YAW_OUTER_KI` | 1.5f | Yaw 增益 |
| `PID_YAW_OUTER_KP` | 6.0f | Yaw 增益 |
| `SERVO_PITCH_POS_MAX` | 1023 | Pitch 舵机 |
| `SERVO_PITCH_POS_MIN` | 0 | Pitch 舵机 |
| `SERVO_ROLL_POS_MAX` | 1023 | Roll 舵机 |
| `SERVO_ROLL_POS_MIN` | 0 | Roll 舵机 |
| `SERVO_YAW_POS_MAX` | 1023 | Yaw 舵机 |
| `SERVO_YAW_POS_MIN` | 0 | Yaw 舵机 |
