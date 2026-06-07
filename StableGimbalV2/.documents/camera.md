# ATK-MC7725F 摄像头模块 — 工程总结

> 平台: 正点原子 STM32F103 精英开发板
> 模块: ATK-MC7725F (OV7725 图像传感器)
> 通信协议: SCCB (类I2C), 8-bit 并行数据口 (FIFO读取)
> 分辨率: VGA 640x480 / QVGA 320x240

---

## 1. 工程文件结构

```
User/
├── main.c          — 程序入口, 硬件初始化 + demo_run()
├── demo.c          — 演示应用: 摄像头初始化/配置/取帧/LCD显示
└── demo.h          — demo_run() 声明

Drivers/BSP/ATK_MC7725F/
├── atk_mc7725f.h       — 摄像头驱动头文件: 引脚定义, 枚举, API声明
├── atk_mc7725f.c       — 摄像头驱动实现: 硬件初始化, SCCB寄存器配置, 取帧
├── atk_mc7725f_sccb.h  — SCCB总线头文件: SCL/SDA引脚, IO操作宏
├── atk_mc7725f_sccb.c  — SCCB总线实现: 软件模拟SCCB时序
└── atk_mc7725f_cfg.h   — OV7725 寄存器地址定义表

Drivers/BSP/LCD/       — LCD显示驱动 (FSMC接口)
Drivers/BSP/LED/       — LED控制
Drivers/BSP/KEY/       — 按键输入
Drivers/SYSTEM/        — 系统基础: sys, delay, usart
Drivers/STM32F1xx_HAL_Driver/ — ST HAL库
```

---

## 2. 整体逻辑链 (启动 → 运行)

```
main()
 │
 ├─ HAL_Init()                          // HAL库初始化, 配置SysTick
 ├─ sys_stm32_clock_init(RCC_PLL_MUL9)  // 配置系统时钟 72MHz
 ├─ delay_init(72)                       // 延时函数初始化
 ├─ usart_init(115200)                   // 串口初始化 (调试输出)
 ├─ led_init()                           // LED初始化
 ├─ key_init()                           // 按键初始化
 ├─ lcd_init()                           // LCD初始化 (FSMC接口)
 │
 └─ demo_run()                          // ★ 主应用入口
     │
     ├─ atk_mc7725f_init()              // ① 摄像头硬件+通信初始化
     │   ├─ atk_mc7725f_hw_init()       // GPIO引脚配置 (数据口+控制口+VSYNC中断)
     │   ├─ atk_mc7725f_sccb_init()     // SCCB总线初始化 (SCL+SDA)
     │   ├─ atk_mc7725f_sw_reset()      // 软件复位传感器 (COM7=0x80)
     │   ├─ atk_mc7725f_get_mid()       // 读取制造商ID (期望0x7FA2)
     │   ├─ atk_mc7725f_get_pid()       // 读取产品ID (期望0x7721)
     │   └─ atk_mc7725f_init_reg()      // 写入默认寄存器配置表 (60个寄存器)
     │
     ├─ demo_config_camera()            // ② 配置摄像头参数
     │   ├─ atk_mc7725f_set_output()         // 输出模式: QVGA 320x240
     │   ├─ atk_mc7725f_set_light_mode()     // 白平衡: 自动
     │   ├─ atk_mc7725f_set_color_saturation() // 色彩饱和度: 0(默认)
     │   ├─ atk_mc7725f_set_brightness()     // 亮度: 0(默认)
     │   ├─ atk_mc7725f_set_contrast()       // 对比度: 0(默认)
     │   └─ atk_mc7725f_set_special_effect() // 特殊效果: 正常
     │
     ├─ atk_mc7725f_enable_output()     // ③ 使能图像输出 (OE=0)
     │
     ├─ lcd_scan_dir(U2D_L2R)           // ④ LCD扫描方向: 上到下, 左到右
     ├─ lcd_set_window(0, 0, W, H)      // LCD设置显示窗口 (与摄像头输出匹配)
     ├─ lcd_write_ram_prepare()          // 准备写入LCD GRAM
     │
     └─ while(1)                        // ⑤ 主循环: 持续取帧
         └─ atk_mc7725f_get_frame(&LCD->LCD_RAM, NOINC)
                                        // 从FIFO读取一帧, 直接写入LCDGRAM
```

---

## 3. 硬件引脚映射

### 3.1 数据口 (8-bit 并行, 读取FIFO)

| 信号 | GPIO端口 | GPIO引脚 | 方向 |
|------|----------|----------|------|
| D0   | GPIOC    | PIN_0    | 输入 |
| D1   | GPIOC    | PIN_1    | 输入 |
| D2   | GPIOC    | PIN_2    | 输入 |
| D3   | GPIOC    | PIN_3    | 输入 |
| D4   | GPIOC    | PIN_4    | 输入 |
| D5   | GPIOC    | PIN_5    | 输入 |
| D6   | GPIOC    | PIN_6    | 输入 |
| D7   | GPIOC    | PIN_7    | 输入 |

> D0~D7 全部在同一GPIOC端口, 通过位掩码 0x00FF 一次性读取整字节

### 3.2 控制信号

| 信号 | GPIO端口 | GPIO引脚 | 功能 |
|------|----------|----------|------|
| WRST | GPIOD    | PIN_6    | 写指针复位 (控制FIFO写入位置) |
| RRST | GPIOG    | PIN_14   | 读指针复位 (控制FIFO读取位置) |
| OE   | GPIOG    | PIN_15   | 输出使能 (低有效, 控制数据口三态) |
| RCLK | GPIOB    | PIN_4    | 读时钟 (上升沿读取FIFO数据) |
| WEN  | GPIOB    | PIN_3    | 写使能 (高有效, 允许FIFO写入) |
| VSYNC | GPIOA   | PIN_8    | 帧同步中断 (上升沿触发EXTI) |

> 注意: PB3/PB4 默认为JTAG功能, 需要禁用JTAG才能用作普通GPIO
>       (通过 __HAL_AFIO_REMAP_SWJ_NOJTAG() 实现)

### 3.3 SCCB总线 (配置寄存器用)

| 信号 | GPIO端口 | GPIO引脚 |
|------|----------|----------|
| SCL  | GPIOD    | PIN_3    |
| SDA  | GPIOG    | PIN_13   |

> SCCB地址: 0x21 (左移1位后: 写=0x42, 读=0x43)

---

## 4. 函数调用关系图

### 4.1 初始化链

```
main()
 └→ atk_mc7725f_init()                    [atk_mc7725f.c]
     ├→ atk_mc7725f_hw_init()             [atk_mc7725f.c]  (static)
     │   ├→ HAL_RCC_GPIOD_CLK_ENABLE()    — 使能各GPIO时钟
     │   ├→ HAL_RCC_GPIOG_CLK_ENABLE()
     │   ├→ HAL_RCC_GPIOC_CLK_ENABLE()
     │   ├→ HAL_RCC_GPIOB_CLK_ENABLE()
     │   ├→ HAL_RCC_GPIOA_CLK_ENABLE()
     │   ├→ __HAL_RCC_AFIO_CLK_ENABLE()
     │   ├→ __HAL_AFIO_REMAP_SWJ_NOJTAG()
     │   ├→ HAL_GPIO_Init() × N           — 配置所有引脚
     │   └→ ATK_MC7725F_WRST/RRST/OE/...  — IO宏拉高各控制引脚
     │
     ├→ atk_mc7725f_sccb_init()           [atk_mc7725f_sccb.c]
     │   ├→ HAL_GPIO_Init(SCL)            — 推挽输出
     │   ├→ HAL_GPIO_Init(SDA)            — 开漏输出(带外部上拉)
     │   └→ atk_mc7725f_sccb_stop()       — 发送停止信号
     │
     ├→ atk_mc7725f_sw_reset()            [atk_mc7725f.c]  (static)
     │   ├→ atk_mc7725f_write_reg(COM7, 0x80)
     │   │   └→ atk_mc7725f_sccb_3_phase_write()
     │   │       ├→ atk_mc7725f_sccb_start()
     │   │       ├→ atk_mc7725f_sccb_write_byte() × 3
     │   │       └→ atk_mc7725f_sccb_stop()
     │   └→ delay_ms(2)
     │
     ├→ atk_mc7725f_get_mid()             [atk_mc7725f.c]  (static)
     │   ├→ atk_mc7725f_read_reg(MIDH)    — 读高字节
     │   │   ├→ atk_mc7725f_sccb_2_phase_write()
     │   │   └→ atk_mc7725f_sccb_2_phase_read()
     │   └→ atk_mc7725f_read_reg(MIDL)    — 读低字节
     │
     ├→ atk_mc7725f_get_pid()             [atk_mc7725f.c]  (static)
     │   └→ (同上, 读取PID和VER寄存器)
     │
     └→ atk_mc7725f_init_reg()            [atk_mc7725f.c]  (static)
         └→ for循环 × 60个寄存器
             └→ atk_mc7725f_write_reg(reg, val)
                 └→ atk_mc7725f_sccb_3_phase_write(0x21, reg, val)
```

### 4.2 配置链

```
demo_config_camera()                      [demo.c]  (static)
 ├→ atk_mc7725f_set_output(320, 240, QVGA)  [atk_mc7725f.c]
 │   ├→ 参数范围检查 (QVGA: ≤320x240)
 │   ├→ 写入COM7/HSTART/HSIZE/VSTRT/VSIZE/HREF等寄存器
 │   │   ├→ atk_mc7725f_write_reg() × N
 │   │   └→ atk_mc7725f_read_reg()  × N     — 读回HSTART/VSTRT/HREF做偏移计算
 │   └→ 更新 g_atk_mc7725f_sta.output.width/height
 │
 ├→ atk_mc7725f_set_light_mode(AUTO)      [atk_mc7725f.c]
 │   └→ atk_mc7725f_write_reg() × 4~6
 │
 ├→ atk_mc7725f_set_color_saturation(4)   [atk_mc7725f.c]
 │   └→ atk_mc7725f_write_reg(USAT, 0x40) + atk_mc7725f_write_reg(VSAT, 0x40)
 │
 ├→ atk_mc7725f_set_brightness(4)         [atk_mc7725f.c]
 │   └→ atk_mc7725f_write_reg(BRIGHT, 0x08) + atk_mc7725f_write_reg(SIGN, 0x06)
 │
 ├→ atk_mc7725f_set_contrast(4)           [atk_mc7725f.c]
 │   └→ atk_mc7725f_write_reg(CNST, 0x20)
 │
 └→ atk_mc7725f_set_special_effect(NORMAL) [atk_mc7725f.c]
     └→ atk_mc7725f_write_reg(SDE, 0x06) + UFIX/VFIX各写0x80
```

### 4.3 取帧链 (主循环, 每帧调用一次)

```
while(1) {
  atk_mc7725f_get_frame(&LCD->LCD_RAM, NOINC)  [atk_mc7725f.c]
   │
   ├─ __disable_irq()                     // 进临界区
   ├─ 检查 g_atk_mc7725f_sta.frame.handle_flag
   │   └─ FRAME_HANDLE_DONE → 返回 EEMPTY (无新帧)
   ├─ 置 handle_flag = FRAME_HANDLE_DONE  // 立即清除标志
   ├─ __enable_irq()                      // 出临界区
   │
   ├─ 读取输出宽高到局部变量 (减少volatile访问)
   │
   ├─ RRST=0 → RCLK=0 → RCLK=1 → RCLK=0 → RRST=1 → RCLK=1
   │   // FIFO读指针复位, 准备读取
   │
   ├─ for each pixel (height × width):
   │   ├─ RCLK=0
   │   ├─ dat = atk_mc7725f_get_byte_data() << 8   // 读高8位
   │   ├─ RCLK=1
   │   ├─ RCLK=0
   │   ├─ dat |= atk_mc7725f_get_byte_data()       // 读低8位
   │   ├─ RCLK=1
   │   └─ *dts = dat                                // NOINC: 地址不变(LCD自动递增)
   │
   └─ g_atk_mc7725f_sta.frame.count++    // 帧计数+1
}
```

### 4.4 中断服务 (VSYNC帧同步)

```
EXTI9_5_IRQHandler()
 └→ ATK_MC7725F_VSYNC_INT_IRQHandler()   [atk_mc7725f.c]
     │
     ├─ 检查 EXTI中断标志位
     │
     ├─ if handle_flag == FRAME_HANDLE_DONE:
     │   ├─ WRST=0 → WEN=1 → WRST=1    // 复位FIFO写指针, 开启写入
     │   └─ handle_flag = FRAME_HANDLE_PEND
     │
     └─ else (上一帧还在读取):
         └─ WEN=0                       // 禁止写入, 丢弃当前帧 (防止数据覆盖)
```

---

## 5. SCCB通信协议详解

SCCB (Serial Camera Control Bus) 是OV系列传感器的专有协议, 与I2C类似但更简化。

### 5.1 时序操作

```
atk_mc7725f_sccb_start()     — SDA: 1→0 (SCL=1时)
atk_mc7725f_sccb_stop()      — SDA: 0→1 (SCL=1时)
atk_mc7725f_sccb_write_byte() — MSB先发, 每位在SCL上升沿采样, 共8+1(ACK)位
atk_mc7725f_sccb_read_byte()  — MSB先收, 每位在SCL高电平中间采样, 共8+1(ACK)位
```

### 5.2 三种传输模式

| 函数 | 阶段 | 用途 |
|------|------|------|
| `sccb_3_phase_write(id, sub, dat)` | ID(写) + 寄存器地址 + 数据 | 写寄存器 |
| `sccb_2_phase_write(id, sub)` | ID(写) + 寄存器地址 | 设置读取地址 |
| `sccb_2_phase_read(id, dat*)` | ID(读) + 读取数据 | 读寄存器 |

### 5.3 读寄存器流程 (两步)

```
read_reg(reg):
  1. sccb_2_phase_write(0x21, reg)   // 设置目标寄存器地址
  2. sccb_2_phase_read(0x21, &dat)   // 读取数据
```

---

## 6. FIFO帧缓冲机制

ATK-MC7725F模块自带AL422B FIFO芯片, 用于解决STM32F103无法实时处理OV7725高速数据流的问题。

### 6.1 写入流程 (VSYNC中断触发)

```
VSYNC上升沿 (一帧开始)
 │
 ├─ 上一帧已处理完 (FRAME_HANDLE_DONE):
 │   ├─ WRST↓WRST↑   — 复位FIFO写指针到起始位置
 │   └─ WEN↑          — 使能FIFO写入, OV7725数据自动写入FIFO
 │   handle_flag = FRAME_HANDLE_PEND
 │
 └─ 上一帧未处理完:
     └─ WEN↓          — 禁止写入, 丢弃本帧
```

### 6.2 读取流程 (主循环轮询)

```
get_frame() 被调用
 │
 ├─ 检查 handle_flag == FRAME_HANDLE_DONE ?
 │   ├─ 是 → 返回 EEMPTY (无新帧可读)
 │   └─ 否 → 继续
 │
 ├─ 关中断 → 清除handle_flag → 开中断 (原子操作, 防止重入)
 │
 ├─ RRST↓RCLK↓RCLK↑RCLK↓RRST↑RCLK↑  — 复位FIFO读指针
 │
 └─ 双字节读取循环:
     RCLK↓ → 读高8位 → RCLK↑ → RCLK↓ → 读低8位 → RCLK↑ → 写入目标
```

### 6.3 状态管理

```c
static volatile struct {
    struct {
        uint16_t width;              // 当前输出宽度
        uint16_t height;             // 当前输出高度
    } output;
    struct {
        enum {
            FRAME_HANDLE_DONE = 0,   // 帧已处理/空闲
            FRAME_HANDLE_PEND,       // 有新帧等待处理
        } handle_flag;
        uint16_t count;              // 已读取帧计数
    } frame;
} g_atk_mc7725f_sta;
```

> volatile 关键字: 该结构体在中断(VSYNC)和主循环中同时访问
> 临界区保护: get_frame() 中用 __disable_irq/__enable_irq 保护 handle_flag 检查

---

## 7. 图像数据流向

```
OV7725传感器 (原始像素数据)
     │
     ▼
AL422B FIFO芯片 (384KB缓冲)
     │
     ├─ 写入端: OV7725 → FIFO (由VSYNC中断控制)
     │
     └─ 读取端: FIFO → STM32 GPIOC[7:0]
                  │
                  ▼
        atk_mc7725f_get_frame()
                  │  (像素16-bit RGB565)
                  ▼
        LCD->LCD_RAM (FSMC地址映射, 直接写入LCD GRAM)
                  │
                  ▼
           LCD屏幕实时显示
```

---

## 8. 关键枚举与参数定义

### 8.1 输出模式
- `ATK_MC7725F_OUTPUT_MODE_VGA`  — 640×480
- `ATK_MC7725F_OUTPUT_MODE_QVGA` — 320×240

### 8.2 白平衡模式
- AUTO / SUNNY / CLOUDY / OFFICE / HOME / NIGHT

### 8.3 图像参数 (均9级可调, _4 = 默认/中间值)
- 色彩饱和度: _0(+4) ~ _8(-4)
- 亮度: _0(+4) ~ _8(-4)
- 对比度: _0(+4) ~ _8(-4)

### 8.4 特殊效果
- NORMAL / BW(黑白) / BLUISH(偏蓝) / SEPIA(复古) / REDISH(偏红) / GREENISH(偏绿) / NEGATIVE(负片)

---

## 9. IO操作性能优化

### 9.1 数据口 (同一GPIO端口)

```c
// 优化后: 一次读取整个端口
dat = ATK_MC7725F_DATE_GPIO_PORT->IDR & ATK_MC7725F_DATA_READ_MASK;
// 等价于: 读取 GPIOC->IDR 的低8位
```

### 9.2 控制信号 (直接寄存器操作)

```c
// 用 BSRR/BRR 寄存器替代 HAL_GPIO_WritePin()
ATK_MC7725F_WRST(x)  →  x ? BSRR=PIN : BRR=PIN
// 消除了函数调用开销, 在高速取帧循环中至关重要
```

### 9.3 volatile变量缓存

```c
// 在get_frame()中, 将volatile全局变量缓存到局部变量
cur_width  = g_atk_mc7725f_sta.output.width;
cur_height = g_atk_mc7725f_sta.output.height;
// 避免循环中每次访问都生成内存读取指令
```

---

## 10. 错误码定义

| 错误码 | 值 | 含义 |
|--------|-----|------|
| `ATK_MC7725F_EOK` | 0 | 操作成功 |
| `ATK_MC7725F_ERROR` | 1 | 通用错误 (如ID不匹配) |
| `ATK_MC7725F_EINVAL` | 2 | 参数无效 |
| `ATK_MC7725F_EEMPTY` | 3 | 无帧可读 (FIFO为空) |

---

## 11. OV7725关键寄存器速查

| 寄存器 | 地址 | 功能 |
|--------|------|------|
| COM7   | 0x12 | 软件复位, 输出模式选择(VGA/QVGA) |
| COM8   | 0x13 | 白平衡控制 |
| HSTART | 0x17 | 水平起始位置 |
| HSIZE  | 0x18 | 水平尺寸 (×4) |
| VSTRT  | 0x19 | 垂直起始位置 |
| VSIZE  | 0x1A | 垂直尺寸 (×2) |
| HOutSize | 0x29 | 水平输出尺寸 (×4) |
| VOutSize | 0x2C | 垂直输出尺寸 (×2) |
| HREF   | 0x32 | HSTART/HSIZE/VSTRT/VSIZE的低位 |
| BRIGHT | 0x9B | 亮度控制 |
| CNST   | 0x9C | 对比度控制 |
| USAT   | 0xA7 | U色差饱和度 |
| VSAT   | 0xA8 | V色差饱和度 |
| SDE    | 0xA6 | 特殊效果使能 |
| UFIX   | 0x60 | U色差固定值 |
| VFIX   | 0x61 | V色差固定值 |

> 注意: HSTART左移2位 = 像素起始列, HSIZE左移2位 = 水平像素数
>       VSIZE左移1位 = 垂直像素数
