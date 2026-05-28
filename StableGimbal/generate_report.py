"""
StableGimbal 工程报告生成脚本
"""
from docx import Document
from docx.shared import Inches, Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.style import WD_STYLE_TYPE
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import datetime

doc = Document()

# ── 全局样式设置 ──────────────────────────────────────────────────
style = doc.styles['Normal']
font = style.font
font.name = 'Microsoft YaHei'
font.size = Pt(11)
style.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')

for level in range(1, 4):
    hs = doc.styles[f'Heading {level}']
    hs.font.name = 'Microsoft YaHei'
    hs.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
    hs.font.color.rgb = RGBColor(0x1A, 0x3C, 0x6D)

# ── 辅助函数 ─────────────────────────────────────────────────────
def add_table_row(table, cells_data, bold=False, bg_color=None):
    row = table.add_row()
    for i, text in enumerate(cells_data):
        cell = row.cells[i]
        cell.text = ''
        p = cell.paragraphs[0]
        run = p.add_run(str(text))
        run.font.size = Pt(9.5)
        run.font.name = 'Microsoft YaHei'
        run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
        if bold:
            run.bold = True
        if bg_color:
            shading = OxmlElement('w:shd')
            shading.set(qn('w:fill'), bg_color)
            shading.set(qn('w:val'), 'clear')
            cell._tc.get_or_add_tcPr().append(shading)
    return row

def set_table_header(table, headers, bg_color='1A3C6D'):
    """设置表头样式"""
    hdr = table.rows[0]
    for i, text in enumerate(headers):
        cell = hdr.cells[i]
        cell.text = ''
        p = cell.paragraphs[0]
        run = p.add_run(text)
        run.font.size = Pt(10)
        run.font.name = 'Microsoft YaHei'
        run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
        run.bold = True
        run.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
        shading = OxmlElement('w:shd')
        shading.set(qn('w:fill'), bg_color)
        shading.set(qn('w:val'), 'clear')
        cell._tc.get_or_add_tcPr().append(shading)

def add_code_block(doc, code_text):
    """添加代码块"""
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.left_indent = Cm(1)
    run = p.add_run(code_text)
    run.font.name = 'Consolas'
    run.font.size = Pt(9)
    run.font.color.rgb = RGBColor(0x2D, 0x2D, 0x2D)
    # 添加灰色背景
    shading = OxmlElement('w:shd')
    shading.set(qn('w:fill'), 'F0F0F0')
    shading.set(qn('w:val'), 'clear')
    run.element.rPr.append(shading)


# ══════════════════════════════════════════════════════════════════
#  封面
# ══════════════════════════════════════════════════════════════════
for _ in range(6):
    doc.add_paragraph()

title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = title.add_run('StableGimbal 云台稳定系统')
run.font.size = Pt(28)
run.font.name = 'Microsoft YaHei'
run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
run.font.color.rgb = RGBColor(0x1A, 0x3C, 0x6D)
run.bold = True

subtitle = doc.add_paragraph()
subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = subtitle.add_run('嵌入式固件工程分析报告')
run.font.size = Pt(18)
run.font.color.rgb = RGBColor(0x66, 0x66, 0x66)

doc.add_paragraph()

info = doc.add_paragraph()
info.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = info.add_run(f'报告日期：{datetime.date.today().strftime("%Y年%m月%d日")}')
run.font.size = Pt(12)
run.font.color.rgb = RGBColor(0x99, 0x99, 0x99)

info2 = doc.add_paragraph()
info2.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = info2.add_run('MCU: STM32F103RB  |  IDE: VS Code + CMake  |  HAL: STM32CubeMX')
run.font.size = Pt(10)
run.font.color.rgb = RGBColor(0x99, 0x99, 0x99)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  目录
# ══════════════════════════════════════════════════════════════════
doc.add_heading('目录', level=1)
toc_items = [
    '1. 项目概述',
    '2. 工程结构分析',
    '3. BSP 驱动层架构',
    '4. RX28 与 AX-12A 驱动合并分析',
    '5. 串级 PID 控制系统设计',
    '6. 传感器融合策略',
    '7. 外设资源分配',
    '8. 编译信息',
    '9. 优化建议',
]
for item in toc_items:
    p = doc.add_paragraph(item)
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    p.paragraph_format.left_indent = Cm(1)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  1. 项目概述
# ══════════════════════════════════════════════════════════════════
doc.add_heading('1. 项目概述', level=1)

doc.add_paragraph(
    'StableGimbal 是一个基于 STM32F103RB 的三轴云台稳定系统。'
    '系统使用 JY901S 九轴 IMU 获取姿态信息，通过串级 PID 算法计算补偿量，'
    '驱动 Dynamixel 兼容舵机（RX28 / AX-12A）实现三轴（俯仰、横滚、航向）稳定控制。'
)

doc.add_heading('1.1 硬件平台', level=2)
hw_table = doc.add_table(rows=1, cols=3)
hw_table.style = 'Table Grid'
hw_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(hw_table, ['组件', '型号/规格', '用途'])
hw_data = [
    ['主控芯片', 'STM32F103RB (Cortex-M3, 72MHz)', '系统主控制器'],
    ['IMU 传感器', 'JY901S 九轴 (UART)', '姿态角度和角速度测量'],
    ['舵机执行器', 'RX28 / AX-12A (Dynamixel Protocol 1.0)', '三轴驱动（俯仰/横滚/航向）'],
    ['通信总线', 'USART3 @ 1Mbps + RS485', 'Dynamixel 舵机总线'],
    ['调试串口', 'USART1 @ 115200', 'printf 调试输出'],
    ['IMU 串口', 'USART2 @ 115200', 'JY901S 数据接收'],
    ['指示灯', 'LED (PB8)', '系统状态指示'],
]
for row_data in hw_data:
    add_table_row(hw_table, row_data)

doc.add_heading('1.2 软件架构', level=2)
doc.add_paragraph(
    '项目采用分层架构设计，基于 STM32CubeMX 生成的 HAL 库代码，'
    '在 USER CODE 区域编写应用层和 BSP 驱动层代码，确保 CubeMX 重新生成代码时不会丢失自定义逻辑。'
)

arch_items = [
    '应用层 (main.c) — 系统初始化、主循环控制逻辑',
    'BSP 驱动层 (BSP/) — 外设驱动抽象（IMU、舵机、PID、LED）',
    'HAL 库层 (Drivers/) — STM32 HAL/LL 驱动（CubeMX 生成）',
    '启动/链接文件 — startup_stm32f103xb.s, STM32F103XX_FLASH.ld',
]
for item in arch_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  2. 工程结构分析
# ══════════════════════════════════════════════════════════════════
doc.add_heading('2. 工程结构分析', level=1)

doc.add_heading('2.1 目录树', level=2)
tree_text = """StableGimbal/
├── CMakeLists.txt              # 顶层 CMake 构建文件
├── CMakePresets.json            # CMake 预设配置
├── startup_stm32f103xb.s       # ARM 启动文件
├── STM32F103XX_FLASH.ld        # 链接脚本
├── TEST.ioc                    # STM32CubeMX 工程文件
├── BSP/
│   ├── Inc/
│   │   ├── bsp_jy901s.h        # JY901S IMU 驱动
│   │   ├── bsp_pid.h           # PID 控制器（单级 + 串级）
│   │   ├── bsp_servo_rx28.h    # RX28 舵机驱动
│   │   ├── bsp_ax12a.h         # AX-12A 反馈驱动
│   │   └── bsp_led.h           # LED 控制
│   └── Src/
│       ├── bsp_jy901s.c
│       ├── bsp_pid.c
│       ├── bsp_servo_rx28.c
│       └── bsp_ax12a.c
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── stm32f1xx_it.h
│   └── Src/
│       ├── main.c              # 主程序（USER CODE 区域）
│       ├── stm32f1xx_it.c      # 中断服务函数
│       ├── stm32f1xx_hal_msp.c # MSP 初始化
│       ├── system_stm32f1xx.c
│       ├── syscalls.c
│       └── sysmem.c
├── Drivers/                    # STM32 HAL 库（CubeMX 生成）
├── cmake/
│   ├── gcc-arm-none-eabi.cmake
│   └── stm32cubemx/CMakeLists.txt
└── build/Debug/                # 构建输出"""
add_code_block(doc, tree_text)

doc.add_heading('2.2 构建系统', level=2)
doc.add_paragraph(
    '使用 CMake + Ninja 构建，交叉编译工具链为 arm-none-eabi-gcc (14.3.1)。'
    'CubeMX 生成的代码放在 cmake/stm32cubemx/ 子目录，用户 BSP 代码在顶层 CMakeLists.txt 中添加。'
)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  3. BSP 驱动层架构
# ══════════════════════════════════════════════════════════════════
doc.add_heading('3. BSP 驱动层架构', level=1)

doc.add_heading('3.1 模块列表', level=2)
bsp_table = doc.add_table(rows=1, cols=4)
bsp_table.style = 'Table Grid'
bsp_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(bsp_table, ['模块', '文件', '功能', '依赖'])
bsp_data = [
    ['JY901S IMU', 'bsp_jy901s.h/.c', '九轴 IMU 数据解析（角度+角速度）', 'USART2, HAL UART IT'],
    ['PID 控制器', 'bsp_pid.h/.c', '单级 PID + 串级 PID + 传感器融合', 'math.h'],
    ['RX28 舵机', 'bsp_servo_rx28.h/.c', 'Dynamixel 舵机控制（写位置/读位置/同步写）', 'USART3, RS485 GPIO'],
    ['AX-12A 反馈', 'bsp_ax12a.h/.c', 'AX-12A 舵机反馈读取（位置+速度+负载）', 'USART3, RS485 GPIO'],
    ['LED 控制', 'bsp_led.h', 'LED 开关/翻转（宏定义）', 'GPIO PB8'],
]
for row_data in bsp_data:
    add_table_row(bsp_table, row_data)

doc.add_heading('3.2 数据流', level=2)
doc.add_paragraph('系统的数据流路径如下：')
flow_text = """JY901S (USART2) ──中断接收──→ g_jy901s.pitch/roll/yaw/wx/wy/wz
                                      │
                                      ▼
                              传感器融合 (SensorFusion)
                                      │
                              AX-12A (USART3) ──主动读取──→ g_ax12a_*.angle/speed
                                      │
                                      ▼
                              串级 PID (CascadePID)
                                      │
                              ┌───────┴───────┐
                              │ 外环: 角度PID  │
                              │ 内环: 角速度PID │
                              └───────┬───────┘
                                      │
                                      ▼
                              舵机控制 (Servo_SetPosition)
                                      │
                              RX28/AX-12A (USART3) ──RS485──→ 舵机执行"""
add_code_block(doc, flow_text)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  4. RX28 与 AX-12A 驱动合并分析
# ══════════════════════════════════════════════════════════════════
doc.add_heading('4. RX28 与 AX-12A 驱动合并分析', level=1)

doc.add_heading('4.1 协议对比', level=2)
doc.add_paragraph(
    'RX28 和 AX-12A 均使用 Dynamixel Protocol 1.0，协议格式完全相同。'
    '以下是两者的关键参数对比：'
)

cmp_table = doc.add_table(rows=1, cols=4)
cmp_table.style = 'Table Grid'
cmp_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(cmp_table, ['特性', 'RX28', 'AX-12A', '兼容性'])
cmp_data = [
    ['通信协议', 'Dynamixel Protocol 1.0', 'Dynamixel Protocol 1.0', '✅ 完全相同'],
    ['物理接口', 'RS485 TTL (半双工)', 'TTL (半双工)', '✅ 相同总线'],
    ['数据包格式', 'FF FF ID LEN INST [data] CHK', 'FF FF ID LEN INST [data] CHK', '✅ 完全相同'],
    ['角度范围', '0~300° (1023 步)', '0~300° (1023 步)', '✅ 完全相同'],
    ['位置分辨率', '~0.293°', '~0.29°', '✅ 基本相同'],
    ['目标位置地址', '0x1E (30)', '0x1E (30)', '✅ 相同'],
    ['当前位置地址', '0x24 (36)', '0x24 (36)', '✅ 相同'],
    ['当前速度地址', '—', '0x26 (38)', '⚠️ AX-12A 额外读取'],
    ['当前负载地址', '—', '0x28 (40)', '⚠️ AX-12A 额外读取'],
    ['同步写指令', '0x83', '0x83', '✅ 相同'],
    ['读指令', '0x02', '0x02', '✅ 相同'],
    ['写指令', '0x03', '0x03', '✅ 相同'],
]
for row_data in cmp_data:
    add_table_row(cmp_table, row_data)

doc.add_heading('4.2 当前代码重叠分析', level=2)
doc.add_paragraph('两个驱动存在以下重叠和耦合问题：')

overlap_items = [
    '校验和计算：两者的 checksum 逻辑完全相同（~(ID+LEN+INST+...)）',
    '帧头构造：都是 FF FF 开头的 Dynamixel 标准帧',
    'RS485 方向控制：bsp_ax12a.c 已经 #include "bsp_servo_rx28.h" 来获取 RS485 宏',
    '发送缓冲区：各自维护独立的静态 tx_buf，存在潜在总线冲突风险',
    'UART 句柄：两者共享 USART3，但各自独立管理，缺乏统一的总线锁',
]
for item in overlap_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_heading('4.3 结论：建议合并', level=2)

# 高亮结论框
p = doc.add_paragraph()
run = p.add_run('▶ 结论：强烈建议将 bsp_servo_rx28 和 bsp_ax12a 合并为统一的 bsp_dynamixel 驱动模块。')
run.bold = True
run.font.size = Pt(12)
run.font.color.rgb = RGBColor(0xCC, 0x33, 0x00)

doc.add_paragraph('合并理由：')

reasons = [
    '协议完全兼容：两者都使用 Dynamixel Protocol 1.0，控制表地址一致，指令集相同',
    '同一物理总线：RX28 和 AX-12A 共享 USART3 + RS485，统一驱动可避免总线竞争',
    '消除循环依赖：当前 bsp_ax12a.c 依赖 bsp_servo_rx28.h 的 RS485 宏，这是不合理的耦合',
    '代码复用：校验和计算、帧构造、发送/接收逻辑可统一为内部函数',
    '扩展性：统一驱动更容易支持新的 Dynamixel 舵机型号（如 MX 系列、XL 系列）',
    '维护成本：一个驱动比两个驱动更容易维护和调试',
]
for item in reasons:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_heading('4.4 合并方案', level=2)
doc.add_paragraph('建议的统一驱动架构：')

merge_text = """bsp_dynamixel.h / bsp_dynamixel.c
│
├── 类型定义
│   ├── Dynamixel_t          // 统一实例结构体
│   ├── Dynamixel_Data_t     // 反馈数据结构体
│   └── Dynamixel_ServoType  // 舵机型号枚举 (RX28, AX12A, MX, ...)
│
├── 控制函数（写操作）
│   ├── Dynamixel_Init()
│   ├── Dynamixel_SetPosition()       // 目标位置
│   ├── Dynamixel_SetSpeed()          // 目标速度
│   └── Dynamixel_SyncWrite()         // 同步写多个舵机
│
├── 反馈函数（读操作）
│   ├── Dynamixel_ReadPosition()      // 读取当前位置
│   ├── Dynamixel_ReadSpeed()         // 读取当前速度
│   ├── Dynamixel_ReadLoad()          // 读取当前负载
│   └── Dynamixel_ReadFeedback()      // 一次读取全部反馈
│
├── 工具函数
│   ├── Dynamixel_Ping()              // 检测在线状态
│   └── Dynamixel_SetBaudrate()       // 设置波特率
│
└── 内部函数（static）
    ├── dxl_send_packet()             // 统一发送
    ├── dxl_receive_packet()          // 统一接收
    └── dxl_checksum()                // 统一校验和"""
add_code_block(doc, merge_text)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  5. 串级 PID 控制系统设计
# ══════════════════════════════════════════════════════════════════
doc.add_heading('5. 串级 PID 控制系统设计', level=1)

doc.add_heading('5.1 控制架构', level=2)
doc.add_paragraph(
    '采用串级 PID（Cascade PID）控制策略，外环控制角度，内环控制角速度，'
    '相比单级 PID 具有更好的动态响应和抗干扰能力。'
)

pid_arch = """目标角度 (setpoint)
        │
        ▼
  ┌─────────────┐
  │  外环 PID    │  输入: 角度误差 (目标 - 融合角度)
  │  (角度环)    │  输出: 角速度指令 (°/s)
  │  Kp Ki Kd   │
  └──────┬──────┘
         │ velocity_cmd
         ▼
  ┌─────────────┐
  │  内环 PID    │  输入: 角速度误差 (指令 - 融合角速度)
  │ (角速度环)   │  输出: 舵机位置偏移量 (°)
  │  Kp Ki Kd   │
  └──────┬──────┘
         │ servo_offset
         ▼
  舵机目标位置 = 中位 + servo_offset"""
add_code_block(doc, pid_arch)

doc.add_heading('5.2 优势分析', level=2)
advantages = [
    '快速响应：内环（角速度环）带宽高，能快速抑制角速度扰动',
    '抗干扰：外扰（如风力、振动）首先被内环抑制，对外环影响小',
    '参数解耦：外环调角度响应，内环调速度响应，调参更直观',
    '限幅灵活：外环输出限幅 = 内环最大角速度指令，防止过冲',
]
for item in advantages:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_heading('5.3 PID 参数（初始值）', level=2)
pid_table = doc.add_table(rows=1, cols=8)
pid_table.style = 'Table Grid'
pid_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(pid_table, ['轴', '外环Kp', '外环Ki', '外环Kd', '内环Kp', '内环Ki', '内环Kd', '备注'])
pid_data = [
    ['Pitch', '2.0', '0.05', '0.8', '0.5', '0.02', '0.1', '俯仰轴'],
    ['Roll',  '2.0', '0.05', '0.8', '0.5', '0.02', '0.1', '横滚轴'],
    ['Yaw',   '1.5', '0.03', '0.6', '0.4', '0.02', '0.08', '航向轴'],
]
for row_data in pid_data:
    add_table_row(pid_table, row_data)

doc.add_heading('5.4 限幅参数', level=2)
lim_table = doc.add_table(rows=1, cols=7)
lim_table.style = 'Table Grid'
lim_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(lim_table, ['轴', '外环死区(°)', '外环积分限幅', '外环输出限幅(°/s)', '内环死区(°/s)', '内环积分限幅', '内环输出限幅(°)'])
lim_data = [
    ['Pitch', '0.3', '100', '150', '2.0', '200', '40'],
    ['Roll',  '0.5', '100', '150', '2.0', '200', '40'],
    ['Yaw',   '0.5', '80',  '100', '2.0', '150', '30'],
]
for row_data in lim_data:
    add_table_row(lim_table, row_data)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  6. 传感器融合策略
# ══════════════════════════════════════════════════════════════════
doc.add_heading('6. 传感器融合策略', level=1)

doc.add_heading('6.1 数据源', level=2)
src_table = doc.add_table(rows=1, cols=4)
src_table.style = 'Table Grid'
src_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(src_table, ['数据源', '角度', '角速度', '特点'])
src_data = [
    ['JY901S IMU', 'pitch/roll/yaw (°)', 'wx/wy/wz (°/s)', '高频更新，低漂移，但有振动噪声'],
    ['AX-12A 舵机', 'Present Position → °', 'Present Speed → °/s', '低频更新，无振动噪声，但有通信延迟'],
]
for row_data in src_data:
    add_table_row(src_table, row_data)

doc.add_heading('6.2 融合模式', level=2)
doc.add_paragraph('系统支持三种传感器数据源模式，可在运行时切换：')

mode_table = doc.add_table(rows=1, cols=3)
mode_table.style = 'Table Grid'
mode_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(mode_table, ['模式', '说明', '适用场景'])
mode_data = [
    ['SENSOR_SOURCE_IMU_ONLY', '仅使用 IMU 数据', 'AX-12A 未连接或通信异常时'],
    ['SENSOR_SOURCE_AX12A_ONLY', '仅使用 AX-12A 反馈', '需要低噪声角度反馈时（IMU 振动大）'],
    ['SENSOR_SOURCE_FUSION', '加权融合（默认）', '综合 IMU 高频和 AX-12A 低噪声优势'],
]
for row_data in mode_data:
    add_table_row(mode_table, row_data)

doc.add_heading('6.3 融合权重', level=2)
doc.add_paragraph('默认融合权重配置：')
w_items = [
    'Pitch 轴：IMU 70% + AX-12A 30%（俯仰角 IMU 精度较高）',
    'Roll 轴：IMU 70% + AX-12A 30%（横滚角 IMU 精度较高）',
    'Yaw 轴：IMU 60% + AX-12A 40%（航向角 AX-12A 反馈更稳定）',
]
for item in w_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_paragraph(
    '融合公式：fused = imu_weight × imu_value + (1 - imu_weight) × ax12a_value。'
    '当 AX-12A 数据无效（通信超时）时，自动回退到仅使用 IMU 数据。'
)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  7. 外设资源分配
# ══════════════════════════════════════════════════════════════════
doc.add_heading('7. 外设资源分配', level=1)

periph_table = doc.add_table(rows=1, cols=4)
periph_table.style = 'Table Grid'
periph_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(periph_table, ['外设', '引脚', '配置', '用途'])
periph_data = [
    ['USART1', 'PA9(TX)/PA10(RX)', '115200, 8N1', '调试串口 (printf)'],
    ['USART2', 'PA2(TX)/PA3(RX)', '115200, 8N1', 'JY901S IMU 数据接收 (中断)'],
    ['USART3', 'PB10(TX)/PB11(RX)', '1Mbps, 8N1', 'Dynamixel 舵机总线'],
    ['SPI1', 'PA4(NSS)/PA5(SCK)/PA6(MISO)/PA7(MOSI)', 'Master, 16bit', '预留扩展'],
    ['GPIO PB12', 'Output PP', 'RS485 方向控制', 'TX Enable/Disable'],
    ['GPIO PB8', 'Output PP', 'LED 指示灯', '系统状态'],
    ['GPIO PA4', 'Output PP', 'SPI1 NSS', '片选信号'],
    ['GPIO PB13/14/15', 'Input', '无上下拉', '预留输入'],
]
for row_data in periph_data:
    add_table_row(periph_table, row_data)

doc.add_heading('7.1 时钟配置', level=2)
clock_items = [
    'HSE: 8MHz 外部晶振',
    'PLL: HSE × 9 = 72MHz 系统时钟',
    'AHB: 72MHz (不分频)',
    'APB1: 36MHz (2分频) — USART2/USART3',
    'APB2: 72MHz (不分频) — USART1/SPI1',
]
for item in clock_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  8. 编译信息
# ══════════════════════════════════════════════════════════════════
doc.add_heading('8. 编译信息', level=1)

build_table = doc.add_table(rows=1, cols=2)
build_table.style = 'Table Grid'
build_table.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(build_table, ['项目', '值'])
build_data = [
    ['编译器', 'arm-none-eabi-gcc 14.3.1'],
    ['构建系统', 'CMake 3.22+ / Ninja'],
    ['编译标准', 'GNU C11'],
    ['优化级别', '-O0 (Debug)'],
    ['RAM 使用', '3000 B / 20 KB (14.65%)'],
    ['FLASH 使用', '21600 B / 64 KB (32.96%)'],
    ['输出文件', 'TEST.elf / TEST.hex / TEST.bin'],
    ['编译状态', '✅ 无错误，无警告'],
]
for row_data in build_data:
    add_table_row(build_table, row_data)

doc.add_heading('8.1 源文件编译列表', level=2)
src_table2 = doc.add_table(rows=1, cols=2)
src_table2.style = 'Table Grid'
src_table2.alignment = WD_TABLE_ALIGNMENT.CENTER
set_table_header(src_table2, ['源文件', '类型'])
src_data2 = [
    ['Core/Src/main.c', '应用层'],
    ['Core/Src/stm32f1xx_it.c', '中断服务 (CubeMX)'],
    ['Core/Src/stm32f1xx_hal_msp.c', 'MSP 初始化 (CubeMX)'],
    ['Core/Src/system_stm32f1xx.c', '系统初始化 (CubeMX)'],
    ['BSP/Src/bsp_jy901s.c', 'BSP 驱动'],
    ['BSP/Src/bsp_pid.c', 'BSP 驱动'],
    ['BSP/Src/bsp_servo_rx28.c', 'BSP 驱动'],
    ['BSP/Src/bsp_ax12a.c', 'BSP 驱动'],
    ['startup_stm32f103xb.s', '启动文件'],
]
for row_data in src_data2:
    add_table_row(src_table2, row_data)

doc.add_page_break()


# ══════════════════════════════════════════════════════════════════
#  9. 优化建议
# ══════════════════════════════════════════════════════════════════
doc.add_heading('9. 优化建议', level=1)

doc.add_heading('9.1 高优先级', level=2)
high_items = [
    '合并 bsp_servo_rx28 和 bsp_ax12a 为统一的 bsp_dynamixel 驱动',
    '添加串级 PID 参数在线调参接口（串口命令或按键）',
    '实现 PID 参数的 Flash 持久化存储',
    '添加舵机通信超时和错误恢复机制',
]
for item in high_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_heading('9.2 中优先级', level=2)
mid_items = [
    '使用 Sync Write 一次性设置三个舵机，减少总线占用时间',
    '使用 Sync Read 一次性读取三个舵机反馈（Dynamixel Protocol 1.0 支持）',
    '添加 JY901S 数据有效性校验（CRC 或范围检查）',
    '优化主循环频率：当前受 IMU 更新率限制，可考虑定时器触发',
]
for item in mid_items:
    p = doc.add_paragraph(item, style='List Bullet')

doc.add_heading('9.3 低优先级', level=2)
low_items = [
    '添加看门狗（IWDG）防止系统死锁',
    '实现 OTA 固件升级功能',
    '添加 SD 卡数据记录功能',
    '考虑升级到 STM32F4 系列以获得 FPU 硬件浮点加速',
]
for item in low_items:
    p = doc.add_paragraph(item, style='List Bullet')

# ── 保存 ─────────────────────────────────────────────────────────
output_path = r'c:\Users\PC\Desktop\STM32\StableGimbal\StableGimbal_工程报告.docx'
doc.save(output_path)
print(f'报告已生成: {output_path}')
