# 2026 全国大学生电子设计竞赛 H 题小车平衡系统

## 1. 项目简介

本项目采用 MaixCam Pro + YOLO26 视觉识别钢球位置，由 STM32F407VET6 作为主控制器，经过串口接收钢球位置和速度数据，利用位置/速度控制算法计算步进电机控制量，通过 TMC2209 驱动开环步进电机，最终由曲柄连杆机构调整平衡杆角度，使钢球保持在目标位置。

```text
MaixCam Pro
  └─ YOLO26 识别钢球
      └─ 位置滤波、坐标标定、速度估计
          └─ UART 串口发送 $C 帧
              └─ STM32F407VET6 UART5 接收
                  └─ STM32 低通滤波
                      └─ 平衡控制器
                          └─ TIM3 输出 STEP，GPIO 输出 DIR/EN
                              └─ TMC2209
                                  └─ 开环步进电机
                                      └─ 曲柄连杆机构
                                          └─ 平衡杆
```

## 2. 当前实际使用的源码

### MaixCam Pro

| 文件 | 功能 |
|---|---|
| `maixcampro/H_demo/main.py` | YOLO26 检测、钢球跟踪、串口发送 |
| `maixcampro/H_demo/ball_position.py` | 位置滤波与几何坐标换算 |
| `maixcampro/H_demo/yolo26_all_maixcam.cvimodel` | YOLO26 模型 |
| `maixcampro/H_demo/yolo26_all_maixcam.mud` | MaixCam 模型描述文件 |

### STM32F407VET6

| 文件 | 功能 |
|---|---|
| `Core/Src/main.c` | 外设初始化、任务调度、平衡控制主流程 |
| `Src/Uart_maixcam.c` | MaixCam UART5 接收和协议解析 |
| `Src/Uart_maixcam.h` | 视觉串口接口定义 |
| `Src/balance_control.c` | 位置环、速度环和预测控制 |
| `Src/balance_control.h` | 平衡控制参数和数据结构 |
| `Src/stepper.c` | STEP/DIR 步进电机控制 |
| `Src/Stepper.h` | 步进电机接口定义 |
| `Src/Tmc2209.c` | TMC2209 UART 寄存器读写备用代码 |
| `MDK-ARM/PT_head_TMC2209.uvprojx` | Keil MDK 工程 |

## 3. 硬件连接定义

### 3.1 MaixCam Pro 与 STM32F407VET6

MaixCam 使用 UART0，STM32 使用 UART5。串口必须交叉连接，且两端共地。

| MaixCam Pro | STM32F407VET6 | 说明 |
|---|---|---|
| A16 / UART0_TX | PD2 / UART5_RX | MaixCam 发送，STM32 接收 |
| A17 / UART0_RX | PC12 / UART5_TX | STM32 回复，MaixCam 接收 |
| GND | GND | 必须共地 |

串口参数：

```text
波特率：115200
数据位：8
校验位：None
停止位：1
硬件流控：None
```

> 注意：源码备注指出 A16/A17 的 UART0 可能被 MaixPy 内部通信协议占用。若出现 MaixCam 能运行但 STM32 收不到连续数据，应优先检查 UART0 的内部占用和引脚复用状态。

### 3.2 TMC2209 与 STM32

当前实际运行方案采用 TMC2209 的 STEP/DIR/EN 控制方式。

| STM32 引脚 | 信号 | TMC2209 引脚/功能 | 说明 |
|---|---|---|---|
| PA6 / TIM3_CH1 | STEP | STEP | TIM3 输出步进脉冲 |
| PC4 | DIR | DIR | 电机方向 |
| PA7 | EN | EN | 低电平有效 |
| GND | GND | GND | 必须共地 |
| 电机电源 | VM | VM | 按电机和驱动器规格供电 |
| 电机两相 | A1/A2、B1/B2 | 电机绕组 | 注意相序 |

当前使用的是源码中称为 yaw 通道的电机接口，而不是 pitch 通道：

```c
Stepper_Init(&balance_motor, &htim3, TIM_CHANNEL_1,
             Yaw_DIR_GPIO_Port, Yaw_DIR_Pin, MODE_POSITION);
```

对应定义：

```c
Yaw_EN  = PA7
Yaw_DIR = PC4
STEP    = PA6 / TIM3_CH1
```

EN 电平逻辑：

```text
EN = 1：关闭 TMC2209 输出，电机不保持
EN = 0：使能 TMC2209
```

系统启动时先将 EN 置为高电平，完成 TIM3 和步进模块初始化后再根据运行模式使能，避免上电瞬间产生误步。

### 3.3 STM32 其他已定义接口

| 外设/功能 | STM32 引脚 | 用途 |
|---|---|---|
| USART1 | PA9 / PA10 | 调试或 TMC2209 UART 备用接口 |
| USART2 | PA2 / PA3 | 调试/调参串口 |
| I2C1 OLED | PC0 / PE12 | OLED SDA/SCL，具体复用以 CubeMX 配置为准 |
| TIM1 备用 STEP | PE9 等 | pitch 通道备用，当前主控制未使用 |
| Pitch EN | PE6 | pitch 驱动使能，当前主控制未使用 |
| Pitch DIR | PE11 | pitch 驱动方向，当前主控制未使用 |
| 蜂鸣器 | PE5 | Buzzer |
| 激光器 | PB10 | Laser |
| 按键 KEY1 | PB12 | 模式/功能按键 |
| 按键 KEY2 | PB13 | 模式/功能按键 |
| 按键 KEY3 | PB14 | 模式/功能按键 |
| 按键 KEY4 | PB15 | 模式/功能按键 |

## 4. MaixCam → STM32 通信协议

当前主控制使用 `$C` 协议。

### 4.1 正常钢球数据帧

```text
$C,<valid>,<position_cm_x100>,<velocity_cms_x100>\n
```

字段定义：

| 字段 | 含义 | 类型 |
|---|---|---|
| `$C` | 帧类型 | 固定字符串 |
| `valid` | 钢球数据是否有效 | `0` 或 `1` |
| `position_cm_x100` | 钢球位置，实际厘米值乘以 100 | 有符号整数 |
| `velocity_cms_x100` | 钢球速度，实际 cm/s 值乘以 100 | 有符号整数 |
| `\n` | 帧结束符 | LF |

示例：

```text
$C,1,125,-36\n
```

表示：

```text
valid = 1
position = 125 / 100 = 1.25 cm
velocity = -36 / 100 = -0.36 cm/s
```

### 4.2 丢球数据帧

```text
$C,0,0,0\n
```

当短时预测窗口结束后仍未重新检测到钢球，MaixCam 发送该帧。STM32 收到后将视觉数据标记为无效，并进入丢球保护逻辑。

### 4.3 STM32 应答帧

收到并解析 `$C` 帧后，STM32 可通过 UART5 返回：

```text
$D,<valid>,<position_cm_x100>,<velocity_cms_x100>\n
```

例如：

```text
$D,1,125,-36\n
```

该应答主要用于串口调试和通信验证，不参与主控制计算。

### 4.4 旧 `$B` 协议

源码仍保留旧的像素协议：

```text
$B,<valid>,<pixel_x>\n
```

例如：

```text
$B,1,160\n
```

该协议会在 STM32 内部通过标定参数换算成厘米，但当前 MaixCam H_demo 主程序使用的是 `$C`，因此 `$B` 不属于当前主控制通信方案。

## 5. 当前实际调用链

STM32 的实际调用流程如下：

```text
main()
  ├─ MX_UART5_Init()
  ├─ VisionUart_Init()
  │    └─ HAL_UART_Receive_IT(&huart5, &rx_byte, 1)
  └─ while(1)
       ├─ VisionUart_Process()
       │    ├─ 解析 $C 帧
       │    ├─ 更新厘米位置和速度
       │    └─ VisionUart_UpdateLowPass()
       └─ Tuning_Process()

TIM4 周期控制任务
  └─ VisionUart_ConsumeFilteredPosition()
       ├─ 获得 filtered_pos_cm
       ├─ 获得 vision_vel_cms
       ├─ 检查 vision_valid
       └─ 进入 BalanceCascade 控制器
```

当前用于平衡控制的接口是：

```c
VisionUart_ConsumeFilteredPosition(
    &vision_valid,
    &filtered_pos_cm,
    &vision_vel_cms
);
```

当前没有将以下接口用于主控制：

```c
VisionUart_ConsumeCmFrame();
VisionUart_ConsumeFrame();
```

## 6. 视觉数据处理流程

MaixCam 端：

```text
YOLO26检测
  → ROI过滤
  → 选择最高置信度钢球
  → 位置时序滤波
  → 像素坐标映射为厘米
  → 计算钢球速度
  → 位置和速度乘以100转为整数
  → 发送$C帧
```

STM32 端：

```text
接收$C帧
  → 解析定点整数
  → 还原为float厘米值
  → 一阶低通滤波
  → VisionUart_ConsumeFilteredPosition()
  → 平衡控制
```

当前 `$C` 帧中的位置和速度已经在 MaixCam 端计算过一次，STM32 又会进行一次低通滤波。这样可以降低 YOLO 检测抖动，但也会增加一定延迟，调参时需要同时考虑两级滤波。

## 7. 平衡控制与步进控制

当前控制器采用位置/速度信息计算平台角度，再转换为步进电机目标运动。

控制过程概念如下：

```text
钢球目标位置 - 实际位置
  → 外环控制器
      → 目标平衡杆角度
          → 曲柄连杆几何换算
              → 电机目标步数/速度
                  → STEP/DIR输出
```

TIM3 的 STEP 输出采用脉冲频率表示电机速度，源码注释中说明 STEP 保持约 50% 占空比，通过修改频率控制步进速度。

当前系统为开环步进控制：

- 没有电机编码器反馈；
- 没有直接的平衡杆角度传感器闭环；
- 软件通过机械水平位置建立零点；
- 上电时机械机构必须处于已知、水平的位置；
- 电机方向和曲柄正负方向必须现场验证。

## 8. TMC2209 配置说明

虽然工程中包含 `Src/Tmc2209.c`，但当前 `main.c` 明确跳过 TMC2209 UART 寄存器初始化，实际方案依赖：

- STEP/DIR/EN 硬件接口；
- TMC2209 的 MS1/MS2 硬件绑定位；
- 驱动器电流和细分的硬件配置；
- 驱动器模块本身的默认状态。

因此，当前方案不能把 `Tmc2209.c` 的存在理解为“运行时已经通过 UART 配置 TMC2209”。如果未来需要使用 TMC2209 UART 配置，应重新确认：

1. STM32 USART1 与 TMC2209 PDN_UART 的物理连接；
2. 单线半双工方向切换；
3. TMC2209 地址设置；
4. UART CRC；
5. 写寄存器后的读回校验；
6. 与 STEP/DIR 模式的兼容关系。

## 9. 启动流程

STM32 上电后的关键顺序：

```text
HAL_Init()
  → 系统时钟初始化
  → GPIO初始化
  → EN置为关闭
  → TIM1/TIM3/TIM4初始化
  → USART1/USART2初始化
  → I2C1初始化
  → UART5初始化
  → DWT和延时初始化
  → 步进电机初始化
  → 平衡控制器初始化
  → 机械水平零点建立
  → 进入任务4/持续平衡模式
  → VisionUart_Init()
  → 启动TIM4控制中断
```

## 10. 调试检查清单

### 串口通信

- [ ] MaixCam 和 STM32 共地；
- [ ] TX/RX 交叉连接；
- [ ] 两端均为 115200、8N1；
- [ ] MaixCam 实际发送 `$C` 而不是 `$B`；
- [ ] STM32 UART5 使用 PC12/PD2；
- [ ] UART5 中断已开启；
- [ ] 串口监视器可以看到 `$C` 和 `$D` 帧；
- [ ] 检查 UART0 是否被 MaixPy 内部通信占用。

### 电机控制

- [ ] PA6 确实输出 TIM3 STEP；
- [ ] PC4 方向逻辑正确；
- [ ] PA7 的 EN 为低有效；
- [ ] TMC2209 电源、逻辑电源和 STM32 共地；
- [ ] 电机两相接线正确；
- [ ] MS1/MS2 细分配置与控制参数一致；
- [ ] 电机电流限制合适；
- [ ] 上电机械机构处于水平零点；
- [ ] 低速单步测试确认正负方向。

### 安全保护

- [ ] 丢球时电机停止或进入安全状态；
- [ ] 视觉串口断开时不能持续输出失控脉冲；
- [ ] 步数和速度软限位有效；
- [ ] 电机启动时没有异常抖动；
- [ ] 机械结构有物理限位；
- [ ] 调试时先降低电机速度和电流。

## 11. 当前方案的已知限制

1. UART 协议没有 CRC 或 checksum，只有帧格式检查。
2. 视觉链路存在 MaixPy UART0 内部占用风险。
3. MaixCam 和 STM32 都进行了滤波，整体延迟需要现场测试。
4. 系统为开环步进控制，丢步无法被软件直接发现。
5. 机械水平零点依赖上电时的人工定位。
6. 工程目录中的旧 STM32F103 工程不属于当前 F407 主工程，不应混合烧录或引用。

## 12. 推荐的 GitHub 仓库结构

```text
.
├── README.md
├── Core/
├── Drivers/
├── Src/
├── maixcampro/
│   └── H_demo/
├── MDK-ARM/
│   └── PT_head_TMC2209.uvprojx
└── tools/
```

建议不要将以下大体积或临时文件作为源码重点提交：

- Keil 的中间编译文件：`.o`、`.d`、`.crf`、`.map`；
- 调试临时目录；
- Python `__pycache__`；
- 旧的 STM32F103 工程，除非明确作为历史参考保存。

## 13. 版本说明

本文档描述的是当前源码实际使用的方案：

```text
视觉协议：$C
STM32接收：UART5
控制读取：VisionUart_ConsumeFilteredPosition()
电机控制：TIM3 STEP + PC4 DIR + PA7 EN
驱动方式：TMC2209 STEP/DIR 开环
主控：STM32F407VET6
```

