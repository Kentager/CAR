# 基于 STM32F4 的智能车速度环控制系统

## 项目概述

本项目是一个基于 STM32F407 微控制器的智能车速度环控制系统，采用双闭环 PID 控制架构中的速度环实现，通过编码器反馈进行精确的速度控制。系统支持双驱和四驱两种模式，可灵活适配不同硬件配置。

## 核心功能

### 1. 电机驱动模块（Motor Driver）
- **PWM 输出**：使用 TIM1 和 TIM8 定时器产生 20kHz PWM 信号
- **H 桥控制**：支持 L298N/TB6612 驱动芯片，独立控制每路电机正反转
- **多模式支持**：
  - 双驱模式：2 路电机（TIM1_CH1/CH2）
  - 四驱模式：4 路电机（TIM1_CH1/CH2 + TIM8_CH1/CH2）
- **分辨率**：1000 级 PWM 占空比控制（0-8399）
- **方向控制**：GPIO 独立控制正转、反转、停止、刹车四种状态

### 2. 编码器测速模块（Encoder Driver）
- **正交解码**：使用 TIM2/TIM3/TIM4/TIM5 硬件正交解码模式
- **4 倍频技术**：上升沿和下降沿同时计数，提高测量精度
- **编码器规格**：
  - 线数：500 线
  - 减速比：1:30
  - 每转脉冲数（PPR）：500×4×30 = 60,000
- **测量功能**：
  - 实时转速（RPM/RPS/rad/s）
  - 线速度（m/s）
  - 累计里程（m）
  - 16 位计数器溢出自动处理
- **采样周期**：1ms 高速采样，10ms 控制循环更新

### 3. 速度环 PID 控制模块（Speed PID Controller）
- **控制算法**：增量式 PID 控制算法
- **默认参数**：
  - 比例系数 Kp = 2.0
  - 积分系数 Ki = 0.5
  - 微分系数 Kd = 0.1
- **控制周期**：10ms（符合香农采样定理）
- **输入输出**：
  - 输入：目标线速度（m/s）
  - 输出：PWM 占空比（0-8399）
- **特性**：
  - 独立控制器实例（左/右轮分离控制）
  - 使能/禁用控制
  - 抗积分饱和处理

## 系统架构

```
┌─────────────────────────────────────────────┐
│           应用层（Application）              │
│  - 速度设定                                   │
│  - 任务调度                                   │
│  - 用户接口                                   │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│           控制层（Control Layer）            │
│  ┌─────────────────────────────────────┐   │
│  │   速度环 PID 控制器                    │   │
│  │   - Speed_PID_Right                  │   │
│  │   - Speed_PID_Left                   │   │
│  └─────────────────────────────────────┘   │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│           驱动层（Driver Layer）             │
│  ┌──────────────┐  ┌──────────────┐        │
│  │  Motor Driver│  │Encoder Driver│        │
│  │  - PWM 生成   │  │  - 正交解码   │        │
│  │  - 方向控制   │  │  - 速度计算   │        │
│  │  - 状态更新   │  │  - 里程累计   │        │
│  └──────────────┘  └──────────────┘        │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│         硬件层（Hardware Layer）             │
│  - STM32F407 微控制器                        │
│  - TIM1/TIM8 (PWM 输出)                      │
│  - TIM2/TIM3/TIM4/TIM5 (编码器输入)          │
│  - GPIO (方向控制)                           │
│  - 电机 + 编码器 + 驱动器                     │
└─────────────────────────────────────────────┘
```

## 硬件配置

### 微控制器
- **型号**：STM32F407xx (Cortex-M4 @ 168MHz)
- **主频**：168MHz
- **定时器资源**：
  - TIM1：电机 PWM 输出（前轮）
  - TIM8：电机 PWM 输出（后轮，四驱模式）
  - TIM2-TIM5：编码器正交解码

### 电机配置
- **类型**：直流减速电机
- **驱动器**：L298N 或 TB6612 H 桥驱动芯片
- **数量**：
  - 双驱模式：2 路
  - 四驱模式：4 路

### 编码器配置
- **类型**：11 线光电编码器
- **安装**：每轮独立编码器
- **输出**：A/B 相正交脉冲信号

### 切换驱动模式
在 `MyHardWare/Inc/HardwareConfig.h` 中配置：
```c
// 四驱模式
#define QUAD_MOTOR_DRIVE

// 双驱模式（注释掉 QUAD_MOTOR_DRIVE）
// #define QUAD_MOTOR_DRIVE
```

## 软件设计

### 模块化设计

系统采用分层模块化设计，各模块职责清晰，接口简洁：

- **电机模块**：负责 PWM 信号生成和电机方向控制，提供速度设置接口
- **编码器模块**：负责正交解码和速度测量，提供实时速度和里程数据
- **速度环 PID 模块**：基于编码器反馈实现闭环速度控制，输出 PWM 控制量

#### 1. 电机模块（motor.h/c）

**模块功能**：
- 产生 20kHz PWM 信号驱动直流电机
- 控制电机正转、反转、停止、刹车四种状态
- 支持双驱/四驱模式自动切换

**主要接口**：
```c
// 初始化所有电机驱动
void Motor_Driver_Init(void);

// 设置电机速度（-8399 ~ +8399）
void Motor_SetSpeed(Motor_Id_e motor, int16_t speed);

// 设置电机方向
void Motor_SetDirection(Motor_Id_e motor, Motor_Direction_e dir);

// 更新电机硬件状态（将软件配置应用到 PWM 寄存器）
void Motor_Update(Motor_Id_e motor);

// 停止/刹车所有电机
void Motor_StopAll(void);
void Motor_BrakeAll(void);
```

**使用示例**：
```c
// 初始化
Motor_Driver_Init();

// 设置右电机正转，速度 4000
Motor_SetSpeed(MOTOR_RIGHT, 4000);
Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);
Motor_Update(MOTOR_RIGHT);

// 设置左电机反转，速度 3000
Motor_SetSpeed(MOTOR_LEFT, -3000);
Motor_SetDirection(MOTOR_LEFT, MOTOR_DIR_BACKWARD);
Motor_Update(MOTOR_LEFT);
```

#### 2. 编码器模块（encoder.h/c）

**模块功能**：
- 通过 TIM2/TIM3/TIM4/TIM5 硬件正交解码采集编码器脉冲
- 实时计算转速（RPM/RPS/rad/s）和线速度（m/s）
- 累计总里程，自动处理 16 位计数器溢出
- 每 1ms 自动采样更新数据

**主要接口**：
```c
// 初始化所有编码器
void Encoder_Driver_Init(void);

// 获取指定编码器实时数据
Encoder_Data_t Encoder_GetData(Encoder_Id_e encoder_id);

// 数据结构包含：
// - count: 当前计数值
// - delta_count: 计数增量
// - speed_rpm/speed_rps/speed_rad_s/speed_m_s: 各种速度
// - total_count/total_distance: 累计脉冲数/里程
// - direction: 旋转方向
```

**使用示例**：
```c
// 初始化
Encoder_Driver_Init();

// 获取右编码器数据
Encoder_Data_t data = Encoder_GetData(ENCODER_RIGHT);

// 读取线速度用于 PID 控制
float current_speed = data.speed_m_s;

// 读取转速用于显示
printf("转速：%.1f RPM\r\n", data.speed_rpm);

// 读取累计里程
printf("里程：%.2f m\r\n", data.total_distance);
```

#### 3. 速度环 PID 模块（pid_speed.h/c）

**模块功能**：
- 基于编码器反馈实现闭环速度控制
- 采用增量式 PID 算法，抗积分饱和
- 独立控制器实例（左右轮分别控制）
- 使能/禁用管理，支持动态启停

**主要接口**：
```c
// 初始化 PID 控制器
void Speed_PID_Init(Speed_PID_Controller_t *controller, 
                    Encoder_Id_e encoder_id, 
                    Motor_Id_e motor_id,
                    float kp, float ki, float kd);

// 设置目标线速度（m/s）
void Speed_PID_SetTargetSpeed(Speed_PID_Controller_t *controller, 
                              float speed_m_s);

// 启用/禁用控制器
void Speed_PID_Enable(Speed_PID_Controller_t *controller);
void Speed_PID_Disable(Speed_PID_Controller_t *controller);

// 执行 PID 计算并更新电机输出（10ms 周期调用）
void Speed_PID_Update(Speed_PID_Controller_t *controller);
```

**使用示例**：
```c
// 初始化 PID 控制器（使用默认参数 Kp=2.0, Ki=0.5, Kd=0.1）
Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
               SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT, 
               SPEED_PID_KD_DEFAULT);

// 设置目标速度为 0.5 m/s
Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.5f);

// 启用控制器
Speed_PID_Enable(&Speed_PID_Right);

// 在主循环中周期性调用（10ms）
while(1) {
    Speed_PID_Update(&Speed_PID_Right);  // 自动完成 PID 计算和电机控制
    tdelay_ms(10);
}
```

### 控制流程

```
1. 系统初始化
   ├─ 电机驱动初始化
   ├─ 编码器驱动初始化
   └─ PID 控制器初始化

2. 10ms 控制循环
   ├─ 读取编码器反馈速度
   ├─ PID 计算（目标速度 vs 实际速度）
   ├─ 输出 PWM 占空比
   └─ 更新电机驱动状态

3. 实时任务调度
   └─ Task_Handler() 周期性调用
```

### 示例代码

#### 基本速度控制
```c
#include "motor.h"
#include "encoder.h"
#include "pid_speed.h"

int main(void) {
    // 1. 硬件初始化
    Motor_Driver_Init();
    Encoder_Driver_Init();
    
    // 2. PID 控制器初始化
    Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                   SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT, 
                   SPEED_PID_KD_DEFAULT);
    
    // 3. 设置目标速度
    Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.5f); // 0.5 m/s
    Speed_PID_Enable(&Speed_PID_Right);
    
    // 4. 主循环（10ms 周期）
    while(1) {
        Speed_PID_Update(&Speed_PID_Right);
        tdelay_ms(10);
    }
}
```

#### 任务调度方式
```c
void speed_control_task(void) {
    Speed_PID_Update(&Speed_PID_Right);
    Speed_PID_Update(&Speed_PID_Left);
}

int main(void) {
    // ... 初始化代码 ...
    
    // 添加 10ms 周期任务
    add_task(speed_control_task, 10);
    
    while(1) {
        Task_Handler(); // 任务调度器
    }
}
```

## 调试指南

### 1. PID 参数整定
- **从低速开始**：先设置 0.2-0.3 m/s 进行测试
- **调整顺序**：P → I → D
  - 增大 Kp：提高响应速度，但可能超调
  - 增大 Ki：消除稳态误差，但可能振荡
  - 增大 Kd：抑制振荡，提高稳定性
- **观察指标**：
  - 响应时间
  - 超调量
  - 稳态误差

### 2. 编码器验证
```c
Encoder_Data_t data = Encoder_GetData(ENCODER_RIGHT);
printf("速度：%.2f m/s, 里程：%.2f m\r\n", 
       data.speed_m_s, data.total_distance);
```

### 3. 电机测试
```c
// 手动测试电机
Motor_SetSpeed(MOTOR_RIGHT, 4000);
Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);
Motor_Update(MOTOR_RIGHT);
```

## 性能指标

- **PWM 频率**：20kHz
- **控制周期**：10ms（100Hz）
- **测速精度**：基于 60,000 PPR 高分辨率
- **速度范围**：0-2.0 m/s（取决于轮径和电机）
- **支持模式**：双驱/四驱可切换

## 目录结构

```
CAR/
├── CMSIS/                  # ARM Cortex-M 内核支持文件
├── Lib/StdPeriphs/        # STM32F4 标准外设库
├── MyHardWare/            # 自研硬件驱动模块
│   ├── Inc/               # 头文件
│   │   ├── motor.h        # 电机驱动
│   │   ├── encoder.h      # 编码器驱动
│   │   ├── pid_speed.h    # 速度环 PID
│   │   ├── task.h         # 任务调度
│   │   └── ...
│   └── Src/               # 源文件
├── USER/                  # 用户应用层
│   ├── Src/
│   │   └── main.c         # 主程序入口
│   └── Inc/
├── Makefile               # 编译脚本
└── README.md              # 项目说明文档
```

## 编译与烧录

### 编译环境
- **IDE**：Keil MDK / IAR EWARM
- **编译器**：ARMCC / GCC
- **调试器**：J-Link / ST-Link

### 编译命令
```bash
make clean
make all
```

### 烧录步骤
1. 连接 ST-Link/J-Link 调试器
2. 运行烧录工具
3. 下载程序到 Flash
4. 复位运行

## 关键技术点

### 1. 正交编码器解码
- 使用 STM32 定时器硬件正交解码模式
- TI1FP1 和 TI2FP2 连接到 CH1 和 CH2
- 上升沿和下降沿同时计数（4 倍频）
- 自动方向检测（根据计数增减）

### 2. 16 位计数器溢出处理
```c
// 计算差值时考虑溢出
int32_t delta = (int32_t)current_count - (int32_t)last_count;
if (delta > 32768) delta -= 65536;
else if (delta < -32768) delta += 65536;
```

### 3. 速度计算
```c
// 基于脉冲增量计算线速度
speed_m_s = delta_count × PULSE_TO_RAD × wheel_diameter / 2 / dt
```

### 4. 实时控制循环
- 使用 SysTick 提供 10ms 基准
- 任务调度器周期性执行控制任务
- 符合香农采样定理和控制理论要求

## 应用场景

- 智能小车速度控制
- AGV（自动导引车）运动控制
- 机器人底盘驱动
- 平衡车速度环
- 其他需要精确速度控制的移动平台

## 后续扩展

### 已预留接口
- **方向环 PID**：可扩展双闭环控制（速度环 + 方向环）
- **路径跟踪**：预留传感器接口（红外、摄像头）
- **姿态解算**：支持 MPU6050 等 IMU 传感器
- **通信接口**：UART 调试输出，可扩展蓝牙/WiFi

### 推荐改进方向
1. 添加电流环形成三环控制
2. 集成路径规划算法
3. 增加无线遥控功能
4. 实现自主导航

## 注意事项

1. **电源管理**：确保电机驱动电源与控制电源隔离
2. **接地处理**：良好接地以减少电磁干扰
3. **编码器接线**：A/B 相不要接反，否则方向相反
4. **PID 参数**：根据实际机械结构调整，不可直接套用
5. **散热设计**：大电流工作时注意驱动芯片散热

## 开发团队

- **硬件开发**：基于 STM32F407 最小系统板
- **软件开发**：采用模块化、面向对象设计思想
- **技术支持**：参考 STM32 官方库文档和数据手册

## 许可证

本项目代码仅供学习和研究使用。

## 版本历史

- **v1.0** (2026-03)
  - 完成基础电机驱动
  - 实现编码器测速功能
  - 实现速度环 PID 控制
  - 支持双驱/四驱模式切换
