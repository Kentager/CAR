# 智能车双环PID控制模块使用手册

## 版本信息
- **版本号**: 1.1
- **更新日期**: 2026-04-06
- **作者**: Claude Sonnet 4.6

## 1. 模块概述

### 1.1 功能描述
本模块实现了智能车的双环PID控制架构，包括：
- **速度环（内环）**: 增量式PID控制，基于编码器反馈控制电机转速
- **角度环（外环）**: 位置式PID控制，基于姿态角度控制车体方向

### 1.2 控制架构
```
┌─────────────────────────────────────────────────────────┐
│                   双环PID控制架构                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │  角度环PID  │───▶│  速度环PID  │───▶│   电机驱动  │  │
│  │  (位置式)   │    │  (增量式)   │    │   (PWM)     │  │
│  └─────────────┘    └─────────────┘    └─────────────┘  │
│       ▲                   ▲                   ▲        │
│       │                   │                   │        │
│  姿态角度反馈        编码器转速反馈        电机实际输出   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 1.3 支持的硬件模式
- **四驱模式**: 4路电机 + 4路编码器
- **双驱模式**: 2路电机 + 2路编码器

## 2. 接口说明

### 2.1 核心数据结构

#### Control_State_t - 控制状态结构体
```c
typedef struct {
  // 控制模式
  Control_Mode_e mode;          // 当前控制模式
  uint8_t enabled;              // 是否启用控制

  // 目标值
  float target_speed_m_s;       // 目标线速度（m/s）
  float target_angle_deg;       // 目标角度（度）
  float turn_angle_deg;         // 目标转弯角度（度）

  // 当前值
  float current_speed_m_s;      // 当前线速度（m/s）
  float current_angle_deg;      // 当前角度（度）

  // 角度偏差补偿值
  float angle_compensation;     // 角度环输出补偿值

  // 四驱/双驱模式速度变量
  #ifdef QUAD_MOTOR_DRIVE
    float speed_fr; // 前右电机速度
    float speed_fl; // 前左电机速度
    float speed_br; // 后右电机速度
    float speed_bl; // 后左电机速度
  #else
    float speed_right; // 右电机速度
    float speed_left;  // 左电机速度
  #endif

  // 时间戳
  uint32_t last_update_time;

  // PID控制器指针
  Speed_PID_Controller_t *speed_pid[MOTOR_COUNT];
  Angle_PID_Controller_t *angle_pid;
} Control_State_t;
```

#### Control_Mode_e - 控制模式枚举
```c
typedef enum {
  CONTROL_MODE_STOP = 0,           // 停止模式
  CONTROL_MODE_STRAIGHT = 1,       // 直线行驶模式
  CONTROL_MODE_TURN_SMALL = 2,     // 小角度转弯模式（<30°）
  CONTROL_MODE_TURN_LARGE = 3,     // 大角度转弯模式（≥30°）
  CONTROL_MODE_SPIN = 4            // 原地旋转模式
} Control_Mode_e;
```

### 2.2 初始化和控制函数

#### Control_Init()
```c
void Control_Init(void);
```
**功能**: 初始化控制系统
**调用时机**: 系统启动时调用一次
**说明**:
- 初始化速度环PID控制器
- 初始化角度环PID控制器
- 初始化姿态解算系统
- 清零控制状态

#### Control_Enable()
```c
void Control_Enable(void);
```
**功能**: 启用控制系统
**调用时机**: 准备开始运动前调用
**说明**:
- 启用所有速度环PID控制器
- 启用角度环PID控制器
- 启动电机定时器

#### Control_Disable()
```c
void Control_Disable(void);
```
**功能**: 禁用控制系统
**调用时机**: 停止运动或需要暂时禁用控制时
**说明**:
- 禁用所有PID控制器
- 不会停止电机，需配合Control_Stop()使用

### 2.3 模式设置函数

#### Control_SetMode()
```c
void Control_SetMode(Control_Mode_e mode);
```
**功能**: 设置控制模式
**参数**:
- `mode`: 控制模式（CONTROL_MODE_STRAIGHT/TURN_SMALL/TURN_LARGE/SPIN）

#### Control_SetSpeed()
```c
void Control_SetSpeed(float speed_m_s);
```
**功能**: 设置目标线速度
**参数**:
- `speed_m_s`: 目标线速度（m/s），正值前进，负值后退
**说明**: 所有电机以相同速度运行

#### Control_SetTurnAngle()
```c
void Control_SetTurnAngle(float angle_deg);
```
**功能**: 设置目标转弯角度
**参数**:
- `angle_deg`: 目标转弯角度（度），正值右转，负值左转
**说明**: 用于转弯控制模式

#### Control_SetTargetAngle()
```c
void Control_SetTargetAngle(float angle_deg);
```
**功能**: 设置目标角度
**参数**:
- `angle_deg`: 目标角度（度）
**说明**: 用于直线行驶时的角度基准，通常设为0度

### 2.4 更新函数

#### Control_Update()
```c
void Control_Update(void);
```
**功能**: 控制系统更新（使用卡尔曼滤波角度）
**调用周期**: 10ms（100Hz）
**说明**:
- 更新姿态解算（JY61P + 卡尔曼滤波）
- 执行角度环PID控制
- 执行速度环PID控制
- 应用角度补偿到速度环

#### Control_UpdateWithDMP()
```c
void Control_UpdateWithDMP(void);
```
**功能**: 控制系统更新（使用MPU6050 DMP角度）
**调用周期**: 10ms（100Hz）
**说明**:
- 从MPU6050 DMP直接获取姿态角（无需磁力计修正）
- 执行角度环PID控制
- 执行速度环PID控制
- 应用角度补偿到速度环
**推荐**: 此方法更简单，适合大多数应用场景

#### Control_UpdateSpeedLoop()
```c
void Control_UpdateSpeedLoop(void);
```
**功能**: 单独更新速度环
**调用周期**: 10ms（100Hz）
**说明**: 用于独立调试速度环或特殊场景

#### Control_UpdateAngleLoop()
```c
void Control_UpdateAngleLoop(void);
```
**功能**: 单独更新角度环
**调用周期**: 5ms（200Hz）
**说明**: 用于独立调试角度环或特殊场景

### 2.5 状态查询函数

#### Control_GetMode()
```c
Control_Mode_e Control_GetMode(void);
```
**功能**: 获取当前控制模式
**返回值**: 当前控制模式

#### Control_GetState()
```c
Control_State_t *Control_GetState(void);
```
**功能**: 获取当前控制状态
**返回值**: 控制状态结构体指针

### 2.6 停止和刹车函数

#### Control_Stop()
```c
void Control_Stop(void);
```
**功能**: 停止所有运动
**说明**:
- 停止所有电机
- 禁用所有PID控制器
- 设置停止模式
- 清零目标值

#### Control_Brake()
```c
void Control_Brake(void);
```
**功能**: 刹车所有运动
**说明**:
- 刹车所有电机（启用刹车功能）
- 禁用所有PID控制器
- 设置停止模式
- 清零目标值

## 3. 使用流程

### 3.1 基本使用流程
```c
// 1. 初始化控制系统
Control_Init();

// 2. 设置目标速度
Control_SetSpeed(0.5f);  // 0.5 m/s

// 3. 设置控制模式
Control_SetMode(CONTROL_MODE_STRAIGHT);

// 4. 启用控制系统
Control_Enable();

// 5. 在主循环中定期更新
while(1) {
  Control_UpdateWithDMP();  // 使用DMP角度更新
  delay_ms(10);  // 10ms周期
}
```

### 3.2 直线行驶流程
```c
// 设置为直线行驶模式
Control_SetMode(CONTROL_MODE_STRAIGHT);

// 设置目标速度（前进）
Control_SetSpeed(0.8f);  // 0.8 m/s

// 启用控制
Control_Enable();

// 主循环
while(1) {
  Control_UpdateWithDMP();
  delay_ms(10);
}
```

### 3.3 转弯流程
```c
// 小角度转弯（<30°）
Control_SetMode(CONTROL_MODE_TURN_SMALL);
Control_SetTurnAngle(15.0f);  // 右转15度
Control_SetSpeed(0.5f);

// 大角度转弯（≥30°）
Control_SetMode(CONTROL_MODE_TURN_LARGE);
Control_SetTurnAngle(90.0f);  // 右转90度
Control_SetSpeed(0.3f);

// 原地旋转
Control_SetMode(CONTROL_MODE_SPIN);
Control_SetTurnAngle(180.0f);  // 旋转180度
Control_SetSpeed(0.2f);
```

### 3.4 停止流程
```c
// 正常停止
Control_Stop();

// 紧急刹车
Control_Brake();
```

## 4. 参数配置

### 4.1 速度环PID参数
**位置**: `pid_speed.h`
```c
#define SPEED_PID_KP_DEFAULT 0.5f  // 比例系数
#define SPEED_PID_KI_DEFAULT 0.1f  // 积分系数
#define SPEED_PID_KD_DEFAULT 0.05f // 微分系数
```

### 4.2 角度环PID参数
**位置**: `pid_angle.h`
```c
#define ANGLE_PID_KP_DEFAULT 0.0004f  // 比例系数
#define ANGLE_PID_KI_DEFAULT 0.0f     // 积分系数
#define ANGLE_PID_KD_DEFAULT 0.0f     // 微分系数
```

### 4.3 控制阈值参数
```c
#define TURN_SMALL_ANGLE_THRESHOLD 30.0f  // 小角度/大角度转弯阈值
#define STRAIGHT_SPEED_THRESHOLD 0.1f     // 直线行驶速度阈值
#define ANGLE_THRESHOLD_DEG 1.0f          // 角度偏差阈值
```

## 5. 调试建议

### 5.1 速度环调试
1. 先禁用角度环，单独调试速度环
2. 设置目标速度，观察电机响应
3. 调整PID参数直到转速稳定

### 5.2 角度环调试
1. 先调试好速度环
2. 启用角度环，设置目标角度
3. 观察角度补偿值和电机转速差
4. 调整PID参数直到角度稳定

### 5.3 联调
1. 在直线模式下测试
2. 在转弯模式下测试
3. 在不同速度下测试
4. 记录控制状态，分析性能

## 6. 注意事项

### 6.1 时序要求
- 速度环更新周期: 10ms（100Hz）
- 角度环更新周期: 5ms（200Hz）
- 必须严格遵守时间周期，否则控制性能会下降

### 6.2 硬件连接
- 确保编码器连接正确
- 确保电机驱动电源充足
- 确保MPU6050/磁力计通信正常

### 6.3 参数调优
- PID参数需要根据实际机械特性调整
- 不同负载下参数可能不同
- 建议从小参数开始逐步调优

### 6.4 安全考虑
- 首次测试时使用低速
- 准备急停按钮
- 注意电机过流保护

## 7. 常见问题

### Q1: 电机不转？
- 检查电机驱动电源
- 检查PWM输出
- 检查控制是否启用

### Q2: 转速不稳定？
- 检查编码器连接
- 调整速度环PID参数
- 检查采样周期

### Q3: 角度控制不准确？
- 检查MPU6050 DMP初始化
- 调整角度环PID参数
- 检查角度阈值设置

### Q4: 转弯不准确？
- 检查转弯角度设置
- 调整角度环PID参数
- 检查编码器精度

## 8. 版本历史

### v1.1 (2026-04-06)
- 新增 `Control_UpdateWithDMP()` 函数，支持使用MPU6050 DMP角度
- 更新文档，说明两种角度更新方式
- 优化代码结构

### v1.0 (2026-03-13)
- 初始版本
- 实现双环PID控制架构
- 支持四驱/双驱模式切换
