# 圆弧转弯控制模块 (Arc Turn Control Module)

## 概述

圆弧转弯控制模块是智能车项目的应用层模块，用于控制小车按照设定的半径进行圆弧运动。该模块基于运动学模型，根据设定的转弯半径和目标速度，自动计算并分配左右轮的目标速度。

## 文件结构

```
APP/
├── Inc/
│   └── arc_turn.h          # 圆弧转弯模块头文件
└── Src/
    ├── arc_turn.c          # 圆弧转弯模块实现
    └── main_arc_turn.c     # 使用示例
```

## 核心功能

### 1. 运动学模型

基于阿克曼转向原理的简化模型：

- **外侧轮速度**: `V_outer = V_center × (R + W/2) / R`
- **内侧轮速度**: `V_inner = V_center × (R - W/2) / R`

其中：
- `R`: 转弯半径（米）
- `W`: 轮距（左右轮中心距离，米）
- `V_center`: 中心目标速度（米/秒）

### 2. 转弯方向

- **顺时针转弯 (CW)**: 左轮为内侧轮，右轮为外侧轮
- **逆时针转弯 (CCW)**: 右轮为内侧轮，左轮为外侧轮
- **直线行驶**: 左右轮速度相同

### 3. 主要API接口

#### 初始化控制器

```c
ArcTurn_Controller_t ArcTurn_Controller;
ArcTurn_Init(&ArcTurn_Controller, &Speed_PID_Left, &Speed_PID_Right);
```

#### 设置转弯参数

```c
// 顺时针转弯，半径1米，中心速度0.2m/s
ArcTurn_SetParameters(&ArcTurn_Controller, 1.0f, 0.2f, ARC_TURN_DIR_CW);

// 逆时针转弯，半径0.5米，中心速度0.15m/s
ArcTurn_SetParameters(&ArcTurn_Controller, 0.5f, 0.15f, ARC_TURN_DIR_CCW);

// 直线行驶
ArcTurn_SetParameters(&ArcTurn_Controller, 10.0f, 0.3f, ARC_TURN_DIR_STRAIGHT);
```

#### 启用/禁用控制

```c
// 启用圆弧转弯控制
ArcTurn_Enable(&ArcTurn_Controller);

// 禁用圆弧转弯控制（会停止电机）
ArcTurn_Disable(&ArcTurn_Controller);
```

#### 更新控制

```c
// 在控制循环中调用（推荐10ms周期）
ArcTurn_Update(&ArcTurn_Controller);
```

#### 获取当前轮速

```c
float left_speed, right_speed;
ArcTurn_GetWheelSpeeds(&ArcTurn_Controller, &left_speed, &right_speed);
printf("Left: %.3f m/s, Right: %.3f m/s\r\n", left_speed, right_speed);
```

## 配置参数

在 [arc_turn.h](APP/Inc/arc_turn.h) 中可以调整以下参数：

```c
// 机械参数（根据实际硬件测量调整）
#define WHEEL_BASE 0.15f        // 轮距（左右轮中心距离，单位：米）
#define WHEEL_RADIUS 0.0325f    // 轮子半径（单位：米）
#define MIN_TURN_RADIUS 0.2f    // 最小转弯半径（单位：米）
#define MAX_TURN_RADIUS 10.0f   // 最大转弯半径（单位：米）

// 控制参数
#define ARC_TURN_UPDATE_PERIOD_MS 10  // 圆弧转弯更新周期（毫秒）
```

## 使用示例

### 基础使用

```c
#include "arc_turn.h"

// 全局变量
ArcTurn_Controller_t ArcTurn_Controller;

int main(void) {
    // 初始化硬件
    Motor_Driver_Init();
    Encoder_Driver_Init();
    
    // 初始化PID控制器
    Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                   SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                   SPEED_PID_KD_DEFAULT);
    Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                   SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                   SPEED_PID_KD_DEFAULT);
    
    // 初始化圆弧转弯控制器
    ArcTurn_Init(&ArcTurn_Controller, &Speed_PID_Left, &Speed_PID_Right);
    
    // 设置转弯参数
    ArcTurn_SetParameters(&ArcTurn_Controller, 1.0f, 0.2f, ARC_TURN_DIR_CW);
    
    // 启用控制
    Speed_PID_Enable(&Speed_PID_Right);
    Speed_PID_Enable(&Speed_PID_Left);
    ArcTurn_Enable(&ArcTurn_Controller);
    
    // 在主循环中更新
    while (1) {
        Encoder_Update();
        Speed_PID_Update(&Speed_PID_Right);
        Speed_PID_Update(&Speed_PID_Left);
        ArcTurn_Update(&ArcTurn_Controller);
        delay_ms(10);
    }
}
```

### 动态调整转弯半径

```c
void dynamic_turn_example(void) {
    float radius = 1.0f;  // 初始半径1米
    float speed = 0.2f;   // 固定速度0.2m/s
    
    while (1) {
        // 设置新的转弯半径
        ArcTurn_SetParameters(&ArcTurn_Controller, radius, speed, ARC_TURN_DIR_CW);
        
        // 增加半径（实现向外螺旋）
        radius += 0.1f;
        
        // 如果半径超过最大值，重置为最小值
        if (radius > MAX_TURN_RADIUS) {
            radius = MIN_TURN_RADIUS;
        }
        
        delay_ms(1000);
    }
}
```

### 结合循迹传感器

```c
void smart_turn_example(void) {
    // 从循迹传感器获取偏差值（-1.0到1.0）
    float line_deviation = get_line_deviation();
    
    // 根据偏差计算转弯半径
    float radius;
    ArcTurn_Direction_e dir;
    
    if (fabsf(line_deviation) < 0.1f) {
        // 偏差很小，直线行驶
        radius = MAX_TURN_RADIUS;
        dir = ARC_TURN_DIR_STRAIGHT;
    } else if (line_deviation > 0) {
        // 偏右，需要左转
        radius = 0.5f / line_deviation;
        dir = ARC_TURN_DIR_CCW;
    } else {
        // 偏左，需要右转
        radius = 0.5f / fabsf(line_deviation);
        dir = ARC_TURN_DIR_CW;
    }
    
    // 限制半径范围
    if (radius < MIN_TURN_RADIUS) {
        radius = MIN_TURN_RADIUS;
    }
    
    // 设置圆弧转弯参数
    ArcTurn_SetParameters(&ArcTurn_Controller, radius, 0.3f, dir);
}
```

## 调试与优化

### 1. 调试信息输出

在 [main_arc_turn.c](APP/Src/main_arc_turn.c) 中提供了调试信息打印任务：

```c
void debug_print_task(void) {
    Encoder_Data_t left_encoder, right_encoder;
    left_encoder = Encoder_GetData(ENCODER_LEFT);
    right_encoder = Encoder_GetData(ENCODER_RIGHT);
    
    float target_left_speed, target_right_speed;
    ArcTurn_GetWheelSpeeds(&ArcTurn_Controller, &target_left_speed, &target_right_speed);
    
    printf("Target: L=%.3f, R=%.3f | Actual: L=%.3f, R=%.3f | Radius: %.2f m\r\n",
           target_left_speed, target_right_speed,
           left_encoder.speed_m_s, right_encoder.speed_m_s,
           ArcTurn_Controller.turn_radius);
}
```

### 2. 参数调优建议

1. **轮距校准**: 使用实际测量的轮距值，影响转弯精度
2. **最小转弯半径**: 根据机械结构限制调整
3. **PID参数**: 转弯时需要重新调优PID参数
4. **更新频率**: 建议与速度环PID控制频率一致（10ms）

### 3. 常见问题

**问题**: 转弯时小车走偏
- **解决**: 检查轮距参数是否准确，调整PID参数

**问题**: 转弯半径不准确
- **解决**: 校准轮距参数，检查编码器精度

**问题**: 内侧轮速度为负
- **解决**: 增大转弯半径或减小中心速度

## 集成到现有系统

将圆弧转弯模块集成到现有的 [main_debug.c](USER/Src/main_debug.c) 中：

1. 在头文件中包含 `arc_turn.h`
2. 声明全局控制器实例
3. 在初始化代码中调用 `ArcTurn_Init()`
4. 添加圆弧转弯控制任务
5. 在合适的位置调用 `ArcTurn_SetParameters()` 设置转弯参数

## 技术参数

- **支持模式**: 四驱/双驱自动适配
- **更新频率**: 推荐10ms（可调整）
- **转弯半径**: 0.2m - 10.0m
- **目标速度**: 0.0m/s - 2.0m/s
- **方向控制**: 顺时针/逆时针/直线

## 注意事项

1. **安全考虑**: 首次使用时建议低速测试，逐渐增加速度
2. **参数校准**: 轮距参数对转弯精度影响很大，需要准确测量
3. **PID适配**: 转弯工况下PID参数可能需要重新调优
4. **机械限制**: 最小转弯半径受机械结构限制
5. **电源供应**: 确保电源能够满足双轮差速驱动的功率需求

## 扩展功能

可以根据需要扩展以下功能：

1. **加速度控制**: 添加加速度限制，实现平滑加减速
2. **路径规划**: 支持多段圆弧路径规划
3. **位置反馈**: 结合编码器反馈实现闭环位置控制
4. **姿态反馈**: 结合JY61P姿态传感器实现更精确的转弯控制
5. **自适应转弯**: 根据速度动态调整转弯半径