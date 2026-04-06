# 简洁控制模块使用手册

## 版本信息
- **版本**: 2.0
- **更新日期**: 2026-04-06

## 1. 快速开始

### 三步上手

```c
// 1. 初始化
Control_Init();

// 2. 设置参数并启用
Control_SetSpeed(0.5f);              // 设置速度 0.5 m/s
Control_SetAngle(0.0f);              // 设置目标角度 0度
Control_SetMode(CONTROL_MODE_STRAIGHT); // 设置模式
Control_Enable();                    // 启用控制

// 3. 周期更新 (10ms)
while(1) {
  Control_Update();
  delay_ms(10);
}
```

## 2. API说明

### 初始化和控制

| 函数 | 说明 |
|------|------|
| `Control_Init()` | 初始化控制模块（系统启动时调用一次） |
| `Control_Enable()` | 启用控制 |
| `Control_Disable()` | 禁用控制 |
| `Control_Stop()` | 停止运动 |

### 参数设置

| 函数 | 参数 | 说明 |
|------|------|------|
| `Control_SetSpeed(float speed)` | speed: 速度(m/s) | 正值前进，负值后退 |
| `Control_SetAngle(float angle)` | angle: 角度(度) | 正值右转，负值左转 |
| `Control_SetMode(Control_Mode_e mode)` | mode: 模式 | 见下方模式说明 |

### 更新和查询

| 函数 | 说明 |
|------|------|
| `Control_Update()` | 控制更新（10ms周期调用） |
| `Control_GetState()` | 获取控制状态 |

## 3. 控制模式

```c
typedef enum {
  CONTROL_MODE_STOP = 0,     // 停止
  CONTROL_MODE_STRAIGHT = 1, // 直线行驶
  CONTROL_MODE_TURN = 2      // 转弯
} Control_Mode_e;
```

## 4. 使用示例

### 示例1: 直线前进
```c
Control_SetSpeed(0.5f);              // 0.5 m/s 前进
Control_SetAngle(0.0f);              // 目标角度0度
Control_SetMode(CONTROL_MODE_STRAIGHT);
Control_Enable();

// 在10ms定时器中调用
void timer_10ms_isr(void) {
  Control_Update();
}
```

### 示例2: 右转90度
```c
Control_SetSpeed(0.2f);              // 低速转弯
Control_SetAngle(90.0f);             // 右转90度
Control_SetMode(CONTROL_MODE_TURN);
Control_Enable();
```

### 示例3: 左转45度
```c
Control_SetSpeed(0.2f);
Control_SetAngle(-45.0f);            // 负值左转
Control_SetMode(CONTROL_MODE_TURN);
Control_Enable();
```

### 示例4: 直线后退
```c
Control_SetSpeed(-0.3f);             // 负值后退
Control_SetAngle(0.0f);
Control_SetMode(CONTROL_MODE_STRAIGHT);
Control_Enable();
```

### 示例5: 停止
```c
Control_Stop();                      // 停止所有运动
```

### 示例6: 状态监控
```c
Control_State_t *state = Control_GetState();

printf("模式: %d\n", state->mode);
printf("速度: %.2f m/s\n", state->target_speed);
printf("角度: %.2f°\n", state->current_angle);
printf("补偿: %.3f\n", state->angle_compensation);
```

## 5. 典型应用流程

### 循迹小车
```c
// 初始化
Control_Init();
Control_Enable();

// 主循环
while(1) {
  // 读取传感器偏差
  float deviation = get_sensor_deviation();

  // 根据偏差设置角度
  float target_angle = deviation * 10.0f;
  Control_SetAngle(target_angle);
  Control_SetSpeed(0.5f);
  Control_SetMode(CONTROL_MODE_STRAIGHT);

  // 更新控制
  Control_Update();
  delay_ms(10);
}
```

### 转弯导航
```c
// 直线行驶
Control_SetSpeed(0.5f);
Control_SetAngle(0.0f);
Control_SetMode(CONTROL_MODE_STRAIGHT);
delay_ms(2000);

// 右转90度
Control_SetSpeed(0.2f);
Control_SetAngle(90.0f);
Control_SetMode(CONTROL_MODE_TURN);
delay_ms(2000);

// 继续直线
Control_SetSpeed(0.5f);
Control_SetAngle(0.0f);
Control_SetMode(CONTROL_MODE_STRAIGHT);
```

## 6. 参数调优

### 速度环PID参数
**位置**: `pid_speed.h`
```c
#define SPEED_PID_KP_DEFAULT 0.5f   // 比例系数
#define SPEED_PID_KI_DEFAULT 0.1f   // 积分系数
#define SPEED_PID_KD_DEFAULT 0.05f  // 微分系数
```

### 角度环PID参数
**位置**: `pid_angle.h`
```c
#define ANGLE_PID_KP_DEFAULT 0.0004f  // 比例系数
#define ANGLE_PID_KI_DEFAULT 0.0f     // 积分系数
#define ANGLE_PID_KD_DEFAULT 0.0f     // 微分系数
```

## 7. 注意事项

1. **时序要求**: 必须每10ms调用一次 `Control_Update()`
2. **初始化顺序**: 必须先调用 `Control_Init()`，再设置参数
3. **启用控制**: 设置参数后必须调用 `Control_Enable()`
4. **MPU6050**: 确保MPU6050 DMP已正确初始化
5. **安全**: 首次测试使用低速，准备急停

## 8. 完整示例代码

参考 `USER/Src/main_control_simple.c`

## 9. 与旧版本的区别

| 特性 | 旧版本(v1.1) | 新版本(v2.0) |
|------|-------------|-------------|
| API复杂度 | 复杂，多个函数 | 简洁，核心函数少 |
| 角度源 | 支持卡尔曼/DMP | 仅支持DMP |
| 模式数量 | 5种 | 3种 |
| 使用难度 | 需要了解内部细节 | 三步上手 |
| 代码量 | 多 | 精简50% |

## 10. 常见问题

**Q: 如何切换前进/后退？**
A: 使用正负速度值：
```c
Control_SetSpeed(0.5f);   // 前进
Control_SetSpeed(-0.5f);  // 后退
```

**Q: 如何调整转弯精度？**
A: 调整 `pid_angle.h` 中的PID参数：
- 增大KP：响应更快但可能震荡
- 减小KP：更平滑但响应慢

**Q: 电机不转怎么办？**
A: 检查：
1. 是否调用了 `Control_Enable()`
2. 是否在10ms周期中调用 `Control_Update()`
3. MPU6050 DMP是否正常工作

**Q: 如何实现原地旋转？**
A: 设置正负速度差：
```c
// 四驱模式
Speed_PID_SetTargetSpeed(&Speed_PID_FR, 0.3f);
Speed_PID_SetTargetSpeed(&Speed_PID_FL, -0.3f);
Speed_PID_SetTargetSpeed(&Speed_PID_BR, 0.3f);
Speed_PID_SetTargetSpeed(&Speed_PID_BL, -0.3f);
```
