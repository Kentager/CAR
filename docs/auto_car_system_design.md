# 自动行驶小车系统设计说明

## 📋 任务概述

### 场地描述
- 白色哑光场地：≥220cm × 120cm
- 两个对称黑色半圆弧：半径40cm，线宽2cm
- 圆弧四顶点：A、B、C、D

### 核心行驶任务
1. **任务1**：A点出发→B点停车（声光提示），完成用时≤18秒
2. **任务2**：A→B→C→D→A（沿半弧线），每经过A/B/C/D点声光提示1次，完成一圈用时≤35秒
3. **任务3**：A→C→B→D→A（沿半弧线），每经过A/B/C/D点声光提示1次，完成一圈用时≤45秒
4. **任务4**：按任务3路径连续自动行驶3圈，无故障且用时越少越好

## 🎯 系统整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                   应用层                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  任务管理器  │→ │  状态机     │→ │  路径规划   │  │
│  │ TaskManager  │  │ StateMachine │  │ PathPlanner │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   控制层 (已实现)                         │
│  ┌──────────────┐  ┌──────────────┐                      │
│  │  角度环PID   │  │  速度环PID   │                      │
│  │ Angle_PID    │  │ Speed_PID    │                      │
│  └──────────────┘  └──────────────┘                      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   传感器层 (已实现)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  TCRT循迹    │  │  编码器     │  │  MPU6050    │  │
│  │  TCRT_Lines  │  │  Encoders   │  │  IMU_Angle  │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   硬件层 (已实现)                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  TB6612驱动  │  │  编码器硬件  │  │  传感器硬件  │  │
│  │  Motors      │  │  Enc_HW     │  │  Sensors    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 🔧 已有模块

### 1. 硬件驱动层
- **电机驱动**：TB6612双驱动，支持四路直流电机
- **编码器**：四路正交编码器，用于测速和里程计算
- **IMU**：MPU6050+HMC5883L，卡尔曼滤波姿态解算
- **循迹传感器**：8路TCRT5000红外传感器

### 2. 控制算法层
- **速度环PID**：增量式PID，抗积分饱和，适合速度控制
- **角度环PID**：位置式PID，高精度角度定位
- **双环控制**：速度+角度闭环控制

### 3. 基础功能
- `GoStraight(float distance)`：基于编码器里程的直线行驶
- `Control_SetSpeed()`：设置目标速度
- `Control_SetAngle()`：设置目标角度
- 任务调度系统

## 🚀 新增模块设计

### 1️⃣ 任务管理器 (task_manager.h/c)

#### 任务类型枚举
```c
typedef enum {
  TASK_TYPE_NONE = 0,
  TASK_TYPE_AB_STRAIGHT,      // 任务1：A→B直线
  TASK_TYPE_ABCD_LOOP,        // 任务2：A→B→C→D→A
  TASK_TYPE_ACBD_LOOP,        // 任务3：A→C→B→D→A
  TASK_TYPE_ACBD_3LOOPS       // 任务4：A→C→B→D→A 3圈
} Task_Type_e;
```

#### 任务状态结构体
```c
typedef struct {
  Task_Type_e current_task;     // 当前任务类型
  uint8_t loop_count;          // 圈数计数
  uint8_t completed;           // 任务完成标志
  float start_time;            // 任务开始时间
  float total_time;            // 总用时
  uint8_t position_count[4];  // 各位置经过次数
} Task_State_t;
```

#### 核心函数
```c
// 任务管理器初始化
void TaskManager_Init(void);

// 启动指定任务
void TaskManager_StartTask(Task_Type_e task_type);

// 任务更新（周期调用）
void TaskManager_Update(void);

// 获取任务状态
Task_State_t* TaskManager_GetState(void);

// 停止当前任务
void TaskManager_Stop(void);
```

### 2️⃣ 位置检测系统 (position_detector.h/c)

#### 位置枚举
```c
typedef enum {
  POSITION_A = 0,    // A点（左前）
  POSITION_B = 1,    // B点（右前）
  POSITION_C = 2,    // C点（右后）
  POSITION_D = 3,    // D点（左后）
  POSITION_UNKNOWN = 4
} Position_e;
```

#### 检测方法
1. **里程计检测**：基于编码器累计距离 + 姿态角
2. **循迹检测**：基于8路TCRT传感器识别路径特征
3. **混合检测**：里程计+循迹双重验证

#### 核心函数
```c
// 位置检测器初始化
void PositionDetector_Init(void);

// 更新位置检测（周期调用）
Position_e PositionDetector_Update(void);

// 获取当前位置
Position_e PositionDetector_GetCurrent(void);

// 重置位置检测
void PositionDetector_Reset(void);
```

#### 检测算法
```c
// 里程计算公式
float distance = encoder_distance;
float angle = imu_yaw_angle;

// 位置判断
if (distance < DISTANCE_AB && angle < ANGLE_THRESHOLD) {
  return POSITION_A;
}
// ... 其他位置判断
```

### 3️⃣ 路径规划器 (path_planner.h/c)

#### 路径段类型
```c
typedef enum {
  PATH_SEGMENT_STRAIGHT,      // 直线段
  PATH_SEGMENT_ARC,          // 圆弧段
  PATH_SEGMENT_POINT_CHECK    // 检查点
} Path_Segment_e;
```

#### 路径节点
```c
typedef struct {
  Position_e from;           // 起始点
  Position_e to;             // 终点
  Path_Segment_e type;       // 路径类型
  float distance;           // 目标距离
  float target_angle;       // 目标角度
  float arc_radius;         // 圆弧半径（40cm）
  float speed;             // 建议速度
} Path_Node_t;
```

#### 核心函数
```c
// 路径规划器初始化
void PathPlanner_Init(void);

// 获取当前路径节点
Path_Node_t* PathPlanner_GetCurrentNode(void);

// 路径节点完成，切换到下一个
void PathPlanner_NextNode(void);

// 加载指定任务的路径
void PathPlanner_LoadPath(Task_Type_e task_type);
```

### 4️⃣ 声光提示系统 (position_alert.h/c)

#### 提示状态
```c
typedef struct {
  Position_e last_position;     // 上次位置
  uint32_t last_beep_time;    // 上次提示时间
  uint8_t beep_count[4];      // 各点提示次数
  uint8_t alert_enabled;      // 提示使能
} Position_Alert_t;
```

#### 核心函数
```c
// 声光提示系统初始化
void PositionAlert_Init(void);

// 触发位置提示
void PositionAlert_Trigger(Position_e position);

// 更新声光状态（周期调用）
void PositionAlert_Update(void);

// 启用/禁用提示
void PositionAlert_Enable(uint8_t enable);
```

#### 提示行为
- 检测到位置变化时触发
- LED闪烁（建议绿色）
- 蜂鸣器短鸣（200ms）
- 串口输出：`"到达A点\n"`
- 记录提示次数

## 📐 路径分析

### 场地几何分析
```
    A ───────────────── B
    ╱                  ╲
   ╱                    ╲
  |                      |
   ╲                    ╱
    ╲                  ╱
    D ───────────────── C
```

### 任务路径详解

#### 任务1：A→B直线
- 距离：约1m（场地宽度的50%）
- 路径：直线行驶
- 控制：`GoStraight(1.1f)`

#### 任务2：A→B→C→D→A
1. A→B：前半圆弧（右侧）
2. B→C：直线段（场地长度）
3. C→D：后半圆弧（左侧）
4. D→A：直线段（场地长度）
- 总距离：约2m + 2×π×0.4m ≈ 4.51m

#### 任务3：A→C→B→D→A
1. A→C：对角线穿越前半场
2. C→B：前半圆弧（左侧）
3. B→D：对角线穿越后半场
4. D→A：后半圆弧（右侧）

## 🎛️ 关键参数配置

### 任务参数
```c
#define TASK1_DISTANCE 1.0f           // A→B距离
#define TASK2_TARGET_TIME 35.0f       // 任务2目标时间
#define TASK3_TARGET_TIME 45.0f       // 任务3目标时间
#define TASK4_LOOP_COUNT 3            // 任务4圈数

// 场地参数
#define FIELD_LENGTH 2.2f             // 场地长度 (220cm)
#define FIELD_WIDTH 1.2f              // 场地宽度 (120cm)
#define ARC_RADIUS 0.4f               // 半圆弧半径 (40cm)
```

### 速度配置
```c
#define SPEED_STRAIGHT 0.4f          // 直线速度
#define SPEED_ARC 0.25f               // 圆弧速度
#define SPEED_ARC_OUTER 0.3f          // 外轮速度
#define SPEED_ARC_INNER 0.2f          // 内轮速度
#define SPEED_START 0.15f             // 启动速度
#define SPEED_STOP 0.0f               // 停止速度
```

### 控制参数
```c
// 位置检测
#define POSITION_DETECTION_TOLERANCE 0.05f    // 位置检测容差 (5cm)
#define POSITION_DETECTION_ANGLE 15.0f       // 角度容差 (15度)

// 声光提示
#define POSITION_BEEP_DURATION 200           // 声光提示时长 (ms)
#define POSITION_BEEP_COUNT_MAX 4           // 最大提示次数

// PID参数（可调）
#define SPEED_PID_KP 6000.0f
#define SPEED_PID_KI 100000.0f
#define SPEED_PID_KD 10.0f

#define ANGLE_PID_KP 2.0f
#define ANGLE_PID_KI 0.1f
#define ANGLE_PID_KD 0.5f
```

## 🔄 实现流程

### 阶段1：基础导航
1. **直线行驶控制**（任务1：A→B）
   - 使用已有的 `GoStraight()` 函数
   - 距离：约1.0m
   - 速度：建议0.3-0.5 m/s
   - 目标时间：≤18秒

### 阶段2：循迹导航
2. **圆弧循迹控制**
   - 基于8路TCRT传感器
   - 加权平均法计算偏差
   - 差速控制保持圆弧轨迹
   - 外轮速度 > 内轮速度

### 阶段3：任务编排
3. **多任务状态机**
   ```c
   switch(current_task) {
     case TASK_TYPE_AB_STRAIGHT:
       // A→B直线
       break;
     case TASK_TYPE_ABCD_LOOP:
       // A→B→C→D→A
       break;
     case TASK_TYPE_ACBD_LOOP:
       // A→C→B→D→A
       break;
     case TASK_TYPE_ACBD_3LOOPS:
       // A→C→B→D→A 3圈
       break;
   }
   ```

### 阶段4：优化调试
4. **性能优化**
   - 调整速度环PID参数
   - 优化角度环控制精度
   - 测试不同速度下的稳定性
   - 优化位置检测精度

## 🎮 用户接口

### 串口命令
```c
// 任务启动命令
"task1"  - 启动任务1（A→B直线）
"task2"  - 启动任务2（A→B→C→D→A）
"task3"  - 启动任务3（A→C→B→D→A）
"task4"  - 启动任务4（3圈）
"stop"   - 停止当前任务
"status" - 查询任务状态

// 参数调优命令
"speed 0.4"  - 设置速度
"kp 6000"    - 调整PID参数
```

### 状态输出
```c
// 位置提示
"到达A点，当前圈数：1/3\n"

// 任务完成
"任务1完成，用时：15.3秒\n"
"任务2完成，用时：33.8秒\n"

// 错误提示
"位置检测失败\n"
"超时警告\n"
```

## 📊 性能指标

### 目标指标
| 任务 | 路径 | 目标时间 | 预期距离 |
|------|------|----------|----------|
| 任务1 | A→B | ≤18s | 1.1m |
| 任务2 | A→B→C→D→A | ≤35s | 4.7m |
| 任务3 | A→C→B→D→A | ≤45s | 约5.0m |
| 任务4 | 任务3×3圈 | 越少越好 | 约15.0m |

### 实时监控
- 当前位置
- 当前速度
- 行驶距离
- 任务进度
- 预计剩余时间

## 🔧 调试指南

### 1. 基础测试
```c
// 测试直线行驶
GoStraight(0.5f);  // 前行0.5米

// 测试转向
Control_SetAngle(90.0f);  // 右转90度
```

### 2. 循迹测试
```c
// 测试循迹算法
Control_SetMode(CONTROL_MODE_STRAIGHT);
TCRT_Update();  // 更新循迹数据
```

### 3. 位置检测测试
```c
// 测试位置检测
Position_e pos = PositionDetector_Update();
printf("当前位置: %d\n", pos);
```

### 4. 完整任务测试
```c
// 启动任务2
TaskManager_StartTask(TASK_TYPE_ABCD_LOOP);

// 周期更新
while(1) {
  TaskManager_Update();
  Task_Scheduler();
}
```

## 📝 设计文件结构

```
USER/Src/
├── task_manager.c         # 任务管理器实现
├── position_detector.c    # 位置检测实现
├── path_planner.c         # 路径规划实现
├── position_alert.c      # 声光提示实现
└── main_test.c           # 主程序（含状态机）

USER/Inc/
├── task_manager.h         # 任务管理器接口
├── position_detector.h    # 位置检测接口
├── path_planner.h         # 路径规划接口
└── position_alert.h      # 声光提示接口
```

## 🚦 状态机设计

```c
typedef enum {
  STATE_IDLE,              // 空闲态
  STATE_START,             // 启动态
  STATE_MOVING,            // 运行态
  STATE_POSITION_CHECK,     // 位置检查态
  STATE_STOPPING,          // 停止态
  STATE_COMPLETED          // 完成态
} System_State_e;

void StateMachine_Update(void) {
  switch(current_state) {
    case STATE_IDLE:
      // 等待任务启动
      break;
    case STATE_START:
      // 初始化任务参数
      current_state = STATE_MOVING;
      break;
    case STATE_MOVING:
      // 执行路径控制
      if (need_position_check) {
        current_state = STATE_POSITION_CHECK;
      }
      break;
    case STATE_POSITION_CHECK:
      // 检查是否到达目标位置
      if (position_reached) {
        PositionAlert_Trigger(position);
        if (task_completed) {
          current_state = STATE_STOPPING;
        } else {
          current_state = STATE_MOVING;
        }
      }
      break;
    case STATE_STOPPING:
      // 停止电机
      current_state = STATE_COMPLETED;
      break;
    case STATE_COMPLETED:
      // 输出完成信息
      current_state = STATE_IDLE;
      break;
  }
}
```

## ⚠️ 注意事项

1. **启动安全**：启动时使用较低速度，避免打滑
2. **位置检测**：采用双重验证，提高检测准确性
3. **速度控制**：圆弧段适当降低速度，保证轨迹精度
4. **时间监控**：实时监控任务时间，避免超时
5. **异常处理**：设计故障检测和恢复机制

## 🎯 下一步工作

1. **实现核心模块**
   - [ ] 任务管理器
   - [ ] 位置检测系统
   - [ ] 路径规划器
   - [ ] 声光提示系统

2. **集成测试**
   - [ ] 单个任务测试
   - [ ] 多任务切换测试
   - [ ] 时间性能测试
   - [ ] 长时间稳定性测试

3. **优化调优**
   - [ ] PID参数优化
   - [ ] 速度参数优化
   - [ ] 位置检测优化
   - [ ] 功耗优化

4. **文档完善**
   - [ ] 用户手册
   - [ ] 开发者文档
   - [ ] 调试指南
   - [ ] 测试报告

---

*文档版本：v1.0*
*创建日期：2026-04-07*
*项目：自动行驶小车 - 大学生电子设计竞赛校赛*
