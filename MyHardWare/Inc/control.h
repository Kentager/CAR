/**
 * @file control.h
 * @brief 双环PID控制架构头文件 - 智能车专用
 * @version 1.0
 * @date 2026-03-13
 *
 * @details 本文件实现智能车双环PID控制架构
 *          - 速度环（增量式PID）：控制电机线速度
 *          - 角度环（位置式PID）：控制车体姿态角度
 *          - 双环协同工作：速度环作为内环，角度环作为外环
 *
 * @note   双环控制架构：
 *         - 速度环（内环，10ms，100Hz）：基于编码器反馈，增量式PID控制电机转速
 *         - 角度环（外环，5ms，200Hz）：基于JY61P卡尔曼滤波，位置式PID控制车体姿态
 *         - 控制流程：角度环输出 -> 速度环目标修正 -> 电机输出
 *
 * @note   应用场景：
 *         1. 直线行驶：
 *            - 基础层：四路编码器反馈转速，PID独立补偿
 *            - 补偿层：JY61P卡尔曼滤波解算偏航角，调整左右侧电机转速差
 *         2. 转弯控制：
 *            - 小角度转弯（<30°）：基于偏航角闭环控制，差速调节左右侧电机
 *            - 大角度/原地转弯（≥30°）：编码器计数控制每路电机转动距离
 */

#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "encoder.h"
#include "pid_speed.h"
#include "pid_angle.h"
#include "kalman_filter.h"
#include "stm32f4xx.h"

/* ==================== 控制模式枚举 ==================== */
/**
 * @brief 控制模式枚举
 */
typedef enum {
  CONTROL_MODE_STOP = 0,           // 停止模式
  CONTROL_MODE_STRAIGHT = 1,        // 直线行驶模式
  CONTROL_MODE_TURN_SMALL = 2,      // 小角度转弯模式（<30°）
  CONTROL_MODE_TURN_LARGE = 3,      // 大角度转弯模式（≥30°）
  CONTROL_MODE_SPIN = 4             // 原地旋转模式
} Control_Mode_e;

/* ==================== 运动控制参数配置 ==================== */
// 角度阈值（度）
#define TURN_SMALL_ANGLE_THRESHOLD 30.0f // 小角度/大角度转弯阈值

// 速度阈值（m/s）
#define STRAIGHT_SPEED_THRESHOLD 0.1f   // 直线行驶速度阈值

// 转弯速度（m/s）
#define TURN_SPEED_DEFAULT 0.5f         // 默认转弯速度

/* ==================== 控制状态数据结构体 ==================== */
/**
 * @brief 控制状态数据结构体
 */
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

  // 四驱模式专用
#ifdef QUAD_MOTOR_DRIVE
  float speed_fr; // 前右电机速度（m/s）
  float speed_fl; // 前左电机速度（m/s）
  float speed_br; // 后右电机速度（m/s）
  float speed_bl; // 后左电机速度（m/s）
#else
  float speed_right; // 右电机速度（m/s）
  float speed_left;  // 左电机速度（m/s）
#endif

  // 时间戳
  uint32_t last_update_time; // 上次更新时间

  // PID控制器指针
  Speed_PID_Controller_t *speed_pid[MOTOR_COUNT]; // 速度环PID控制器数组
  Angle_PID_Controller_t *angle_pid;              // 角度环PID控制器
} Control_State_t;

// 在头文件中声明全局控制状态实例供外部使用
extern Control_State_t ControlState;

/* ==================== 全局函数声明 ==================== */

/**
 * @brief 初始化控制系统
 * @note 初始化速度环PID控制器、角度环PID控制器、姿态解算系统
 */
void Control_Init(void);

/**
 * @brief 启用控制系统
 */
void Control_Enable(void);

/**
 * @brief 禁用控制系统
 */
void Control_Disable(void);

/**
 * @brief 设置控制模式
 * @param mode 控制模式（CONTROL_MODE_STRAIGHT/TURN_SMALL/TURN_LARGE/SPIN）
 */
void Control_SetMode(Control_Mode_e mode);

/**
 * @brief 获取当前控制模式
 * @return 当前控制模式
 */
Control_Mode_e Control_GetMode(void);

/**
 * @brief 设置目标速度
 * @param speed_m_s 目标线速度（m/s）
 * @note 所有电机以相同速度运行（直线行驶）或作为基准速度（转弯）
 */
void Control_SetSpeed(float speed_m_s);

/**
 * @brief 设置目标转弯角度
 * @param angle_deg 目标转弯角度（度，正值右转，负值左转）
 * @note 用于转弯控制（TURN_SMALL或TURN_LARGE模式）
 */
void Control_SetTurnAngle(float angle_deg);

/**
 * @brief 设置目标角度
 * @param angle_deg 目标角度（度）
 * @note 用于直线行驶时的角度基准（通常为0度）
 */
void Control_SetTargetAngle(float angle_deg);

/**
 * @brief 控制系统更新（10ms，100Hz）
 * @note 执行双环PID控制：
 *       1. 更新姿态解算（JY61P + 卡尔曼滤波）
 *       2. 执行角度环PID控制（5ms，200Hz）
 *       3. 执行速度环PID控制（10ms，100Hz）
 *       4. 应用角度补偿到速度环
 */
void Control_Update(void);

/**
 * @brief 控制系统速度环更新（10ms，100Hz）
 * @note 单独更新速度环，用于独立调试或特殊场景
 */
void Control_UpdateSpeedLoop(void);

/**
 * @brief 控制系统角度环更新（5ms，200Hz）
 * @note 单独更新角度环，用于独立调试或特殊场景
 */
void Control_UpdateAngleLoop(void);

/**
 * @brief 停止所有运动
 * @note 停止所有电机，禁用所有PID控制器
 */
void Control_Stop(void);

/**
 * @brief 刹车所有运动
 * @note 刹车所有电机，禁用所有PID控制器
 */
void Control_Brake(void);

/**
 * @brief 获取当前控制状态
 * @return 控制状态数据结构体指针
 */
Control_State_t *Control_GetState(void);

#endif /* __CONTROL_H */
