/**
 * @file pid_angle.h
 * @brief 角度环PID控制头文件 - 智能车专用
 * @version 1.1
 * @date 2026-03-13
 *
 * @details 本文件实现角度环PID控制器（位置式PID）
 *          - 控制目标：车体姿态角度（度）
 *          - 算法：位置式PID，直接输出绝对控制量 u(k)
 *          - 更新频率：5ms（200Hz）
 *          - 反馈源：MPU6050 DMP输出的姿态角（不使用磁力计修正）
 *          - 应用场景：直线行驶偏航修正、转向控制
 *
 * @note   位置式PID公式：u(k) = Kp·e(k) + Ki·∫e(t)dt + Kd·de(t)/dt
 * @note   角度反馈源：使用 mpu_dmp_get_data() 直接获取的 yaw 角度
 */

#ifndef __PID_ANGLE_H
#define __PID_ANGLE_H

#include "delay.h"
#include "jy61p.h"
#include "stm32f4xx.h"
#include "task.h"
#include <math.h>

/* ==================== 角度环PID参数配置 ==================== */
// 默认PID参数（可根据实际调试调整）
#define ANGLE_PID_KP_DEFAULT 0.01f   // 比例系数
#define ANGLE_PID_KI_DEFAULT 0.0f    // 积分系数
#define ANGLE_PID_KD_DEFAULT 0.0008f // 微分系数

// 采样周期 (ms)
#define ANGLE_PID_SAMPLE_PERIOD_MS 5

// 最大输出限制（角度偏差补偿值）
#define ANGLE_PID_OUTPUT_MAX 0.2f // 最大补偿值

// 角度阈值（度）
#define ANGLE_THRESHOLD_DEG 0.8f    // 角度偏差阈值，小于此值不进行补偿
#define ANGLE_INTEGRAL_LIMIT 100.0f // 积分限幅值，防止积分饱和

/* ==================== PID内部数据结构 ==================== */
/**
 * @brief 位置式PID控制内部状态结构体
 * @note 位置式PID公式: u(k) = Kp·e(k) + Ki·∫e(t)dt + Kd·de(t)/dt
 */
typedef struct {
  float kp;           // 比例系数
  float ki;           // 积分系数
  float kd;           // 微分系数
  float target_value; // 目标角度值（度）
  float integral_sum; // 积分累积值 ∫e(t)dt
  float last_error;   // 上次误差值 e(k-1)
} PositionalPID_State_t;

/* ==================== 角度环PID数据结构体 ==================== */
/**
 * @brief 角度环PID控制数据结构体
 */
typedef struct {
  // 配置参数
  float target_angle_deg; // 目标角度（度，通常为0表示直线）
  uint8_t axis;           // 控制轴：0=Yaw(偏航), 1=Pitch(俯仰), 2=Roll(横滚)

  // 状态信息
  float current_angle_deg;   // 当前角度（度）
  float angle_error_deg;     // 角度偏差（度）
  float compensation_value;  // 补偿值（用于调整电机转速差）
  uint8_t enabled;           // 是否启用
  uint32_t last_update_time; // 上次更新时间

  // PID参数和状态
  PositionalPID_State_t pid_state;
} Angle_PID_Controller_t;

/* ==================== 角度轴枚举 ==================== */
/**
 * @brief 角度轴枚举
 */
typedef enum {
  ANGLE_AXIS_YAW = 0,   // 偏航角 - 用于方向控制
  ANGLE_AXIS_PITCH = 1, // 俯仰角 - 用于爬坡/下坡控制
  ANGLE_AXIS_ROLL = 2   // 横滚角 - 用于侧倾控制
} Angle_Axis_e;

// 在头文件中声明全局控制器实例供外部使用
extern Angle_PID_Controller_t Angle_PID_Yaw;   // 偏航角PID控制器
extern Angle_PID_Controller_t Angle_PID_Pitch; // 俯仰角PID控制器
extern Angle_PID_Controller_t Angle_PID_Roll;  // 横滚角PID控制器
/* ==================== 全局函数声明 ==================== */

/**
 * @brief 初始化角度环PID控制器
 * @param controller 控制器实例指针
 * @param axis 控制轴（ANGLE_AXIS_YAW/PITCH/ROLL）
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void Angle_PID_Init(Angle_PID_Controller_t *controller, Angle_Axis_e axis,
                    float kp, float ki, float kd);

/**
 * @brief 设置角度环PID目标角度
 * @param controller 控制器实例指针
 * @param angle_deg 目标角度（度）
 */
void Angle_PID_SetTargetAngle(Angle_PID_Controller_t *controller,
                              float angle_deg);

/**
 * @brief 启用角度环PID控制
 * @param controller 控制器实例指针
 */
void Angle_PID_Enable(Angle_PID_Controller_t *controller);

/**
 * @brief 禁用角度环PID控制
 * @param controller 控制器实例指针
 */
void Angle_PID_Disable(Angle_PID_Controller_t *controller);

/**
 * @brief 执行角度环PID控制计算
 * @param controller 控制器实例指针
 * @param euler_angle 当前欧拉角（从主函数获得）
 * @return 补偿值（正值表示右侧需加速/左侧需减速，负值相反）
 * @note 应在5ms控制循环中调用
 */
float Angle_PID_Update(Angle_PID_Controller_t *controller, float euler_angle);

/**
 * @brief 重置角度环PID积分项
 * @param controller 控制器实例指针
 * @note 在切换控制模式时调用，防止积分饱和
 */
void Angle_PID_Reset(Angle_PID_Controller_t *controller);

#endif /* __PID_ANGLE_H */
