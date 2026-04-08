/**
 * @file control.h
 * @brief 简洁的智能车双环PID控制模块
 * @version 2.0
 * @date 2026-04-06
 *
 * @details 简化的双环PID控制，使用MPU6050 DMP角度
 *          - 速度环：增量式PID控制电机转速
 *          - 角度环：位置式PID控制车体方向
 *          - 简洁API：初始化->设置参数->启用->周期更新
 */

#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#define ANGLE_PID_BUFFER_SIZE 10 // 滤波缓冲区大小

/* ==================== 控制模式 ==================== */
typedef enum {
  CONTROL_MODE_STOP = 0,     // 停止
  CONTROL_MODE_STRAIGHT = 1, // 直线行驶
  CONTROL_MODE_TURN = 2      // 转弯
} Control_Mode_e;

/* ==================== 控制状态 ==================== */

typedef struct {
  Control_Mode_e mode;                       // 控制模式
  float target_speed;                        // 目标速度 (m/s)
  float target_angle;                        // 目标角度 (度)
  float current_angle;                       // 当前角度 (度)
  float angle_compensation;                  // 角度补偿值
  float angle_buffer[ANGLE_PID_BUFFER_SIZE]; // 角度缓冲区（用于滑动平均滤波）
  uint8_t angle_buffer_index;                // 缓冲区索引
  uint8_t enabled;                           // 是否启用
} Control_State_t;

/* ==================== 公共函数 ==================== */

/**
 * @brief 初始化控制模块
 */
void Control_Init(void);

/**
 * @brief 启用控制
 */
void Control_Enable(void);

/**
 * @brief 禁用控制
 */
void Control_Disable(void);

/**
 * @brief 设置速度
 * @param speed 速度 (m/s)，正=前进，负=后退
 */
void Control_SetSpeed(float speed);

/**
 * @brief 设置目标角度
 * @param angle 目标角度 (度)，正=右转，负=左转
 */
void Control_SetAngle(float angle);

/**
 * @brief 设置控制模式
 * @param mode 控制模式
 */
void Control_SetMode(Control_Mode_e mode);

/**
 * @brief 控制更新 (10ms周期调用)
 * @note 从MPU6050 DMP获取角度并执行双环PID控制
 */
void Control_Update(void);

/**
 * @brief 停止运动
 */
void Control_Stop(void);

/**
 * @brief 获取控制状态
 * @return 控制状态指针
 */
Control_State_t *Control_GetState(void);

#endif /* __CONTROL_H */
