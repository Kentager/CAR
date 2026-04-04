#ifndef PID_SPEED_H
#define PID_SPEED_H

#include "encoder.h"
#include "motor.h"
#include <stddef.h>

/* ==================== 速度环PID参数配置 ==================== */
// 默认PID参数（可根据实际调试调整）
#define SPEED_PID_KP_DEFAULT 36000.0f // 比例系数
#define SPEED_PID_KI_DEFAULT 500000.0f // 积分系数
#define SPEED_PID_KD_DEFAULT 100.0f // 微分系数
#define SPEED_PID_BUFFER_SIZE 10 // 滤波缓冲区大小
// 采样周期 (ms)
#define SPEED_PID_SAMPLE_PERIOD_MS 5

// 最大输出限制
#define SPEED_PID_OUTPUT_MAX MOTOR_PWM_MAX_DUTY

// 编码器方向映射到电机方向
#define ENCODER_TO_MOTOR_DIR(dir)                                              \
  ((dir) == ENCODER_DIR_FORWARD    ? MOTOR_DIR_FORWARD                         \
   : (dir) == ENCODER_DIR_BACKWARD ? MOTOR_DIR_BACKWARD                        \
                                   : MOTOR_DIR_STOP)

/* ==================== PID内部数据结构 ==================== */
/**
 * @brief 增量式PID控制内部状态结构体
 * @note 增量式PID公式: Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k)·dt +
 * Kd·[e(k)-2e(k-1)+e(k-2)]/dt 输出: u(k) = u(k-1) + Δu(k)
 */
typedef struct {
  float kp;           // 比例系数
  float ki;           // 积分系数
  float kd;           // 微分系数
  float target_value; // 目标值
  float last_error;   // 上次误差值 e(k-1)
  float last_error2;  // 上上次误差值 e(k-2)
  float last_output;  // 上次输出值 u(k-1)
} IncrementalPID_State_t;

/* ==================== 速度环PID数据结构体 ==================== */
/**
 * @brief 速度环PID控制数据结构体
 */
typedef struct {
  // 配置参数
  float target_speed_m_s; // 目标线速度 (m/s)
  float angle_deviation_speed_m_s; // 角度偏差速度 (m/s)
  uint8_t angle_deviation_enabled; // 角度偏差速度启用标志
  Encoder_Id_e encoder_id; // 关联的编码器ID
  Motor_Id_e motor_id;     // 关联的电机ID

  // 状态信息
  float current_speed_m_s;   // 当前线速度 (m/s)
  uint8_t enabled;           // 是否启用
  uint32_t last_update_time; // 上次更新时间

  float filtered_speed_m_s;  // 滤波后的速度
  float speed_buffer[SPEED_PID_BUFFER_SIZE];     // 速度缓冲区（用于滑动平均滤波）
  uint8_t speed_buffer_index; // 缓冲区索引

  // PID参数和状态
  IncrementalPID_State_t pid_state;
} Speed_PID_Controller_t;

// 在头文件中声明全局控制器实例供外部使用
extern Speed_PID_Controller_t Speed_PID_Right;
extern Speed_PID_Controller_t Speed_PID_Left;

/* ==================== 全局函数声明 ==================== */

/**
 * @brief 初始化速度环PID控制器
 * @param controller 控制器实例指针
 * @param encoder_id 关联的编码器ID
 * @param motor_id 关联的电机ID
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void Speed_PID_Init(Speed_PID_Controller_t *controller, Encoder_Id_e encoder_id,
                    Motor_Id_e motor_id, float kp, float ki, float kd);

/**
 * @brief 设置速度环PID目标速度
 * @param controller 控制器实例指针
 * @param speed_m_s 目标线速度 (m/s)
 */
void Speed_PID_SetTargetSpeed(Speed_PID_Controller_t *controller,
                              float speed_m_s);

/**
 * @brief 启用速度环PID控制
 * @param controller 控制器实例指针
 */
void Speed_PID_Enable(Speed_PID_Controller_t *controller);

/**
 * @brief 禁用速度环PID控制
 * @param controller 控制器实例指针
 */
void Speed_PID_Disable(Speed_PID_Controller_t *controller);

/**
 * @brief 执行速度环PID控制计算
 * @param controller 控制器实例指针
 * @note 应在10ms控制循环中调用
 */
void Speed_PID_Update(Speed_PID_Controller_t *controller);

/**
 * @brief 更改输入速度环PID控制器的目标速度（是否使用角度环）
 * @param controller 控制器实例指针
 * @param enabled 是否启用角度偏差速度(1:启动 0:关闭)
 * @note 状态机中状态更新调用
 */
void Speed_PID_Deviation_Change(Speed_PID_Controller_t *controller,
                                uint8_t enabled);

/**
 * @brief 更改输入速度环PID控制器的目标速度（是否使用角度环）
 * @param controller 控制器实例指针
 * @param angle_deviation_speed_m_s 角度偏差速度 (m/s)
 * @note 在PID速度角度环中周期调用
 */
void Speed_PID_Angle_Deviation_Speed_Change(Speed_PID_Controller_t *controller, float angle_deviation_speed_m_s);


#endif /* PID_SPEED_H */