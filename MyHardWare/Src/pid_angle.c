/**
 * @file pid_angle.c
 * @brief 角度环PID控制实现 - 智能车专用
 * @version 1.0
 * @date 2026-03-13
 */

#include "pid_angle.h"

/* ==================== 全局变量定义 ==================== */
Angle_PID_Controller_t Angle_PID_Yaw;   // 偏航角PID控制器
Angle_PID_Controller_t Angle_PID_Pitch; // 俯仰角PID控制器
Angle_PID_Controller_t Angle_PID_Roll;  // 横滚角PID控制器

/* ==================== 私有函数声明 ==================== */
static float get_axis_angle(EulerAngle_Param *euler_angle, uint8_t axis);
static void pid_init(PositionalPID_State_t *pid, float target, float kp, float ki,
                     float kd);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief 初始化角度环PID控制器
 * @param controller 控制器实例指针
 * @param axis 控制轴（ANGLE_AXIS_YAW/PITCH/ROLL）
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void Angle_PID_Init(Angle_PID_Controller_t *controller, Angle_Axis_e axis,
                    float kp, float ki, float kd) {
  // 参数检查
  if (controller == NULL || axis > ANGLE_AXIS_ROLL) {
    return;
  }

  // 初始化PID参数和状态
  pid_init(&controller->pid_state, 0.0f, kp, ki, kd);

  // 设置控制轴
  controller->axis = axis;

  // 初始化其他参数
  controller->target_angle_deg = 0.0f;
  controller->current_angle_deg = 0.0f;
  controller->angle_error_deg = 0.0f;
  controller->compensation_value = 0.0f;
  controller->enabled = 0;
  controller->last_update_time = GetSysTick();
}

/**
 * @brief 设置角度环PID目标角度
 * @param controller 控制器实例指针
 * @param angle_deg 目标角度（度）
 */
void Angle_PID_SetTargetAngle(Angle_PID_Controller_t *controller,
                              float angle_deg) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 设置目标角度
  controller->target_angle_deg = angle_deg;
  // 更新PID目标值
  controller->pid_state.target_value = angle_deg;

  // 如果目标角度发生变化，重置积分项
  Angle_PID_Reset(controller);
}

/**
 * @brief 启用角度环PID控制
 * @param controller 控制器实例指针
 */
void Angle_PID_Enable(Angle_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 启用控制器
  controller->enabled = 1;
}

/**
 * @brief 禁用角度环PID控制
 * @param controller 控制器实例指针
 */
void Angle_PID_Disable(Angle_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 禁用控制器
  controller->enabled = 0;

  // 清零补偿值
  controller->compensation_value = 0.0f;
}

/**
 * @brief 执行角度环PID控制计算（位置式PID）
 * @param controller 控制器实例指针
 * @param euler_angle 当前欧拉角（从mpu6050获取）
 * @return 补偿值（正值表示右侧需加速/左侧需减速，负值相反）
 * @note 应在5ms控制循环中调用
 *       位置式PID公式: u(k) = Kp·e(k) + Ki·∫e(t)dt + Kd·de(t)/dt
 */
float Angle_PID_Update(Angle_PID_Controller_t *controller,
                       float euler_angle) {
  // 参数检查
  if (controller == NULL || !controller->enabled) {
    return 0.0f;
  }

  // 计算时间间隔（秒）
  uint32_t current_time = GetSysTick();
  float dt =
      (current_time - controller->last_update_time) / 1000.0f; // 转换为秒

  // 获取当前角度（根据控制轴）
  controller->current_angle_deg = euler_angle;

  // 计算角度偏差 e(k) = 目标值 - 当前值
  // 对于偏航角，通常目标为0度（直线行驶）
  float error =
      controller->pid_state.target_value - controller->current_angle_deg;

  // 保存角度偏差供外部使用
  controller->angle_error_deg = error;

  // 角度阈值检查：如果偏差小于阈值，不进行补偿（避免频繁微调）
  if (fabsf(error) < ANGLE_THRESHOLD_DEG) {
    controller->last_update_time = current_time;
    controller->pid_state.last_error = error;
    return 0.0f;
  }

  // 位置式PID算法
  // u(k) = Kp·e(k) + Ki·∫e(t)dt + Kd·de(t)/dt

  // 比例项：Kp·e(k)
  float proportional = controller->pid_state.kp * error;

  // 积分项：Ki·∫e(t)dt
  controller->pid_state.integral_sum += error * dt;

  // 积分限幅（防止积分饱和）
  if (controller->pid_state.integral_sum > ANGLE_INTEGRAL_LIMIT) {
    controller->pid_state.integral_sum = ANGLE_INTEGRAL_LIMIT;
  } else if (controller->pid_state.integral_sum < -ANGLE_INTEGRAL_LIMIT) {
    controller->pid_state.integral_sum = -ANGLE_INTEGRAL_LIMIT;
  }

  float integral =
      controller->pid_state.ki * controller->pid_state.integral_sum;

  // 微分项：Kd·de(t)/dt
  float derivative = controller->pid_state.kd *
                     (error - controller->pid_state.last_error) / dt;

  // 计算PID输出
  float output = proportional + integral + derivative;

  // 限制输出范围
  if (output > ANGLE_PID_OUTPUT_MAX) {
    output = ANGLE_PID_OUTPUT_MAX;
  } else if (output < -ANGLE_PID_OUTPUT_MAX) {
    output = -ANGLE_PID_OUTPUT_MAX;
  }

  // 更新PID状态
  controller->last_update_time = current_time;
  controller->pid_state.last_error = error;

  // 保存补偿值供外部使用
  controller->compensation_value = output;

  return output;
}

/**
 * @brief 重置角度环PID积分项
 * @param controller 控制器实例指针
 * @note 在切换控制模式时调用，防止积分饱和
 */
void Angle_PID_Reset(Angle_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 重置PID状态
  controller->pid_state.integral_sum = 0.0f;
  controller->pid_state.last_error = 0.0f;
  controller->compensation_value = 0.0f;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 根据控制轴获取对应角度值
 * @param euler_angle 欧拉角数据结构
 * @param axis 控制轴
 * @return 角度值（度）
 */
static float get_axis_angle(EulerAngle_Param *euler_angle, uint8_t axis) {
  switch (axis) {
  case ANGLE_AXIS_YAW:
    return euler_angle->Yaw;
  case ANGLE_AXIS_PITCH:
    return euler_angle->Pitch;
  case ANGLE_AXIS_ROLL:
    return euler_angle->Roll;
  default:
    return 0.0f;
  }
}

/**
 * @brief 位置式PID控制器初始化函数
 * @param pid Pointer to the PositionalPID state structure
 * @param target Target value for the PID controller
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
static void pid_init(PositionalPID_State_t *pid, float target, float kp, float ki,
                     float kd) {
  pid->target_value = target;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_sum = 0.0f; // ∫e(t)dt
  pid->last_error = 0.0f;   // e(k-1)
}
