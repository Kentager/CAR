#include "pid_speed.h"
#include "delay.h"

/* ==================== 全局变量定义 ==================== */
// 改为外部引用，在main.c中定义
Speed_PID_Controller_t Speed_PID_Right; // 右轮速度PID控制器
Speed_PID_Controller_t Speed_PID_Left;  // 左轮速度PID控制器

/* ==================== 私有函数声明 ==================== */
static float get_encoder_speed_m_s(Encoder_Id_e encoder_id);
static void update_motor_output(Speed_PID_Controller_t *controller,
                                float output);
static void pid_init(IncrementalPID_State_t *pid, float target, float kp, float ki,
                     float kd);

/* ==================== 公有函数实现 ==================== */

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
                    Motor_Id_e motor_id, float kp, float ki, float kd) {
  // 参数检查
  if (controller == NULL || encoder_id >= ENCODER_MAX ||
      motor_id >= MOTOR_MAX) {
    return;
  }

  // 初始化PID参数和状态
  pid_init(&controller->pid_state, 0.0f, kp, ki, kd);

  // 设置关联的编码器和电机
  controller->encoder_id = encoder_id;
  controller->motor_id = motor_id;

  // 初始化其他参数
  controller->target_speed_m_s = 0.0f;
  controller->current_speed_m_s = 0.0f;
  controller->enabled = 0;
  controller->last_update_time = GetSysTick();

   // 初始化滤波器
  controller->filtered_speed_m_s = 0.0f;
  for (int i = 0; i < SPEED_PID_BUFFER_SIZE; i++) {
    controller->speed_buffer[i] = 0.0f;
  }
  controller->speed_buffer_index = 0;
}

/**
 * @brief 设置速度环PID目标速度
 * @param controller 控制器实例指针
 * @param speed_m_s 目标线速度 (m/s)
 */
void Speed_PID_SetTargetSpeed(Speed_PID_Controller_t *controller,
                              float speed_m_s) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 设置目标速度
  controller->target_speed_m_s = speed_m_s;
  // 更新PID目标值
  controller->pid_state.target_value = speed_m_s;
}

/**
 * @brief 启用速度环PID控制
 * @param controller 控制器实例指针
 */
void Speed_PID_Enable(Speed_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 启用控制器
  controller->enabled = 1;
}

/**
 * @brief 禁用速度环PID控制
 * @param controller 控制器实例指针
 */
void Speed_PID_Disable(Speed_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 禁用控制器
  controller->enabled = 0;

  // 停止电机
  update_motor_output(controller, 0.0f);

  // 重置PID状态以防止积分饱和
  controller->pid_state.last_error = 0.0f;
  controller->pid_state.last_error2 = 0.0f;
  controller->pid_state.last_output = 0.0f;
}

/**
 * @brief 执行速度环PID控制计算（增量式PID）
 * @param controller 控制器实例指针
 * @note 应在10ms控制循环中调用
 *       增量式PID公式: Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k)·dt +
 * Kd·[e(k)-2e(k-1)+e(k-2)]/dt 输出: u(k) = u(k-1) + Δu(k)
 */
void Speed_PID_Update(Speed_PID_Controller_t *controller) {
  // 参数检查
  if (controller == NULL || !controller->enabled) {
    return;
  }

  // 计算时间间隔（秒）
  uint32_t current_time = GetSysTick();
  float dt =
      (current_time - controller->last_update_time) / 1000.0f; // 转换为秒

  // // 获取当前速度 (m/s)
  // controller->current_speed_m_s = get_encoder_speed_m_s(controller->encoder_id);



  // 获取当前速度 (m/s)
  float raw_speed = get_encoder_speed_m_s(controller->encoder_id);
  
  // 滑动平均滤波
  controller->speed_buffer[controller->speed_buffer_index] = raw_speed;
  controller->speed_buffer_index = (controller->speed_buffer_index + 1) % SPEED_PID_BUFFER_SIZE;
  
  // 计算滤波后的速度
  float sum = 0.0f;
  for (int i = 0; i < SPEED_PID_BUFFER_SIZE; i++) {
    sum += controller->speed_buffer[i];
  }
  controller->current_speed_m_s = sum / (float)SPEED_PID_BUFFER_SIZE;



  // 计算当前误差 e(k)
  float error =
      controller->pid_state.target_value - controller->current_speed_m_s;

  // 增量式PID算法
  // Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k)·dt + Kd·[e(k)-2e(k-1)+e(k-2)]/dt
  float delta_u =
      controller->pid_state.kp * (error - controller->pid_state.last_error) +
      controller->pid_state.ki * error * dt +
      controller->pid_state.kd *
          (error - 2.0f * controller->pid_state.last_error +
           controller->pid_state.last_error2) /
          dt;

  // 计算当前输出: u(k) = u(k-1) + Δu(k)
  float output = controller->pid_state.last_output + delta_u;
 
  // 更新PID状态
  controller->last_update_time = current_time;
  controller->pid_state.last_error2 =
      controller->pid_state.last_error;       // e(k-2) = e(k-1)
  controller->pid_state.last_error = error;   // e(k-1) = e(k)
  controller->pid_state.last_output = output; // u(k-1) = u(k) // 放大100倍，便于电机控制
  // 更新电机输出
  update_motor_output(controller, output);
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 获取编码器测量的速度 (m/s)
 * @param encoder_id 编码器ID
 * @return 当前线速度 (m/s)
 */
static float get_encoder_speed_m_s(Encoder_Id_e encoder_id) {
  // 获取编码器数据
  Encoder_Data_t encoder_data = Encoder_GetData(encoder_id);

  // 返回线速度
  return encoder_data.speed_m_s;
}

/**
 * @brief 更新电机输出
 * @param controller 控制器实例指针
 * @param output PID输出值
 */
static void update_motor_output(Speed_PID_Controller_t *controller,
                                float output) {
  // 限制输出范围
  if (output > SPEED_PID_OUTPUT_MAX) {
    output = SPEED_PID_OUTPUT_MAX;
  } else if (output < -SPEED_PID_OUTPUT_MAX) {
    output = -SPEED_PID_OUTPUT_MAX;
  }

  // 根据输出符号确定方向
  Motor_Direction_e direction;
  if (output > 50) { // 死区设置
    direction = MOTOR_DIR_FORWARD;
  } else if (output < -50) {
    direction = MOTOR_DIR_BACKWARD;
  } else {
    direction = MOTOR_DIR_STOP;
    output = 0; // 停止时输出为0
  }

  // 设置电机方向和速度
  Motor_SetDirection(controller->motor_id, direction);
  Motor_SetSpeed(controller->motor_id, (int16_t)output);

  // 更新硬件
  Motor_Update(controller->motor_id);
}

/**
 * @brief 增量式PID控制器初始化函数
 * @param pid Pointer to the PID state structure
 * @param target Target value for the PID controller
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
static void pid_init(IncrementalPID_State_t *pid, float target, float kp, float ki,
                     float kd) {
  pid->target_value = target;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->last_error = 0.0f;  // e(k-1)
  pid->last_error2 = 0.0f; // e(k-2)
  pid->last_output = 0.0f; // u(k-1)
}

