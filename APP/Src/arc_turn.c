/**
 * @file arc_turn.c
 * @brief 圆弧转弯控制模块实现 - 智能车专用
 * @version 1.0
 * @date 2026-04-05
 *
 * @details 本模块实现圆弧转弯功能
 *          - 基于运动学模型计算左右轮速度分配
 *          - 支持顺时针和逆时针转弯
 *          - 自动设置PID控制器的目标速度
 */

#include "arc_turn.h"
#include "delay.h"
#include <math.h>

/* ==================== 全局变量定义 ==================== */

/* ==================== 私有函数声明 ==================== */

/**
 * @brief 根据转弯半径和中心速度计算左右轮速度
 * @param radius 转弯半径（单位：米）
 * @param center_speed 中心目标速度（单位：米/秒）
 * @param direction 转弯方向
 * @param left_speed 输出左轮速度（单位：米/秒）
 * @param right_speed 输出右轮速度（单位：米/秒）
 */
static void calculate_wheel_speeds(float radius, float center_speed,
                                   ArcTurn_Direction_e direction,
                                   float *left_speed, float *right_speed);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief 初始化圆弧转弯控制器
 * @param controller 控制器实例指针
 * @param left_pid 左轮PID控制器指针
 * @param right_pid 右轮PID控制器指针
 */
void FArcTurn_Init(ArcTurn_Controller_t *controller,
                  Speed_PID_Controller_t *left_pid,
                  Speed_PID_Controller_t *right_pid) {
  // 参数检查
  if (controller == NULL || left_pid == NULL || right_pid == NULL) {
    return;
  }

  // 初始化控制参数
  controller->turn_radius = MAX_TURN_RADIUS;  // 默认大半径（接近直线）
  controller->center_speed_m_s = 0.0f;        // 默认速度为0
  controller->direction = ARC_TURN_DIR_STRAIGHT;

  // 初始化计算结果
  controller->left_speed_m_s = 0.0f;
  controller->right_speed_m_s = 0.0f;

  // 初始化状态信息
  controller->enabled = 0;  // 默认禁用
  controller->last_update_time = GetSysTick();

  // 保存关联的PID控制器指针
  controller->left_pid = left_pid;
  controller->right_pid = right_pid;
}

/**
 * @brief 设置圆弧转弯参数
 * @param controller 控制器实例指针
 * @param radius 转弯半径（单位：米）
 * @param center_speed 中心目标速度（单位：米/秒）
 * @param direction 转弯方向（顺时针/逆时针）
 */
void ArcTurn_SetParameters(ArcTurn_Controller_t *controller,
                           float radius, float center_speed,
                           ArcTurn_Direction_e direction) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 限制转弯半径范围
  if (fabsf(radius) < MIN_TURN_RADIUS) {
    controller->turn_radius = MIN_TURN_RADIUS;
  } else if (fabsf(radius) > MAX_TURN_RADIUS) {
    controller->turn_radius = MAX_TURN_RADIUS;
  } else {
    controller->turn_radius = fabsf(radius);
  }

  // 限制中心速度范围（防止超过电机能力）
  if (center_speed < 0.0f) {
    controller->center_speed_m_s = 0.0f;
  } else if (center_speed > 2.0f) {
    controller->center_speed_m_s = 2.0f;  // 最大速度2m/s
  } else {
    controller->center_speed_m_s = center_speed;
  }

  // 设置转弯方向
  controller->direction = direction;

  // 计算左右轮速度
  calculate_wheel_speeds(controller->turn_radius, controller->center_speed_m_s,
                         controller->direction, &controller->left_speed_m_s,
                         &controller->right_speed_m_s);
}

/**
 * @brief 启用圆弧转弯控制
 * @param controller 控制器实例指针
 */
void ArcTurn_Enable(ArcTurn_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 启用控制器
  controller->enabled = 1;

  // 立即计算并设置一次速度
  calculate_wheel_speeds(controller->turn_radius, controller->center_speed_m_s,
                         controller->direction, &controller->left_speed_m_s,
                         &controller->right_speed_m_s);

  // 设置左右轮PID控制器的目标速度
  if (controller->left_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->left_pid, controller->left_speed_m_s);
  }
  if (controller->right_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->right_pid, controller->right_speed_m_s);
  }
}

/**
 * @brief 禁用圆弧转弯控制
 * @param controller 控制器实例指针
 */
void ArcTurn_Disable(ArcTurn_Controller_t *controller) {
  // 参数检查
  if (controller == NULL) {
    return;
  }

  // 禁用控制器
  controller->enabled = 0;

  // 停止电机（设置目标速度为0）
  if (controller->left_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->left_pid, 0.0f);
  }
  if (controller->right_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->right_pid, 0.0f);
  }
}

/**
 * @brief 更新圆弧转弯控制
 * @param controller 控制器实例指针
 * @note 应在控制循环中调用，计算并设置左右轮目标速度
 */
void ArcTurn_Update(ArcTurn_Controller_t *controller) {
  // 参数检查
  if (controller == NULL || !controller->enabled) {
    return;
  }

  // 检查更新周期
  uint32_t current_time = GetSysTick();
  if (current_time - controller->last_update_time < ARC_TURN_UPDATE_PERIOD_MS) {
    return;
  }

  // 计算左右轮速度
  calculate_wheel_speeds(controller->turn_radius, controller->center_speed_m_s,
                         controller->direction, &controller->left_speed_m_s,
                         &controller->right_speed_m_s);

  // 设置左右轮PID控制器的目标速度
  if (controller->left_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->left_pid, controller->left_speed_m_s);
  }
  if (controller->right_pid != NULL) {
    Speed_PID_SetTargetSpeed(controller->right_pid, controller->right_speed_m_s);
  }

  // 更新时间戳
  controller->last_update_time = current_time;
}

/**
 * @brief 获取当前计算的左右轮速度
 * @param controller 控制器实例指针
 * @param left_speed 输出左轮速度（单位：米/秒）
 * @param right_speed 输出右轮速度（单位：米/秒）
 */
void ArcTurn_GetWheelSpeeds(ArcTurn_Controller_t *controller,
                           float *left_speed, float *right_speed) {
  // 参数检查
  if (controller == NULL || left_speed == NULL || right_speed == NULL) {
    return;
  }

  *left_speed = controller->left_speed_m_s;
  *right_speed = controller->right_speed_m_s;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 根据转弯半径和中心速度计算左右轮速度
 * @param radius 转弯半径（单位：米）
 * @param center_speed 中心目标速度（单位：米/秒）
 * @param direction 转弯方向
 * @param left_speed 输出左轮速度（单位：米/秒）
 * @param right_speed 输出右轮速度（单位：米/秒）
 */
static void calculate_wheel_speeds(float radius, float center_speed,
                                   ArcTurn_Direction_e direction,
                                   float *left_speed, float *right_speed) {
  // 计算轮距的一半
  float half_wheel_base = WHEEL_BASE / 2.0f;

  // 根据转弯方向计算左右轮速度
  switch (direction) {
    case ARC_TURN_DIR_CW:  // 顺时针转弯（左轮为内侧轮）
      // 左轮（内侧）：V_left = V_center × (R - W/2) / R
      *left_speed = center_speed * (radius - half_wheel_base) / radius;
      // 右轮（外侧）：V_right = V_center × (R + W/2) / R
      *right_speed = center_speed * (radius + half_wheel_base) / radius;
      break;

    case ARC_TURN_DIR_CCW:  // 逆时针转弯（右轮为内侧轮）
      // 右轮（内侧）：V_right = V_center × (R - W/2) / R
      *right_speed = center_speed * (radius - half_wheel_base) / radius;
      // 左轮（外侧）：V_left = V_center × (R + W/2) / R
      *left_speed = center_speed * (radius + half_wheel_base) / radius;
      break;

    case ARC_TURN_DIR_STRAIGHT:  // 直线行驶
      // 左右轮速度相同
      *left_speed = center_speed;
      *right_speed = center_speed;
      break;

    default:
      // 默认为直线行驶
      *left_speed = center_speed;
      *right_speed = center_speed;
      break;
  }

  // 确保速度不为负数（防止反向运动）
  if (*left_speed < 0.0f) {
    *left_speed = 0.0f;
  }
  if (*right_speed < 0.0f) {
    *right_speed = 0.0f;
  }
}