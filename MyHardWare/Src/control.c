/**
 * @file control.c
 * @brief 双环PID控制架构实现 - 智能车专用
 * @version 1.0
 * @date 2026-03-13
 */

#include "control.h"
#include "delay.h"
#include <math.h>
#include <string.h>

/* ==================== 全局变量定义 ==================== */
Control_State_t ControlState; // 全局控制状态实例

/* ==================== 私有函数声明 ==================== */
static void apply_angle_compensation_to_speed_loop(void);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief 初始化控制系统
 * @note 初始化速度环PID控制器、角度环PID控制器、姿态解算系统
 */
void Control_Init(void) {
  // 清零控制状态
  memset(&ControlState, 0, sizeof(Control_State_t));

  // 设置初始模式
  ControlState.mode = CONTROL_MODE_STOP;
  ControlState.enabled = 0;

  // 初始化时间戳
  ControlState.last_update_time = GetSysTick();

  // 初始化姿态解算系统
  AttitudeSolver_Init();

  // 初始化角度环PID控制器（偏航角）
  Angle_PID_Init(&Angle_PID_Yaw, ANGLE_AXIS_YAW, ANGLE_PID_KP_DEFAULT,
                  ANGLE_PID_KI_DEFAULT, ANGLE_PID_KD_DEFAULT);

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：初始化4个速度环PID控制器
  Speed_PID_Init(&Speed_PID_FR, ENCODER_FR, MOTOR_FR, SPEED_PID_KP_DEFAULT,
                  SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_FL, ENCODER_FL, MOTOR_FL, SPEED_PID_KP_DEFAULT,
                  SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_BR, ENCODER_BR, MOTOR_BR, SPEED_PID_KP_DEFAULT,
                  SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_BL, ENCODER_BL, MOTOR_BL, SPEED_PID_KP_DEFAULT,
                  SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);

  // 设置速度环PID控制器指针数组
  ControlState.speed_pid[0] = &Speed_PID_FR;
  ControlState.speed_pid[1] = &Speed_PID_FL;
  ControlState.speed_pid[2] = &Speed_PID_BR;
  ControlState.speed_pid[3] = &Speed_PID_BL;
#else
  // 双驱模式：初始化2个速度环PID控制器
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                  SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                  SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                  SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                  SPEED_PID_KD_DEFAULT);

  // 设置速度环PID控制器指针数组
  ControlState.speed_pid[0] = &Speed_PID_Right;
  ControlState.speed_pid[1] = &Speed_PID_Left;
#endif

  // 设置角度环PID控制器指针
  ControlState.angle_pid = &Angle_PID_Yaw;

  // 设置初始目标值
  ControlState.target_speed_m_s = 0.0f;
  ControlState.target_angle_deg = 0.0f;
  ControlState.turn_angle_deg = 0.0f;
}

/**
 * @brief 启用控制系统
 */
void Control_Enable(void) {
  ControlState.enabled = 1;

  // 启用所有速度环PID控制器
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Speed_PID_Enable(ControlState.speed_pid[i]);
  }

  // 启用角度环PID控制器
  Angle_PID_Enable(ControlState.angle_pid);

  // 启动电机定时器
  Motor_StartTimers();
}

/**
 * @brief 禁用控制系统
 */
void Control_Disable(void) {
  ControlState.enabled = 0;

  // 禁用所有速度环PID控制器
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Speed_PID_Disable(ControlState.speed_pid[i]);
  }

  // 禁用角度环PID控制器
  Angle_PID_Disable(ControlState.angle_pid);
}

/**
 * @brief 设置控制模式
 * @param mode 控制模式（CONTROL_MODE_STRAIGHT/TURN_SMALL/TURN_LARGE/SPIN）
 */
void Control_SetMode(Control_Mode_e mode) {
  // 参数检查
  if (mode > CONTROL_MODE_SPIN) {
    return;
  }

  // 如果切换模式，重置角度环PID积分项
  if (ControlState.mode != mode) {
    Angle_PID_Reset(ControlState.angle_pid);
  }

  // 设置控制模式
  ControlState.mode = mode;

  // 根据模式设置目标角度
  switch (mode) {
    case CONTROL_MODE_STRAIGHT:
      // 直线行驶：目标角度为0度
      ControlState.target_angle_deg = 0.0f;
      Angle_PID_SetTargetAngle(ControlState.angle_pid, 0.0f);
      break;

    case CONTROL_MODE_TURN_SMALL:
      // 小角度转弯：设置目标转弯角度
      ControlState.target_angle_deg = ControlState.turn_angle_deg;
      Angle_PID_SetTargetAngle(ControlState.angle_pid, ControlState.turn_angle_deg);
      break;

    case CONTROL_MODE_TURN_LARGE:
      // 大角度转弯：设置目标转弯角度
      ControlState.target_angle_deg = ControlState.turn_angle_deg;
      Angle_PID_SetTargetAngle(ControlState.angle_pid, ControlState.turn_angle_deg);
      break;

    case CONTROL_MODE_SPIN:
      // 原地旋转：目标角度为转弯角度
      ControlState.target_angle_deg = ControlState.turn_angle_deg;
      Angle_PID_SetTargetAngle(ControlState.angle_pid, ControlState.turn_angle_deg);
      break;

    default:
      // 停止模式：不设置目标角度
      break;
  }
}

/**
 * @brief 获取当前控制模式
 * @return 当前控制模式
 */
Control_Mode_e Control_GetMode(void) { return ControlState.mode; }

/**
 * @brief 设置目标速度
 * @param speed_m_s 目标线速度（m/s）
 * @note 所有电机以相同速度运行（直线行驶）或作为基准速度（转弯）
 */
void Control_SetSpeed(float speed_m_s) {
  ControlState.target_speed_m_s = speed_m_s;

  // 设置所有速度环PID控制器的目标速度
  for (int i = 0; i < MOTOR_COUNT; i++) {
    Speed_PID_SetTargetSpeed(ControlState.speed_pid[i], speed_m_s);
  }
}

/**
 * @brief 设置目标转弯角度
 * @param angle_deg 目标转弯角度（度，正值右转，负值左转）
 * @note 用于转弯控制（TURN_SMALL或TURN_LARGE模式）
 */
void Control_SetTurnAngle(float angle_deg) {
  ControlState.turn_angle_deg = angle_deg;

  // 如果当前是转弯模式，更新目标角度
  if (ControlState.mode == CONTROL_MODE_TURN_SMALL ||
      ControlState.mode == CONTROL_MODE_TURN_LARGE ||
      ControlState.mode == CONTROL_MODE_SPIN) {
    ControlState.target_angle_deg = angle_deg;
    Angle_PID_SetTargetAngle(ControlState.angle_pid, angle_deg);
  }
}

/**
 * @brief 设置目标角度
 * @param angle_deg 目标角度（度）
 * @note 用于直线行驶时的角度基准（通常为0度）
 */
void Control_SetTargetAngle(float angle_deg) {
  ControlState.target_angle_deg = angle_deg;
  Angle_PID_SetTargetAngle(ControlState.angle_pid, angle_deg);
}

/**
 * @brief 控制系统更新（10ms，100Hz）
 * @note 执行双环PID控制：
 *       1. 更新姿态解算（JY61P + 卡尔曼滤波）
 *       2. 执行角度环PID控制（5ms，200Hz）
 *       3. 执行速度环PID控制（10ms，100Hz）
 *       4. 应用角度补偿到速度环
 */
void Control_Update(void) {
  // 参数检查
  if (!ControlState.enabled) {
    return;
  }

  // 1. 更新姿态解算（JY61P + 卡尔曼滤波）
  AttitudeSolver_Update();

  // 获取当前角度
  EulerAngle_Param *euler_filtered = AttitudeSolver_GetFilteredAngle();
  ControlState.current_angle_deg = euler_filtered->Yaw;

  // 2. 执行角度环PID控制（仅在直线行驶或小角度转弯模式）
  if (ControlState.mode == CONTROL_MODE_STRAIGHT ||
      ControlState.mode == CONTROL_MODE_TURN_SMALL) {
    float compensation = Angle_PID_Update(ControlState.angle_pid, euler_filtered);
    ControlState.angle_compensation = compensation;
  } else {
    // 其他模式不使用角度补偿
    ControlState.angle_compensation = 0.0f;
  }

  // 3. 应用角度补偿到速度环
  apply_angle_compensation_to_speed_loop();

  // 4. 执行速度环PID控制
  Control_UpdateSpeedLoop();

  // 更新时间戳
  ControlState.last_update_time = GetSysTick();
}

/**
 * @brief 控制系统速度环更新（10ms，100Hz）
 * @note 单独更新速度环，用于独立调试或特殊场景
 */
void Control_UpdateSpeedLoop(void) {
  // 参数检查
  if (!ControlState.enabled) {
    return;
  }

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：更新4个速度环PID控制器
  // 在应用角度补偿后，控制器内部已经处理了补偿值
  Speed_PID_Update(&Speed_PID_FR);
  Speed_PID_Update(&Speed_PID_FL);
  Speed_PID_Update(&Speed_PID_BR);
  Speed_PID_Update(&Speed_PID_BL);

  // 获取当前速度
  ControlState.speed_fr = Speed_PID_FR.current_speed_m_s;
  ControlState.speed_fl = Speed_PID_FL.current_speed_m_s;
  ControlState.speed_br = Speed_PID_BR.current_speed_m_s;
  ControlState.speed_bl = Speed_PID_BL.current_speed_m_s;
#else
  // 双驱模式：更新2个速度环PID控制器
  Speed_PID_Update(&Speed_PID_Right);
  Speed_PID_Update(&Speed_PID_Left);

  // 获取当前速度
  ControlState.speed_right = Speed_PID_Right.current_speed_m_s;
  ControlState.speed_left = Speed_PID_Left.current_speed_m_s;
#endif
}

/**
 * @brief 控制系统角度环更新（5ms，200Hz）
 * @note 单独更新角度环，用于独立调试或特殊场景
 */
void Control_UpdateAngleLoop(void) {
  // 参数检查
  if (!ControlState.enabled) {
    return;
  }

  // 更新姿态解算（JY61P + 卡尔曼滤波）
  AttitudeSolver_Update();

  // 获取当前角度
  EulerAngle_Param *euler_filtered = AttitudeSolver_GetFilteredAngle();
  ControlState.current_angle_deg = euler_filtered->Yaw;

  // 执行角度环PID控制
  float compensation = Angle_PID_Update(ControlState.angle_pid, euler_filtered);
  ControlState.angle_compensation = compensation;

  // 更新时间戳
  ControlState.last_update_time = GetSysTick();
}

/**
 * @brief 停止所有运动
 * @note 停止所有电机，禁用所有PID控制器
 */
void Control_Stop(void) {
  // 停止所有电机
  Motor_StopAll();

  // 禁用控制系统
  Control_Disable();

  // 设置停止模式
  ControlState.mode = CONTROL_MODE_STOP;

  // 清零目标值
  ControlState.target_speed_m_s = 0.0f;
  ControlState.turn_angle_deg = 0.0f;
  ControlState.angle_compensation = 0.0f;
}

/**
 * @brief 刹车所有运动
 * @note 刹车所有电机，禁用所有PID控制器
 */
void Control_Brake(void) {
  // 刹车所有电机
  Motor_BrakeAll();

  // 禁用控制系统
  Control_Disable();

  // 设置停止模式
  ControlState.mode = CONTROL_MODE_STOP;

  // 清零目标值
  ControlState.target_speed_m_s = 0.0f;
  ControlState.turn_angle_deg = 0.0f;
  ControlState.angle_compensation = 0.0f;
}

/**
 * @brief 获取当前控制状态
 * @return 控制状态数据结构体指针
 */
Control_State_t *Control_GetState(void) { return &ControlState; }

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 应用角度补偿到速度环
 * @note 根据角度环输出补偿值，调整左右侧电机转速差
 */
static void apply_angle_compensation_to_speed_loop(void) {
  // 角度补偿值（正值表示右侧需加速/左侧需减速，负值相反）
  float compensation = ControlState.angle_compensation;

  // 如果补偿值为0，不进行调整
  if (fabsf(compensation) < 0.1f) {
    return;
  }

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：调整前后电机转速
  // 右侧电机（前右、后右）补偿为正值
  float speed_fr_target = ControlState.target_speed_m_s + compensation;
  float speed_br_target = ControlState.target_speed_m_s + compensation;

  // 左侧电机（前左、后左）补偿为负值
  float speed_fl_target = ControlState.target_speed_m_s - compensation;
  float speed_bl_target = ControlState.target_speed_m_s - compensation;

  // 应用补偿后的目标速度
  Speed_PID_SetTargetSpeed(&Speed_PID_FR, speed_fr_target);
  Speed_PID_SetTargetSpeed(&Speed_PID_FL, speed_fl_target);
  Speed_PID_SetTargetSpeed(&Speed_PID_BR, speed_br_target);
  Speed_PID_SetTargetSpeed(&Speed_PID_BL, speed_bl_target);
#else
  // 双驱模式：调整左右电机转速
  // 右侧电机补偿为正值
  float speed_right_target = ControlState.target_speed_m_s + compensation;

  // 左侧电机补偿为负值
  float speed_left_target = ControlState.target_speed_m_s - compensation;

  // 应用补偿后的目标速度
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, speed_right_target);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, speed_left_target);
#endif
}
