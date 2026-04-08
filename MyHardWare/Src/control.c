/**
 * @file control.c
 * @brief 简洁的智能车双环PID控制模块实现
 * @version 2.0
 * @date 2026-04-06
 */

#include "control.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "motor.h"
#include "pid_angle.h"
#include "pid_speed.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ==================== 全局变量 ==================== */
static Control_State_t control_state;

/* ==================== 私有函数 ==================== */
static void update_speed_loop(float angle_comp);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief 初始化控制模块
 */
void Control_Init(void) {
  // 清零状态
  memset(&control_state, 0, sizeof(Control_State_t));
  control_state.mode = CONTROL_MODE_STOP;
  control_state.enabled = 0;

  // 初始化速度环PID
#ifdef QUAD_MOTOR_DRIVE
  Speed_PID_Init(&Speed_PID_FR, ENCODER_FR, MOTOR_FR, SPEED_PID_KP_DEFAULT,
                 SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_FL, ENCODER_FL, MOTOR_FL, SPEED_PID_KP_DEFAULT,
                 SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_BR, ENCODER_BR, MOTOR_BR, SPEED_PID_KP_DEFAULT,
                 SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_BL, ENCODER_BL, MOTOR_BL, SPEED_PID_KP_DEFAULT,
                 SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT);
#else
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);
#endif

  // 初始化角度环PID
  Angle_PID_Init(&Angle_PID_Yaw, ANGLE_AXIS_YAW, ANGLE_PID_KP_DEFAULT,
                 ANGLE_PID_KI_DEFAULT, ANGLE_PID_KD_DEFAULT);
}

/**
 * @brief 启用控制
 */
void Control_Enable(void) {
  control_state.enabled = 1;

  // 启用速度环
#ifdef QUAD_MOTOR_DRIVE
  Speed_PID_Enable(&Speed_PID_FR);
  Speed_PID_Enable(&Speed_PID_FL);
  Speed_PID_Enable(&Speed_PID_BR);
  Speed_PID_Enable(&Speed_PID_BL);
#else
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);
#endif

  // 启用角度环
  Angle_PID_Enable(&Angle_PID_Yaw);
}

/**
 * @brief 禁用控制
 */
void Control_Disable(void) {
  control_state.enabled = 0;

  // 禁用PID
#ifdef QUAD_MOTOR_DRIVE
  Speed_PID_Disable(&Speed_PID_FR);
  Speed_PID_Disable(&Speed_PID_FL);
  Speed_PID_Disable(&Speed_PID_BR);
  Speed_PID_Disable(&Speed_PID_BL);
#else
  Speed_PID_Disable(&Speed_PID_Right);
  Speed_PID_Disable(&Speed_PID_Left);
#endif
  Angle_PID_Disable(&Angle_PID_Yaw);
}

/**
 * @brief 设置速度
 * @param speed 速度 (m/s)
 */
void Control_SetSpeed(float speed) {
  control_state.target_speed = speed;

#ifdef QUAD_MOTOR_DRIVE
  Speed_PID_SetTargetSpeed(&Speed_PID_FR, speed);
  Speed_PID_SetTargetSpeed(&Speed_PID_FL, speed);
  Speed_PID_SetTargetSpeed(&Speed_PID_BR, speed);
  Speed_PID_SetTargetSpeed(&Speed_PID_BL, speed);
#else
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, speed);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, speed);
#endif
}

/**
 * @brief 设置目标角度
 * @param angle 目标角度 (度)
 */
void Control_SetAngle(float angle) {
  control_state.target_angle = angle;
  Angle_PID_SetTargetAngle(&Angle_PID_Yaw, angle);
}

/**
 * @brief 设置控制模式
 * @param mode 控制模式
 */
void Control_SetMode(Control_Mode_e mode) {
  if (control_state.mode != mode) {
    control_state.mode = mode;
    Angle_PID_Reset(&Angle_PID_Yaw); // 切换模式时重置积分
  }
}

/**
 * @brief 控制更新 (10ms周期调用)
 */
void Control_Update(void) {
  if (!control_state.enabled) {
    return;
  }

  // 1. 从MPU6050 DMP获取角度
  float pitch, roll, yaw;
  if (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0) {
    // printf("mpu6050数据读取失败");
    // return; // 读取失败，跳过本次更新
    yaw = control_state.current_angle;
  }
  // printf("target_speed:%f,target_angle:%f,current yaw:%.2f\r\n",
  //        control_state.target_speed, control_state.target_angle, yaw);

  // 滑动平均滤波
  control_state.angle_buffer[control_state.angle_buffer_index] = yaw;
  control_state.angle_buffer_index =
      (control_state.angle_buffer_index + 1) % ANGLE_PID_BUFFER_SIZE;

  // 计算滤波后的角度
  float sum = 0.0f;
  for (int i = 0; i < ANGLE_PID_BUFFER_SIZE; i++) {
    sum += control_state.angle_buffer[i];
  }
  control_state.current_angle = sum / (float)ANGLE_PID_BUFFER_SIZE;
  // 2. 更新当前角度
  // control_state.current_angle = yaw;

  // 3. 执行角度环PID (仅在直线和转弯模式)
  if (control_state.mode != CONTROL_MODE_STOP) {
    control_state.angle_compensation = Angle_PID_Update(&Angle_PID_Yaw, yaw);
  } else {
    control_state.angle_compensation = 0.0f;
  }

  // 4. 应用角度补偿到速度环
  update_speed_loop(control_state.angle_compensation);
  static uint32_t step = 0;
  // if (step % 4 == 0) {
  //   printf("%f,%f,%f,%f,%f\r\n", control_state.target_angle,
  //          control_state.current_angle, Angle_PID_Yaw.pid_state.kp,
  //          Angle_PID_Yaw.pid_state.ki, Angle_PID_Yaw.pid_state.kd);
  // }
  //=============================================================================
  // NOTE: 速度环的更新依赖于encoder的更新，encoder
  // 2ms更新周期，速度环更新周期为5ms，将这几个更新任务放再main函数中
  //=============================================================================

  //   // 5. 执行速度环PID
  // #ifdef QUAD_MOTOR_DRIVE
  //   Speed_PID_Update(&Speed_PID_FR);
  //   Speed_PID_Update(&Speed_PID_FL);
  //   Speed_PID_Update(&Speed_PID_BR);
  //   Speed_PID_Update(&Speed_PID_BL);
  // #else
  //   Speed_PID_Update(&Speed_PID_Right);
  //   Speed_PID_Update(&Speed_PID_Left);
  // #endif
}

/**
 * @brief 停止运动
 */
void Control_Stop(void) {
  Control_Disable();
  Motor_StopAll();
  control_state.mode = CONTROL_MODE_STOP;
  control_state.target_speed = 0.0f;
  control_state.target_angle = 0.0f;
  control_state.angle_compensation = 0.0f;
}

/**
 * @brief 获取控制状态
 * @return 控制状态指针
 */
Control_State_t *Control_GetState(void) { return &control_state; }

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 更新速度环（应用角度补偿）
 * @param angle_comp 角度补偿值
 */
static void update_speed_loop(float angle_comp) {
  if (fabsf(angle_comp) < 0.01f) {
    Speed_PID_SetTargetSpeed(&Speed_PID_Left, control_state.target_speed);
    Speed_PID_SetTargetSpeed(&Speed_PID_Right, control_state.target_speed);
    return; // 补偿值太小，忽略
  }

#ifdef QUAD_MOTOR_DRIVE
  // 四驱：右侧加速，左侧减速
  Speed_PID_SetTargetSpeed(&Speed_PID_FR,
                           control_state.target_speed + angle_comp);
  Speed_PID_SetTargetSpeed(&Speed_PID_FL,
                           control_state.target_speed - angle_comp);
  Speed_PID_SetTargetSpeed(&Speed_PID_BR,
                           control_state.target_speed + angle_comp);
  Speed_PID_SetTargetSpeed(&Speed_PID_BL,
                           control_state.target_speed - angle_comp);
#else
  // 双驱：右侧加速，左侧减速
  Speed_PID_SetTargetSpeed(&Speed_PID_Right,
                           control_state.target_speed + angle_comp);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left,
                           control_state.target_speed - angle_comp);
#endif
}
