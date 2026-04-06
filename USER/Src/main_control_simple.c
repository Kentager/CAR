/**
 * @file main_control_simple.c
 * @brief 简化的控制模块使用示例
 * @version 1.0
 * @date 2026-04-06
 *
 * @details 演示如何使用简洁的控制模块API
 */

#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "inv_mpu.h"
#include "usart.h"
#include "task.h"
#include <stdio.h>

/**
 * @brief 初始化MPU6050 DMP
 * @return 0=成功, 非0=失败
 */
static uint8_t init_mpu6050(void) {
  printf("初始化MPU6050 DMP...\r\n");
  if (mpu_dmp_init() != 0) {
    printf("MPU6050 DMP初始化失败\r\n");
    return 1;
  }
  printf("MPU6050 DMP初始化成功\r\n");
  return 0;
}

/**
 * @brief 主函数
 */
int main(void) {
  // 系统初始化
  delay_init();
  USART1_Init();

  // 硬件初始化
  Motor_Driver_Init();
  Encoder_Driver_Init();

  // 初始化MPU6050
  if (init_mpu6050() != 0) {
    printf("系统初始化失败\r\n");
    while (1) {
      delay_ms(1000);
    }
  }

  // 初始化控制模块
  printf("初始化控制模块...\r\n");
  Control_Init();
  printf("控制模块初始化完成\r\n");

  // 初始化任务系统
  Task_Init();

  // 添加10ms周期的控制更新任务
  add_task(Control_Update, 10);

  printf("\r\n========== 简易控制示例 ==========\r\n");
  printf("使用说明:\r\n");
  printf("1. 直线前进: Control_SetSpeed(0.5); Control_SetMode(CONTROL_MODE_STRAIGHT);\r\n");
  printf("2. 右转90度: Control_SetAngle(90.0); Control_SetMode(CONTROL_MODE_TURN);\r\n");
  printf("3. 左转45度: Control_SetAngle(-45.0); Control_SetMode(CONTROL_MODE_TURN);\r\n");
  printf("4. 停止: Control_Stop();\r\n");
  printf("====================================\r\n\r\n");

  // 示例1: 直线前进
  printf("示例1: 直线前进 0.5 m/s\r\n");
  Control_SetSpeed(0.5f);
  Control_SetAngle(0.0f);
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_Enable();

  delay_ms(5000); // 前进5秒
  Control_Stop();
  printf("直线前进完成\r\n\r\n");
  delay_ms(1000);

  // 示例2: 右转90度
  printf("示例2: 右转90度\r\n");
  Control_SetSpeed(0.2f);
  Control_SetAngle(90.0f);
  Control_SetMode(CONTROL_MODE_TURN);
  Control_Enable();

  delay_ms(5000); // 转弯5秒
  Control_Stop();
  printf("右转完成\r\n\r\n");
  delay_ms(1000);

  // 示例3: 左转45度
  printf("示例3: 左转45度\r\n");
  Control_SetSpeed(0.2f);
  Control_SetAngle(-45.0f);
  Control_SetMode(CONTROL_MODE_TURN);
  Control_Enable();

  delay_ms(3000); // 转弯3秒
  Control_Stop();
  printf("左转完成\r\n\r\n");
  delay_ms(1000);

  // 示例4: 直线后退
  printf("示例4: 直线后退 0.3 m/s\r\n");
  Control_SetSpeed(-0.3f);
  Control_SetAngle(0.0f);
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_Enable();

  delay_ms(3000); // 后退3秒
  Control_Stop();
  printf("后退完成\r\n\r\n");
  delay_ms(1000);

  // 示例5: 监控模式（打印状态）
  printf("示例5: 状态监控模式\r\n");
  printf("每秒打印一次控制状态...\r\n");
  Control_SetSpeed(0.4f);
  Control_SetAngle(0.0f);
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_Enable();

  uint32_t last_print = 0;
  while (1) {
    Task_Scheduler();

    // 每秒打印一次状态
    if (GetSysTick() - last_print >= 1000) {
      last_print = GetSysTick();
      Control_State_t *state = Control_GetState();

      printf("状态: 模式=%d, 速度=%.2f m/s, 角度=%.2f°, 目标=%.2f°, 补偿=%.3f\r\n",
             state->mode,
             state->target_speed,
             state->current_angle,
             state->target_angle,
             state->angle_compensation);
    }
  }
}
