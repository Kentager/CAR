#include "delay.h"
#include "encoder.h" // 编码器模块
#include "led.h"
#include "motor.h"
#include "pid_speed.h" // 添加速度环PID控制器头文件
#include "pid_angle.h"
#include "rtc.h"
#include "task.h" // 任务调度模块
#include "ulog.h"
#include "usart.h"
#include <math.h>
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "hmc5883l.h"
#include <stdio.h>
#include <stm32f4xx.h>

// 删除外部声明，让pid_speed模块内部管理实例
// Deleted:extern Speed_PID_Controller_t Speed_PID_Right; // 右轮速度PID控制器
// Deleted:extern Speed_PID_Controller_t Speed_PID_Left;  // 左轮速度PID控制器

float yaw_angle = 0;
u8 MPU6050_DMP_Init(void) {
  u8 res = 0;

  printf("正在初始化MPU6050 DMP...\r\n");

  // 初始化DMP
  res = mpu_dmp_init();
  if (res != 0) {
    printf("MPU6050 DMP初始化失败，错误码：%d\r\n", res);
    return res;
  }

  printf("MPU6050 DMP初始化成功！\r\n");
  return 0;
}

u8 MPU6050_DMP_ReadAttitude(void) {
  u8 res = 0;
  float pitch, roll, yaw;
  // 使用mpu_dmp_get_data函数获取姿态数据
  res = mpu_dmp_get_data(&pitch, &roll, &yaw);
  if (res != 0) {
    printf("读取MPU6050 DMP数据失败，错误码：%d\r\n", res);
    return res;
  }
  yaw = HMC5883L_Get_Azimuth(pitch * M_PI / 180.0f, roll * M_PI / 180.0f);
  printf("%.2f,%.2f,%.2f\r\n", pitch, roll, yaw);
  yaw_angle = yaw;
  return 0;
}

void mpu6050_dmp_task(void) {
  MPU6050_DMP_ReadAttitude();
  hmc5883l_single_measurement();
}
// 10ms周期的速度环PID控制任务
void angle_control_task(void) {
  // //获取角度环的目标差速
  float angle_deviation_speed_m_s = Angle_PID_Update(&Angle_PID_Yaw,yaw_angle);
  // //更新角度换的目标差速到速度环PID控制器
  Speed_PID_Angle_Deviation_Speed_Change(&Speed_PID_Right, angle_deviation_speed_m_s);
  Speed_PID_Angle_Deviation_Speed_Change(&Speed_PID_Left, angle_deviation_speed_m_s);
  // //更新速度环PID控制器
  Speed_PID_Update(&Speed_PID_Right);
  Speed_PID_Update(&Speed_PID_Left);
  // //更新电机控制
  Motor_Update(MOTOR_RIGHT);
  Motor_Update(MOTOR_LEFT);
}


int main() {

  led_Init();
  LED_Off();
  Motor_Driver_Init();
  delay_init();
  // 初始化编码器模块
  Encoder_Driver_Init();
  USART1_Init();
  // // 初始化任务调度系统
  Task_Init();



  // // 初始化速度环PID控制器
  // // 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 初始化角度环PID控制器
  Angle_PID_Init(&Angle_PID_Yaw, ANGLE_AXIS_YAW,
                 ANGLE_PID_KP_DEFAULT,ANGLE_PID_KI_DEFAULT , ANGLE_PID_KD_DEFAULT);

  // // 启用速度环PID控制
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);
  // // 启用角度环PID控制
  Angle_PID_Enable(&Angle_PID_Yaw);



  // 设置目标角度为0度（直线行驶）
  Angle_PID_SetTargetAngle(&Angle_PID_Yaw, 0);

  // 设置基础目标速度（根据实际情况调整）
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.5f);  // 右轮目标速度 0.5 m/s
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.5f);   // 左轮目标速度 0.5 m/s

  // 启用角度偏差控制
  Speed_PID_Deviation_Change(&Speed_PID_Right, 1);
  Speed_PID_Deviation_Change(&Speed_PID_Left, 1);

      
  // //初始化MPU6050和HMC5883L
  u8 res;
  res = MPU6050_DMP_Init();
  hmc5883l_init();
  if (res != 0) {
    printf("MPU6050 DMP初始化失败！程序终止。\r\n");
  }

  // //打印表头
  printf("开始读取MPU6050 DMP数据...\r\n");
  printf("格式: 俯仰角 | 横滚角 | 偏航角\r\n");
  printf("------------------------------------------------\r\n");
  hmc5883l_single_measurement();
  delay_ms(10);

  // // 添加周期控制任务
  add_task(angle_control_task, ANGLE_PID_SAMPLE_PERIOD_MS);
  add_task(Encoder_Update, 1);
  add_task(mpu6050_dmp_task, 10);

  while (1) {
    Task_Scheduler();
  }
}
