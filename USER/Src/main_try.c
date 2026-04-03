#include "delay.h"
#include "hmc5883l.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "encoder.h" // 编码器模块
#include "led.h"
#include "motor.h"
#include "pid_speed.h" // 添加速度环PID控制器头文件
#include "rtc.h"
#include "task.h" // 任务调度模块
#include "ulog.h"
#include <math.h>



typedef struct{
    float pitch;
    float roll;
    float yaw;
    float target_yaw;
    float use_target_yaw;
    // float frist_pitch;
    // float frist_roll;
    float first_yaw;
}Attitude_typedef;

Attitude_typedef attitude;
/**
 * @brief  初始化MPU6050 DMP
 * @note   此函数初始化MPU6050的DMP功能
 * @retval 初始化结果: 0表示成功，非0表示失败
 */
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

/**
 * @brief  读取并打印MPU6050 DMP姿态数据
 * @note   此函数读取MPU6050 DMP的姿态数据并打印
 * @retval 读取结果: 0表示成功，非0表示失败
 */
float MPU6050_DMP_ReadAttitude(void) {
  u8 res = 0;
  float pitch, roll, yaw;

  // 使用mpu_dmp_get_data函数获取姿态数据
  res = mpu_dmp_get_data(&pitch, &roll, &yaw);
  if (res != 0) {
    printf("读取MPU6050 DMP数据失败，错误码：%d\r\n", res);
    return res;
  }
  yaw = HMC5883L_Get_Azimuth(pitch * M_PI / 180.0f, roll * M_PI / 180.0f);

  // 打印姿态数据
//   printf("%.2f,%.2f,%.2f\r\n", pitch, roll, yaw);
  // printf("俯仰角: %.2f | 横滚角: %.2f | 偏航角: %.2f\r\n", pitch, roll, yaw);
  return yaw;
}

void speed_control_task(void) {
  // 更新右轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);

  Motor_Update(MOTOR_LEFT);
  Motor_Update(MOTOR_RIGHT);
  // printf("finlish//\r\n");
}

void Attitude_Typedef_Init(Attitude_typedef *attitude) {
  float res = MPU6050_DMP_ReadAttitude();
  attitude->first_yaw = res;
}

void Attitude_Pid_Task(Attitude_typedef *attitude) {
  
}




/**
 * @brief  主函数
 * @note   演示MPU6050 DMP的完整使用流程
 * @retval None
 */
int main(void) {
  // 系统初始化

  delay_init();
  USART1_Init();
  led_Init();
  LED_Off();
  Motor_Driver_Init();
  Encoder_Driver_Init();
  Task_Init();


  // 初始化MPU6050 DMP
  u8 res = 0;
  res = MPU6050_DMP_Init();
  hmc5883l_init();
  if (res != 0) {
    printf("MPU6050 DMP初始化失败！程序终止。\r\n");
    while (1) {
      delay_ms(1000);
    }
  }

  hmc5883l_single_measurement();
  delay_ms(10);


  //PID控制器初始化
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.0f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.0f);

  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);


  //添加任务
  add_task(speed_control_task, SPEED_PID_SAMPLE_PERIOD_MS);
  add_task(Encoder_Update, 1);

  // 主循环
  while (1) {
    // 读取并打印MPU6050 DMP姿态数据
    // res = MPU6050_DMP_ReadAttitude();
    // 延时
    delay_ms(10);
    hmc5883l_single_measurement();
  }
}


