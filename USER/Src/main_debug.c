#include "delay.h"
#include "hmc5883l.h"
#include "inv_mpu.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>

// #include "inv_mpu.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_speed.h"

int main(void) {
  delay_init();
  USART1_Init();
  printf("hmc5883l calibrate\r\n");
  hmc5883l_init();
  mpu_dmp_init();
  // MPU_Init();
  Motor_Driver_Init();
  Encoder_Driver_Init();
  // // 初始化速度环PID控制器
  // // 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.05f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.1f);
  // // 启用速度环PID控制
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);

  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);
  for (uint8_t i = 1; i <= 3; i++) {
    printf("it's turn %d\r\n", i);
    delay_ms(1000);
  }
  HMC5883L_Data_t data;
  MPU_Updata();
  // hmc5883l_calibrate(1000);
  hmc5883l_single_measurement();
  delay_ms(10);
  while (1) {
    hmc5883l_read_data(&data);
    mpu_dmp_get_data(float *pitch, float *roll, float *yaw);
    // printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d,\r\n",
    // raw_data[0],raw_data[1],raw_data[2],raw_data[3],raw_data[4],raw_data[5],data.raw_data[0],
    // data.raw_data[1], data.raw_data[2]);
    printf("mx:%.5f,my:%.5f,mz:%.5f \r\n", data.x, data.y, data.z);
    hmc5883l_single_measurement();
    MPU_Updata();
    Speed_PID_Update(&Speed_PID_Right);
    Speed_PID_Update(&Speed_PID_Left);
    delay_ms(10);
  }
}