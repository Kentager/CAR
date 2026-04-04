#include "delay.h"
#include "hmc5883l.h"
#include "stm32f4xx.h"
#include "task.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
// #include "inv_mpu.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_speed.h"
HMC5883L_Data_t data;
void get_data(void) {
  hmc5883l_read_data(&data);
  // printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d,\r\n",
  // raw_data[0],raw_data[1],raw_data[2],raw_data[3],raw_data[4],raw_data[5],data.raw_data[0],
  // data.raw_data[1], data.raw_data[2]);
  printf("mx:%.5f,my:%.5f,mz:%.5f \r\n", data.x, data.y, data.z);
  hmc5883l_single_measurement();
  // MPU_Updata();
}
void speed_control_task(void) {
  // 更新右轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);
}
void encoder_print_data(void) {
  Encoder_Data_t encoder_data;
  encoder_data = Encoder_GetData(ENCODER_LEFT);
  printf("Left speed_m_s:%.2f, total_distance: %.2fm       ",
         encoder_data.speed_m_s, encoder_data.total_distance);
  encoder_data = Encoder_GetData(ENCODER_RIGHT);
  printf("Right speed_m_s:%.2f, total_distance: %.2fm\r\n",
         encoder_data.speed_m_s, encoder_data.total_distance);
}
int main(void) {
  delay_init();
  USART1_Init();
  printf("hmc5883l calibrate\r\n");
  hmc5883l_init();
  hmc5883l_read_id_info();
  // // 设置传感器增益（可选）
  set_sensor_gain(HMC5883L_GAIN_390);
  // mpu_dmp_init();
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
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.2f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.2f);
  // // 启用速度环PID控制
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);

  // Speed_PID_Update(&Speed_PID_Right);

  // // 更新左轮速度环PID控制器
  // Speed_PID_Update(&Speed_PID_Left);
  // for (uint8_t i = 1; i <= 3; i++) {
  //   printf("it's turn %d\r\n", i);
  //   delay_ms(1000);
  // }

  // MPU_Updata();
  // hmc5883l_calibrate(1000);
  hmc5883l_single_measurement();
  delay_ms(10);
  get_data();
  Task_Init();
  // add_task(get_data, 10);
  add_task(Encoder_Update, 5);
  add_task(speed_control_task, 5);
  add_task(encoder_print_data, 5);

  while (1) {
    Task_Scheduler();
  }
}
