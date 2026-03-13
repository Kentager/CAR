#include "delay.h"
#include "encoder.h" // 编码器模块
#include "led.h"
#include "motor.h"
#include "pid_speed.h" // 添加速度环PID控制器头文件
#include "rtc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "task.h" // 任务调度模块
#include "ulog.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>

// 删除外部声明，让pid_speed模块内部管理实例
// Deleted:extern Speed_PID_Controller_t Speed_PID_Right; // 右轮速度PID控制器
// Deleted:extern Speed_PID_Controller_t Speed_PID_Left;  // 左轮速度PID控制器

void my_console_logger(ulog_level_t severity, const char *msg) {
  printf("%s [%s]: %s",
         RTC_GetNowTime(), // user defined function
         ulog_level_name(severity), msg);
}

// 10ms周期的速度环PID控制任务
void speed_control_task(void) {
  // 更新右轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);
}

void motor_test(void) {
  Motor_SetSpeed(MOTOR_LEFT, 4000);  // 左电机正转，速度 4000
  Motor_SetSpeed(MOTOR_RIGHT, 4000); // 右电机正转，速度 4000
  Motor_SetDirection(MOTOR_LEFT, MOTOR_DIR_FORWARD);
  Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_BACKWARD);

  // 更新电机状态，将配置应用到硬件
  Motor_Update(MOTOR_LEFT);
  Motor_Update(MOTOR_RIGHT);
}

Encoder_Data_t encoder_data_LEFT;
Encoder_Data_t encoder_data_RIGHT;
void encoder_upgrade(void) {
  encoder_data_LEFT = Encoder_GetData(ENCODER_LEFT);
  encoder_data_RIGHT = Encoder_GetData(ENCODER_RIGHT);
}
void print_encoderData(void) {
  // 调试信息：打印详细的编码器数据
  printf("=== LEFT ENCODER DEBUG ===\r\n");
  printf("tim:%d Left Encoder - Count: %d, speed:%.2f m/s X: %.2f m\r\n",
         GetSysTick(), encoder_data_LEFT.total_count,
         encoder_data_LEFT.speed_m_s, encoder_data_LEFT.total_distance);

  printf("=== RIGHT ENCODER DEBUG ===\r\n");
  printf("tim:%d Right Encoder - Count: %d, speed:%.2f m/s X: %.2f m\r\n",
         GetSysTick(), encoder_data_RIGHT.total_count,
         encoder_data_RIGHT.speed_m_s, encoder_data_RIGHT.total_distance);

  printf("\r\n");
}
int main() {
  int arg = 42;
  led_Init();
  LED_Off();
  Motor_Driver_Init();
  // motor_test();

  // 初始化编码器模块
  Encoder_Driver_Init();

  // 初始化任务调度系统
  Task_Init();
  /*

    // 初始化速度环PID控制器
    // 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
    Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                   SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                   SPEED_PID_KD_DEFAULT);

    // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
    Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                   SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                   SPEED_PID_KD_DEFAULT);

    // 设置目标速度（例如0.5m/s）
    Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.5f);
    Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.5f);

    // 启用速度环PID控制
    Speed_PID_Enable(&Speed_PID_Right);
    Speed_PID_Enable(&Speed_PID_Left);

    // 添加10ms周期的速度控制任务
    add_task(speed_control_task, SPEED_PID_SAMPLE_PERIOD_MS);
    */

  USART1_Init();
  printf("hello world\r\n");

  delay_init();
  Encoder_Data_t encoder_data;

  // 添加任务
  add_task(print_encoderData, 200); // 每200ms执行一次
  add_task(encoder_upgrade, 5);     // 每5ms执行一次

  // 启动任务（将任务标记为就绪态）
  start_task(0); // print_encoderData
  start_task(1); // encoder_upgrade

  while (1) {
    Task_Scheduler(); // 调度器执行就绪态任务
  }
}
