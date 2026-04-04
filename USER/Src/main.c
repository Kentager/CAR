#include "delay.h"
#include "encoder.h" // 编码器模块
#include "led.h"
#include "motor.h"
#include "pid_speed.h" // 添加速度环PID控制器头文件
#include "rtc.h"
#include "task.h" // 任务调度模块
#include "ulog.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stm32f4xx.h>

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


  Motor_Update(MOTOR_LEFT);
  Motor_Update(MOTOR_RIGHT);
  // printf("finlish//\r\n");
  // printf("finlish//\r\n");
}

void motor_test(void) {
  Motor_SetSpeed(MOTOR_LEFT, 4000);  // 左电机正转，速度 4000
  Motor_SetSpeed(MOTOR_RIGHT, 4000); // 右电机正转，速度 4000
  Motor_SetDirection(MOTOR_LEFT, MOTOR_DIR_FORWARD);
  Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);
  // Motor_Update(MOTOR_RIGHT);
  // Motor_Update(MOTOR_LEFT);
  Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);
  // Motor_Update(MOTOR_RIGHT);
  // Motor_Update(MOTOR_LEFT);
  // 更新电机状态，将配置应用到硬件
}
void motor_print_data(void) {
    // Encoder_Data_t encoder_data;
    // encoder_data = Encoder_GetData(ENCODER_LEFT);
    printf("%.5f,%.5f,%.5f\n", Speed_PID_Right.target_speed_m_s, Speed_PID_Right.current_speed_m_s,Speed_PID_Left.current_speed_m_s);
    // printf("Left Encoder - Count: %d, speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count, encoder_data.speed_m_s,encoder_data.total_distance);
    // encoder_data = Encoder_GetData(ENCODER_RIGHT);
    // printf("DATA,%.2f,%.2f\n", Speed_PID_Left.current_speed_m_s, encoder_data.speed_m_s);
    // printf("Right Encoder - Count: %d, speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count, encoder_data.speed_m_s,encoder_data.total_distance);
    // printf("\r\n");
}
void change_pid_target_speed(void) {
  uint32_t count = GetSysTick();
  if (count % 2000 == 0) {
    if (Speed_PID_Right.target_speed_m_s == 0.1f) {
      Speed_PID_SetTargetSpeed(&Speed_PID_Right, -0.1f);
      Speed_PID_SetTargetSpeed(&Speed_PID_Left, -0.087f);
    }
    else if(Speed_PID_Right.target_speed_m_s == -0.1f) {
      Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.1f);
      Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.087f);
    }
  }
  
}
void motor_print_data(void) {
    // Encoder_Data_t encoder_data;
    // encoder_data = Encoder_GetData(ENCODER_LEFT);
    printf("%.5f,%.5f,%.5f\n", Speed_PID_Right.target_speed_m_s, Speed_PID_Right.current_speed_m_s,Speed_PID_Left.current_speed_m_s);
    // printf("Left Encoder - Count: %d, speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count, encoder_data.speed_m_s,encoder_data.total_distance);
    // encoder_data = Encoder_GetData(ENCODER_RIGHT);
    // printf("DATA,%.2f,%.2f\n", Speed_PID_Left.current_speed_m_s, encoder_data.speed_m_s);
    // printf("Right Encoder - Count: %d, speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count, encoder_data.speed_m_s,encoder_data.total_distance);
    // printf("\r\n");
}
void change_pid_target_speed(void) {
  uint32_t count = GetSysTick();
  if (count % 2000 == 0) {
    if (Speed_PID_Right.target_speed_m_s == 0.1f) {
      Speed_PID_SetTargetSpeed(&Speed_PID_Right, -0.1f);
      Speed_PID_SetTargetSpeed(&Speed_PID_Left, -0.087f);
    }
    else if(Speed_PID_Right.target_speed_m_s == -0.1f) {
      Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.1f);
      Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.087f);
    }
  }
  
}
int main() {


  led_Init();
  LED_Off();
  Motor_Driver_Init();
  // motor_test();

  // 初始化编码器模块
  Encoder_Driver_Init();

  // // 初始化任务调度系统
  // // 初始化任务调度系统
  Task_Init();

  // // 初始化速度环PID控制器
  // // 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
  // // 初始化速度环PID控制器
  // // 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 设置目标速度（例如0.5m/s）
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.05f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.1f);

  // // 启用速度环PID控制
  // // 启用速度环PID控制
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);

  // // 添加10ms周期的速度控制任务
  add_task(speed_control_task, SPEED_PID_SAMPLE_PERIOD_MS);
  add_task(Encoder_Update, 1);
  add_task(motor_print_data, 30);
  // add_task(change_pid_target_speed, 1);
  add_task(speed_control_task, SPEED_PID_SAMPLE_PERIOD_MS);
  add_task(Encoder_Update, 1);
  add_task(motor_print_data, 30);
  // add_task(change_pid_target_speed, 1);
  USART1_Init();
  // printf("hello world\r\n");
  // printf("hello world\r\n");

  delay_init();

  uint32_t count = 0;

  uint32_t count = 0;
  // motor_test();
  // Motor_SetSpeed(MOTOR_LEFT, 4000); // 右电机正转，速度 4000
  // Motor_Update( MOTOR_LEFT);
  // Motor_SetSpeed(MOTOR_LEFT, 4000); // 右电机正转，速度 4000
  // Motor_Update( MOTOR_LEFT);
  // 更新电机状态，将配置应用到硬件
  while (1) {

    

    
    // speed_control_task();
    // GPIO_ToggleBits(GPIOC, 13);
    Task_Scheduler();
    // speed_control_task();
    // Motor_SetSpeed(MOTOR_RIGHT, 4000); // 右电机正转，速度 4000
    // Motor_Update( MOTOR_RIGHT);
    // printf("in task//");
    // delay_ms(100);
    Task_Scheduler();
    // speed_control_task();
    // Motor_SetSpeed(MOTOR_RIGHT, 4000); // 右电机正转，速度 4000
    // Motor_Update( MOTOR_RIGHT);
    // printf("in task//");
    // delay_ms(100);
    // printf("this is %d:\r\n", count);
    // Encoder_Update();
    // Encoder_Update();
    // encoder_data = Encoder_GetData(ENCODER_LEFT);
    // printf("Left Encoder - Count: %d, speed_m_s:%.2f, total_distance:
    // %.2fm\r\n",encoder_data.last_count,
    // encoder_data.speed_m_s,encoder_data.total_distance); encoder_data =
    // Encoder_GetData(ENCODER_RIGHT); printf("Right Encoder - Count: %d,
    // speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count,
    // encoder_data.speed_m_s,encoder_data.total_distance);
    //  printf("\r\n");
    // printf("Left Encoder - Count: %d, speed_m_s:%.2f, total_distance:
    // %.2fm\r\n",encoder_data.last_count,
    // encoder_data.speed_m_s,encoder_data.total_distance); encoder_data =
    // Encoder_GetData(ENCODER_RIGHT); printf("Right Encoder - Count: %d,
    // speed_m_s:%.2f, total_distance: %.2fm\r\n",encoder_data.last_count,
    // encoder_data.speed_m_s,encoder_data.total_distance);
    //  printf("\r\n");
    // count++;


  }
}
