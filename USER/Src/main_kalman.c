/**
 * @file kalman_filter_test.c
 * @brief 卡尔曼滤波姿态解算测试代码
 * @version 1.0
 * @date 2026-03-13
 */

#include "kalman_filter.h"
#include "delay.h"
#include "task.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx.h"

/**
 * @brief 卡尔曼滤波器单元测试
 * @note 测试单个卡尔曼滤波器的功能
 */
void KalmanFilter_UnitTest(void) {
  KalmanFilter_t kf;
  float filtered_value;
  
  printf("\r\n========== 卡尔曼滤波器单元测试 ==========\r\n");
  
  // 初始化卡尔曼滤波器
  KalmanFilter_Init(&kf, 0.01f, 0.1f, 0.01f, 0.0f);
  printf("卡尔曼滤波器初始化完成\r\n");
  
  // 模拟数据输入并观察滤波效果
  printf("\r\n模拟数据输入测试：\r\n");
  printf("测量值\t滤波值\t\t差值\r\n");
  
  for (int i = 0; i < 20; i++) {
    float measurement = 10.0f + (float)(rand() % 100) / 10.0f; // 10.0-20.0的随机值
    float control_input = 0.0f;
    
    filtered_value = KalmanFilter_Update(&kf, measurement, control_input);
    printf("%.2f\t%.2f\t\t%.2f\r\n", measurement, filtered_value, measurement - filtered_value);
  }
  
  // 测试重置功能
  printf("\r\n测试卡尔曼滤波器重置功能...\r\n");
  KalmanFilter_Reset(&kf, 0.0f);
  printf("卡尔曼滤波器已重置\r\n");
  
  printf("========== 卡尔曼滤波器单元测试完成 ==========\r\n\r\n");
}

/**
 * @brief 姿态解算系统测试
 * @note 测试姿态解算系统的完整功能
 */
void AttitudeSolver_SystemTest(void) {
  EulerAngle_Param *raw_angle;
  EulerAngle_Param *filtered_angle;
  Acc_Param *acc_data;
  Gyro_Param *gyro_data;
  
  printf("\r\n========== 姿态解算系统测试 ==========\r\n");
  
  // 初始化姿态解算系统
  printf("初始化姿态解算系统...\r\n");
  AttitudeSolver_Init();
  printf("姿态解算系统初始化完成\r\n");
  
  // 等待系统稳定
  printf("等待系统稳定...\r\n");
  delay_ms(1000);
  
  // 连续读取并显示姿态数据
  printf("\r\n开始读取姿态数据（每100ms更新一次）：\r\n");
  printf("俯仰角(原始/滤波)\t横滚角(原始/滤波)\t偏航角(原始/滤波)\r\n");
  
  for (int i = 0; i < 50; i++) {
    // 更新姿态解算
    AttitudeSolver_Update();
    
    // 获取数据
    raw_angle = AttitudeSolver_GetRawAngle();
    filtered_angle = AttitudeSolver_GetFilteredAngle();
    acc_data = AttitudeSolver_GetAcc();
    gyro_data = AttitudeSolver_GetGyro();
    
    // 打印数据
    printf("%.2f/%.2f\t\t%.2f/%.2f\t\t%.2f/%.2f\r\n", 
           raw_angle->Pitch, filtered_angle->Pitch,
           raw_angle->Roll, filtered_angle->Roll,
           raw_angle->Yaw, filtered_angle->Yaw);
    
    // 延时
    delay_ms(100);
  }
  
  printf("========== 姿态解算系统测试完成 ==========\r\n\r\n");
}

/**
 * @brief 姿态解算系统性能测试
 * @note 测试姿态解算系统的响应速度和稳定性
 */
void AttitudeSolver_PerformanceTest(void) {
  EulerAngle_Param *filtered_angle;
  uint32_t start_time, end_time;
  float update_time;
  
  printf("\r\n========== 姿态解算系统性能测试 ==========\r\n");
  
  // 初始化姿态解算系统
  AttitudeSolver_Init();
  
  // 等待系统稳定
  delay_ms(1000);
  
  // 测试更新时间
  printf("测试姿态解算更新时间...\r\n");
  start_time = GetSysTick();
  for (int i = 0; i < 100; i++) {
    AttitudeSolver_Update();
  }
  end_time = GetSysTick();
  update_time = (float)(end_time - start_time) / 100.0f;
  printf("平均更新时间: %.3f ms\r\n", update_time);
  
  // 测试稳定性
  printf("\r\n测试姿态解算稳定性（静止状态）：\r\n");
  printf("俯仰角\t横滚角\t偏航角\r\n");
  
  for (int i = 0; i < 20; i++) {
    AttitudeSolver_Update();
    filtered_angle = AttitudeSolver_GetFilteredAngle();
    
    printf("%.2f\t%.2f\t%.2f\r\n", 
           filtered_angle->Pitch,
           filtered_angle->Roll,
           filtered_angle->Yaw);
    
    delay_ms(100);
  }
  
  printf("========== 姿态解算系统性能测试完成 ==========\r\n\r\n");
}

/**
 * @brief 主函数
 * @note 执行所有测试
 */
int main(void) {
  // 系统初始化
  delay_init();  // 初始化延时函数
  USART1_Init(); // 初始化串口1，用于打印调试信息
  
  // 打印测试开始信息
  printf("\r\n========================================\r\n");
  printf("    卡尔曼滤波姿态解算系统测试程序\r\n");
  printf("========================================\r\n");
  
  // 执行卡尔曼滤波器单元测试
  KalmanFilter_UnitTest();
  
  // 执行姿态解算系统测试
  AttitudeSolver_SystemTest();
  
  // 执行姿态解算系统性能测试
  AttitudeSolver_PerformanceTest();
  
  // 持续运行姿态解算并显示数据
  printf("\r\n========== 持续运行姿态解算系统 ==========\r\n");
  printf("俯仰角\t横滚角\t偏航角\r\n");
  
  while (1) {
    // 更新姿态解算
    AttitudeSolver_Update();
    
    // 获取并显示滤波后的姿态角
    EulerAngle_Param *filtered_angle = AttitudeSolver_GetFilteredAngle();
    printf("%.2f\t%.2f\t%.2f\r\n", 
           filtered_angle->Pitch,
           filtered_angle->Roll,
           filtered_angle->Yaw);
    
    // 延时
    delay_ms(100);
  }
}
