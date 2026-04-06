#include "delay.h"
#include "mpu6050.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "usart.h"

/**
 * @brief  初始化MPU6050
 * @note   此函数初始化MPU6050传感器
 * @retval 初始化结果: 0表示成功，非0表示失败
 */
u8 MPU6050_Init(void) {
  u8 res = 0;

  // 初始化MPU6050
  res = MPU_Init();
  if (res != 0) {
    printf("MPU6050初始化失败，错误码：%d\r\n", res);
    return res;
  }

  printf("MPU6050初始化成功！\r\n");
  return 0;
}

/**
 * @brief  读取并打印MPU6050原始数据
 * @note   此函数读取MPU6050的原始数据并打印
 * @retval 读取结果: 0表示成功，非0表示失败
 */
u8 MPU6050_ReadRawData(void) {
  u8 res = 0;

  // 读取原始数据
  res = MPU_Updata();
  if (res != 0) {
    printf("读取MPU6050原始数据失败，错误码：%d\r\n", res);
    return res;
  }

  // 打印原始数据
  printf("加速度: %d, %d, %d | ", raw_data[0], raw_data[1], raw_data[2]);
  printf("陀螺仪: %d, %d, %d\r\n", raw_data[3], raw_data[4], raw_data[5]);

  return 0;
}

/**
 * @brief  读取并打印MPU6050姿态数据
 * @note   此函数读取MPU6050的姿态数据并打印
 * @retval 读取结果: 0表示成功，非0表示失败
 */
u8 MPU6050_ReadAttitude(void) {
  u8 res = 0;

  // 更新MPU6050数据
  res = MPU_Updata();
  if (res != 0) {
    printf("更新MPU6050数据失败，错误码：%d\r\n", res);
    return res;
  }

  // 计算姿态角
  MPU_Proc();

  // 打印姿态数据
  printf("%.2f ,%.2f ,%.2f\r\n", MPU_Attitude.pitch, MPU_Attitude.roll,
         MPU_Attitude.yaw);

  return 0;
}

/**
 * @brief  主函数
 * @note   演示MPU6050的完整使用流程
 * @retval None
 */
int main(void) {
  u8 res = 0;

  // 系统初始化
  delay_init();  // 初始化延时函数
  USART1_Init(); // 初始化串口1，用于打印调试信息

  // 初始化MPU6050
  printf("正在初始化MPU6050...\r\n");
  res = MPU6050_Init();
  if (res != 0) {
    printf("MPU6050初始化失败！程序终止。\r\n");
    while (1) {
      delay_ms(1000);
    }
  }

  // 打印表头
  printf("开始读取MPU6050数据...\r\n");
  printf("格式: 加速度 | 陀螺仪 | 姿态角\r\n");
  printf("------------------------------------------------\r\n");

  // 主循环
  while (1) {
    // 读取并打印MPU6050原始数据
    // res = MPU6050_ReadRawData();

    // 读取并打印MPU6050姿态数据
    res = MPU6050_ReadAttitude();

    // 延时
    delay_ms(5);
  }
}
