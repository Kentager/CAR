#include "delay.h"
#include "hmc5883l.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "usart.h"

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

  // 打印姿态数据
  printf("俯仰角: %.2f | 横滚角: %.2f | 偏航角: %.2f\r\n", pitch, roll, yaw);

  return 0;
}

/**
 * @brief  主函数
 * @note   演示MPU6050 DMP的完整使用流程
 * @retval None
 */
int main(void) {
  u8 res = 0;

  // 系统初始化
  delay_init();  // 初始化延时函数
  USART1_Init(); // 初始化串口1，用于打印调试信息

  // 初始化MPU6050 DMP
  res = MPU6050_DMP_Init();
  hmc5883l_init();
  if (res != 0) {
    printf("MPU6050 DMP初始化失败！程序终止。\r\n");
    while (1) {
      delay_ms(1000);
    }
  }

  // 打印表头
  printf("开始读取MPU6050 DMP数据...\r\n");
  printf("格式: 俯仰角 | 横滚角 | 偏航角\r\n");
  printf("------------------------------------------------\r\n");
  hmc5883l_single_measurement();
  delay_ms(10);

  // 主循环
  while (1) {
    // 读取并打印MPU6050 DMP姿态数据
    res = MPU6050_DMP_ReadAttitude();
    // 延时
    delay_ms(10);
    hmc5883l_single_measurement();
  }
}
