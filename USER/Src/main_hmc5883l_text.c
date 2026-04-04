#include "delay.h"
#include "encoder.h" // 编码器模块
#include "led.h"
#include "motor.h"
#include "pid_speed.h" // 添加速度环PID控制器头文件
#include "rtc.h"
#include "task.h" // 任务调度模块
#include "ulog.h"s
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stm32f4xx.h>

#include "hmc5883l.h"

// 全局变量用于存储磁力计数据
HMC5883L_Data_t mag_data;

/**
 * @brief  HMC5883L磁力计测试函数
 * @note   此函数通过串口输出HMC5883L磁力计的测量数据
 */
void HMC5883L_Test_Example(void) {
  uint8_t check_result;
  
  // 1. 初始化HMC5883L磁力计
  printf("正在初始化HMC5883L磁力计...\r\n");
  hmc5883l_init();
  
  // 2. 检测HMC5883L是否存在
  printf("正在检测HMC5883L磁力计...\r\n");
  check_result = hmc5883l_read_id_info();
  
  if (check_result == 0) {
    printf("HMC5883L磁力计检测成功！\r\n");
  } else {
    printf("HMC5883L磁力计检测失败！请检查硬件连接。\r\n");
    return;
  }
  
  // 3. 设置传感器增益
  set_sensor_gain(HMC5883L_GAIN_390);
  
  // 4. 循环读取并输出磁力计数据
  printf("开始读取HMC5883L磁力计数据...\r\n");
  printf("格式: 磁场强度(X,Y,Z) | 方位角(Azimuth)\r\n");
  printf("----------------------------------------\r\n");
  
  while (1) {
    // 启动一次测量
    hmc5883l_single_measurement();
    
    // 等待测量完成
    delay_ms(10);
    
    // 读取测量数据
    hmc5883l_read_data(&mag_data);
    
    // 计算方位角
    float azimuth = HMC5883L_Get_Azimuth();
    
    // 打印磁场强度数据 (单位: Gauss)
    printf("磁场强度: %.2f, %.2f, %.2f | ", mag_data.x, mag_data.y, mag_data.z);
    
    // 打印方位角数据 (单位: 度)
    printf("方位角: %.2f\r\n", azimuth);
    
    // 延时500ms
    delay_ms(100);
  }
}

int main(void) {
  // 初始化系统
  delay_init();
  USART1_Init();
  
  // 运行HMC5883L测试函数
  HMC5883L_Test_Example();
  
  // 主循环
  while (1) {
    // 主循环
  }
}
