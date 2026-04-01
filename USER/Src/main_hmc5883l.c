#include "delay.h"
#include "hmc5883l.h"
#include "jy61p.h"
#include "rtc.h"
#include "stm32f4xx.h"
#include "ulog.h"
#include "usart.h"
// 全局变量用于存储传感器数据
Acc_Param acc_data;
EulerAngle_Param euler_data;
Gyro_Param gyro_data;
/**
 * @brief  JY61P传感器测试例程
 * @note   此函数演示了JY61P传感器的完整使用流程
 */
void my_console_logger(ulog_level_t severity, char *msg) {
  printf("%s [%s]: %s",
         RTC_GetNowTime(), // user defined function
         ulog_level_name(severity), msg);
}
void JY61P_Test_Example(void) {
  u8 check_result;

  // 1. 初始化JY61P传感器
  printf("正在初始化JY61P传感器...\r\n");
  JY61p_Init();

  // 2. 检测JY61P是否存在
  printf("正在检测JY61P传感器...\r\n");
  while (1) {
    check_result = JY61p_Check();

    if (check_result == 0) {
      printf("JY61P传感器检测成功！\r\n");
      break;
    } else {
      printf("JY61P传感器检测失败！请检查硬件连接。\r\n");
      delay_ms(500);
    }
  }

  // 3. 循环读取传感器数据
  printf("开始读取JY61P传感器数据...\r\n");
  printf(
      "格式: 加速度(Ax,Ay,Az) | 角速度(Gx,Gy,Gz) | 欧拉角(Roll,Pitch,Yaw)\r\n");
  printf(
      "-----------------------------------------------------------------\r\n");

  while (1) {
    ulog_init();
    ULOG_SUBSCRIBE(my_console_logger, ULOG_WARNING_LEVEL);
    // 读取传感器数据
    hmc5883l_single_measurement();
    delay_ms(10);
    float azimuth = HMC5883L_Get_Azimuth();
    printf("Azimuth: %6.3f\r\n", azimuth);
    JY61p_Get(&acc_data, &euler_data, &gyro_data);

    // 打印加速度数据 (单位: g)
    printf("加速度: %.2f, %.2f, %.2f | ", acc_data.Ax, acc_data.Ay,
           acc_data.Az);

    // 打印角速度数据 (单位: °/s)
    printf("角速度: %.2f, %.2f, %.2f | ", gyro_data.Gx, gyro_data.Gy,
           gyro_data.Gz);

    // 打印欧拉角数据 (单位: °)
    printf("欧拉角: %.2f, %.2f, %.2f\r\n", euler_data.Roll, euler_data.Pitch,
           euler_data.Yaw);

    // 延时500ms
    delay_ms(500);
  }
}
int main(void) {
  // 初始化传感器
  delay_init();
  USART1_Init();
  hmc5883l_init();
  hmc5883l_read_id_info();
  // // 设置传感器增益（可选）
  set_sensor_gain(HMC5883L_GAIN_390);
  JY61P_Test_Example();
  // 主循环
  while (1) {
    // 启动一次测量
    // hmc5883l_single_measurement();

    // // 等待测量完成
    // delay_ms(10);

    // // 读取测量数据
    // float azimuth = HMC5883L_Get_Azimuth();

    // // 使用测量数据
    // printf("Azimuth: %6.3f\r\n", azimuth);

    // 其他处理...
  }
}