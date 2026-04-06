#include "delay.h"
#include "hmc5883l.h"
#include "sdio_sd.h"
#include "stm32f4xx.h"
#include "task.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
// #include "inv_mpu.h"
#include "encoder.h"
#include "ff.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_speed.h"
#include "rtc.h"
HMC5883L_Data_t data;
uint8_t hmc_flag = 0;
FATFS fs;    // 文件系统对象
FIL file;    // 文件对象
FRESULT res; // 操作结果
UINT bw;     // 写入字节数
UINT br;     // 读取字节数
void get_data(void) {
  // uint32_t start = GetSysTick(), end;
  hmc5883l_read_data(&data);
  // printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d,\r\n",
  // raw_data[0],raw_data[1],raw_data[2],raw_data[3],raw_data[4],raw_data[5],data.raw_data[0],
  // data.raw_data[1], data.raw_data[2]);
  // printf("mx:%.5f,my:%.5f,mz:%.5f \r\n", data.x, data.y, data.z);
  printf("%.4f\r\n", HMC5883L_Get_Azimuth2());
  hmc5883l_single_measurement();
  // printf("get_data task tim:%d\r\n", GetSysTick());
  // MPU_Updata();
  // end = GetSysTick();
  // printf("get_data execution time: %d ms\r\n", end - start);
}
/**
 * @brief 保存HMC5883L校准参数到SD卡
 * @param data 包含校准参数的磁力计数据结构体
 * @retval 0表示成功，非0表示失败
 */
// ... existing code ...
// ... existing code ...
uint8_t save_calibration_to_sd(HMC5883L_Data_t *data) {
  char filename[] = "0:/hmc5883l_calib.txt";
  char buffer[256];
  FRESULT fres;
  UINT bytes_written;

  // 获取当前时间
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

  // 以追加模式打开文件，如果不存在则创建
  fres = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
  if (fres != FR_OK) {
    printf("打开/创建文件失败: %d\r\n", fres);
    return 1;
  }

  // 将文件指针移动到末尾
  // 修复：使用 f_size() 获取文件大小，而不是直接访问内部成员 fsize
  f_lseek(&file, f_size(&file));

  // 写入分隔线和日期
  sprintf(buffer, "\n========== Calibration Record ==========\n");
  f_write(&file, buffer, strlen(buffer), &bytes_written);

  sprintf(buffer, "Date: 20%02d-%02d-%02d %02d:%02d:%02d\n",
          RTC_DateStructure.RTC_Year, RTC_DateStructure.RTC_Month,
          RTC_DateStructure.RTC_Date, RTC_TimeStructure.RTC_Hours,
          RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);
  f_write(&file, buffer, strlen(buffer), &bytes_written);

  // 写入偏移参数
  sprintf(buffer, "\nOffset (Hard Iron Compensation):\n");
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "X = %.6f\n", data->offset[0]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "Y = %.6f\n", data->offset[1]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "Z = %.6f\n", data->offset[2]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);

  // 写入比例参数
  sprintf(buffer, "\nScale (Soft Iron Compensation):\n");
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "X = %.6f\n", data->scale[0]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "Y = %.6f\n", data->scale[1]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);
  sprintf(buffer, "Z = %.6f\n", data->scale[2]);
  f_write(&file, buffer, strlen(buffer), &bytes_written);

  sprintf(buffer, "========================================\n");
  f_write(&file, buffer, strlen(buffer), &bytes_written);

  // 关闭文件
  f_close(&file);

  printf("校准参数已追加到文件: %s\r\n", filename);
  return 0;
}
// ... existing code ...
// ... existing code ...
void hmc5883l_CA(void) {
  if (hmc_flag != 0) {
    if (hmc5883l_calibrate(1000) == 0) {
      hmc_flag = 0;
      // 在sd卡中创建一个时间命名的文件，并将校准参数保存到其中
      HMC5883L_Data_t *calib_data = hmc5883l_get_data();
      save_calibration_to_sd(calib_data);
    } else {
      printf(".");
    }

  } else {
    get_data();
  }
}
void speed_control_task(void) {
  // 更新右轮速度环PID控制器
  // printf("speed_control_task tim:%d\r\n", GetSysTick());
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
void system_init(void) {
  delay_init();
  delay_ms(2000);
  USART1_Init();
  // 初始化RTC时钟
  RTC_CLK_Config();
  // 设置时间和日期
  RTC_TimeAndDate_Set();
  printf("RTC初始化完成\r\n");

  if (SD_Init() != SD_OK) {
    printf("SD卡初始化失败！\r\n");
    while (1)
      ;
  }
  res = f_mount(&fs, "0:", 1);
  if (res != FR_OK) {
    printf("文件系统挂载失败:%d\r\n", res);
    while (1)
      ;
  }
  printf("文件系统挂载成功\r\n");
}
int main(void) {
  system_init();
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
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.04f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, -0.04f);
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
  // get_data();
  Task_Init();
  add_task(Encoder_Update, 1);
  add_task(speed_control_task, 5);
  add_task(hmc5883l_CA, 7);
  add_task(hmc5883l_single_measurement, 7);
  // add_task(encoder_print_data, 5);

  while (1) {
    Task_Scheduler();
  }
}
