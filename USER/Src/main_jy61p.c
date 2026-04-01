#include "SENSOR_AK09911C.h"
#include "delay.h"
#include "jy61p.h"
#include "led.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <stdint.h>

// 全局变量用于存储传感器数据
Acc_Param acc_data;
EulerAngle_Param euler_data;
Gyro_Param gyro_data;

// AK09911C 全局变量
AK09911C_AXIS_DATA mag_raw;                // 原始磁力计数据
AK09911C_AXIS_DATA mag_cal;                // 校准后磁力计数据
AK09911C_CALIBRATE cal_factor;             // 校准因子
AK09911C_4_QUADRANT_DATA_POINT quad_ready; // 象限数据采集状态
float azimuth = 0.0f;                      // 方位角

/**
 * @brief  JY61P传感器测试例程
 * @note   此函数演示了JY61P传感器的完整使用流程
 */
void JY61P_Test_Example(void) {
  u8 check_result;

  // 1. 初始化JY61P传感器
  printf("正在初始化JY61P传感器...\r\n");
  JY61p_Init();

  // 2. 检测JY61P是否存在
  printf("正在检测JY61P传感器...\r\n");
  check_result = JY61p_Check();
  if (check_result == 0) {
    printf("JY61P传感器检测成功！\r\n");
  } else {
    printf("JY61P传感器检测失败！请检查硬件连接。\r\n");
    return;
  }

  // 3. 循环读取传感器数据
  printf("开始读取JY61P传感器数据...\r\n");
  printf(
      "格式: 加速度(Ax,Ay,Az) | 角速度(Gx,Gy,Gz) | 欧拉角(Roll,Pitch,Yaw)\r\n");
  printf(
      "-----------------------------------------------------------------\r\n");

  while (1) {
    // 读取传感器数据
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

/**
 * @brief  JY61P传感器配置示例
 * @note   演示如何配置JY61P传感器的参数
 */
void JY61P_Config_Example(void) {
  u8 config_buffer[1];
  u8 result;

  printf("JY61P传感器配置示例\r\n");

  // 示例1: 设置输出频率为20Hz (寄存器0x03)
  config_buffer[0] = 0x04; // 20Hz对应值为4
  result = JY61p_Write(JY61p_DeviceAddr, JY61p_RRATE, 1, config_buffer);
  if (result == 0) {
    printf("成功设置输出频率为20Hz\r\n");
  } else {
    printf("设置输出频率失败\r\n");
  }

  // 示例2: 保存当前配置到EEPROM (寄存器0x00)
  config_buffer[0] = 0x00; // 保存命令
  result = JY61p_Write(JY61p_DeviceAddr, JY61p_SAVE, 1, config_buffer);
  if (result == 0) {
    printf("成功保存配置到EEPROM\r\n");
  } else {
    printf("保存配置失败\r\n");
  }

  delay_ms(100); // 等待保存完成
}
/**
 * @brief  AK09911C 传感器初始化
 * @retval 0: 成功；其他：失败
 */
u8 AK09911C_Init(void) {
  uint8_t id_info[4];

  printf("正在初始化 AK09911C 传感器...\r\n");

  // 读取芯片 ID 和信息
  if (AK09911C_GET_ID_INFO(id_info) != 0) {
    printf("读取芯片 ID 失败！\r\n");
    return 1;
  }

  printf("芯片 ID: 0x%02X, 设备 ID: 0x%02X\r\n", id_info[0], id_info[1]);
  printf("信息 1: 0x%02X, 信息 2: 0x%02X\r\n", id_info[2], id_info[3]);

  // 初始化传感器
  if (AK09911C_SET_INITIAL() != 0) {
    printf("传感器初始化失败！\r\n");
    return 1;
  }

  printf("AK09911C 初始化完成\r\n");
  printf("ASA 补偿值已加载（X, Y, Z 轴灵敏度补偿）\r\n");

  return 0;
}

/**
 * @brief  AK09911C 数据采集例程（需要旋转设备 360 度）
 * @note   用于采集校准数据点
 */
void AK09911C_Collect_Calibration_Data(void) {
  u8 progress = 0;
  u8 last_progress = 0;

  printf("\r\n=== AK09911C 数据采集模式 ===\r\n");
  printf("请缓慢均匀旋转设备 360 度，采集各象限数据...\r\n");
  printf("进度：等待数据...\r\n");

  while (1) {
    if (AK09911C_GET_XYZ_DATA(&mag_raw) == 0) {
      // 显示原始数据
      printf("\r原始：X=%5d, Y=%5d, Z=%5d", mag_raw.X_AXIS, mag_raw.Y_AXIS,
             mag_raw.Z_AXIS);

      // 应用 ASA 补偿
      mag_cal = mag_raw;
      AK09911C_GET_CAL(&mag_cal);
      printf("  | 校准：X=%5d, Y=%5d, Z=%5d", mag_cal.X_AXIS, mag_cal.Y_AXIS,
             mag_cal.Z_AXIS);

      // 采集校准数据点
      if (AK09011C_GET_CALIBRATE_DATA(mag_cal, &quad_ready) == 1) {
        printf("\r\n数据采集完成！\r\n");
        break;
      }

      // 每采集一个新数据点显示一次进度
      if (quad_ready.first_quadant_data_pointok ||
          quad_ready.second_quadant_data_pointok ||
          quad_ready.third_quadant_data_pointok ||
          quad_ready.fourth_quadant_data_pointok) {
        printf("\r进度更新中...");
      }
    } else {
      printf("\r读取数据失败...");
    }

    delay_ms(100);
  }

  // 计算校准因子
  printf("\r\n计算校准因子...\r\n");
  AK09911C_GET_CALIBRATE_FACTOR(&cal_factor);

  printf("\r\n校准结果:\r\n");
  printf("X 偏移：%f, Y 偏移：%f\r\n", cal_factor.X_OFFSET,
         cal_factor.Y_OFFSET);
  printf("X 增益：%f, Y 增益：%f\r\n", cal_factor.X_FACTOR_BASE,
         cal_factor.Y_FACTOR_BASE);
}

/**
 * @brief  AK09911C 方位角测试例程
 * @note   演示如何读取磁力计数据并计算方位角
 */
void AK09911C_Azimuth_Test(void) {
  printf("\r\n=== AK09911C 方位角测试 ===\r\n");
  printf("方向指示：北 (N), 东北 (NE), 东 (E), 东南 (SE), 南 (S), 西南 (SW), "
         "西 (W), 西北 (NW)\r\n");
  printf(
      "-----------------------------------------------------------------\r\n");

  while (1) {
    // 读取磁力计数据
    if (AK09911C_GET_XYZ_DATA(&mag_raw) == 0) {
      // 应用 ASA 补偿
      mag_cal = mag_raw;
      AK09911C_GET_CAL(&mag_cal);

      // 如果已校准，应用硬铁/软铁补偿
      // mag_cal.X_AXIS = (int16_t)((mag_cal.X_AXIS - cal_factor.X_OFFSET) *
      // cal_factor.X_FACTOR_BASE); mag_cal.Y_AXIS = (int16_t)((mag_cal.Y_AXIS -
      // cal_factor.Y_OFFSET) * cal_factor.Y_FACTOR_BASE);

      // 计算方位角
      azimuth = AK09011C_GET_AZIMUTH_WITHOUT_COMPENSATION(mag_cal);

      // 显示数据
      printf("\r磁场：X=%6d, Y=%6d, Z=%6d | 方位角：%6.2f°", mag_cal.X_AXIS,
             mag_cal.Y_AXIS, mag_cal.Z_AXIS, azimuth);

      // 方向判断
      printf(" | 方向：");
      if (azimuth >= 337.5 || azimuth < 22.5)
        printf("北 (N)   ");
      else if (azimuth >= 22.5 && azimuth < 67.5)
        printf("东北 (NE)");
      else if (azimuth >= 67.5 && azimuth < 112.5)
        printf("东 (E)   ");
      else if (azimuth >= 112.5 && azimuth < 157.5)
        printf("东南 (SE)");
      else if (azimuth >= 157.5 && azimuth < 202.5)
        printf("南 (S)   ");
      else if (azimuth >= 202.5 && azimuth < 247.5)
        printf("西南 (SW)");
      else if (azimuth >= 247.5 && azimuth < 292.5)
        printf("西 (W)   ");
      else if (azimuth >= 292.5 && azimuth < 337.5)
        printf("西北 (NW)");

    } else {
      printf("\r读取数据失败...");
    }

    delay_ms(100); // 10Hz 更新率
  }
}

/**
 * @brief  AK09911C 综合测试例程
 * @note   完整的 AK09911C 使用示例
 */
void AK09911C_Test_Example(void) {
  u8 init_result;

  printf("\r\n========================================\r\n");
  printf("   AK09911C 地磁传感器综合测试\r\n");
  printf("========================================\r\n\r\n");

  // 1. 初始化传感器
  init_result = AK09911C_Init();
  if (init_result != 0) {
    printf("AK09911C 初始化失败，程序终止！\r\n");
    return;
  }

  // 2. 自测试（可选）
  printf("\r\n是否执行自测试？(此功能未实现，按任意键跳过)...");
  // 等待用户输入（实际使用时可实现）
  delay_ms(1000);

  // 3. 数据采集模式（首次使用或需要校准时）
  /*
  printf("\r\n是否进入数据采集模式？(Y/N): ");
  // 等待用户输入
  // 如果用户选择 Y，则调用：
  // AK09911C_Collect_Calibration_Data();
  */

  // 4. 方位角测试
  printf("\r\n开始方位角测试...\r\n");
  AK09911C_Azimuth_Test();
}

/**
 * @brief  AK09911C 快速测试（简化版本）
 * @note   快速验证 AK09911C 是否工作正常
 */
void ak0911c_test(void) {
  u8 init_result;

  printf("\r\n=== AK09911C 快速测试 ===\r\n");

  // 初始化传感器
  init_result = AK09911C_Init();
  if (init_result != 0) {
    printf("初始化失败！\r\n");
    return;
  }

  printf("\r\n开始读取数据（按 Ctrl+C 退出）...\r\n");

  while (1) {
    // 读取磁力计数据
    if (AK09911C_GET_XYZ_DATA(&mag_raw) == 0) {
      // 应用 ASA 补偿
      mag_cal = mag_raw;
      AK09911C_GET_CAL(&mag_cal);

      // 计算方位角
      azimuth = AK09011C_GET_AZIMUTH_WITHOUT_COMPENSATION(mag_cal);

      // 显示数据
      printf("\rX=%6d, Y=%6d, Z=%6d | 方位角：%6.2f°", mag_cal.X_AXIS,
             mag_cal.Y_AXIS, mag_cal.Z_AXIS, azimuth);
    } else {
      printf("\r读取失败...");
    }

    delay_ms(200);
  }
}

/**
 * @brief  主函数 - JY61P 和 AK09911C 测试程序入口
 */
int main(void) {
  // 系统时钟初始化
  SystemInit();

  // 延时函数初始化
  delay_init();
  led_Init();
  // USART1 初始化 (用于打印调试信息)
  USART1_Init();
  uint8_t data;
  printf("\r\n=== 传感器测试程序 ===\r\n");

  printf("1. JY61P 六轴 IMU 传感器\r\n");
  printf("2. AK09911C 地磁传感器\r\n\r\n");

  // 运行配置示例（可选）
  // JY61P_Config_Example();

  // 选择要运行的测试例程：

  // 选项 1: JY61P 测试例程
  // JY61P_Test_Example();

  // 选项 2: AK09911C 快速测试（简单验证）
  ak0911c_test();


  // 选项 3: AK09911C 综合测试（完整功能）
  // AK09911C_Test_Example();

  // 选项 4: AK09911C 数据采集（需要旋转设备校准）
  // AK09911C_Collect_Calibration_Data();

  return 0;
}