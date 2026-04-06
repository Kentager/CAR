/* Includes ------------------------------------------------------------------*/
#include "hmc5883l.h"
#include "myiic.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*传感器增益*/
uint16_t HMC5883L_GAIN = 1090;
/*HMC2883L数据*/
HMC5883L_Data_t HMC5883L_Data;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  I2C写一个字节到QMC5883L寄存器
 * @param  reg_addr: 寄存器地址
 * @param  data: 要写入的数据
 * @retval  返回0表示写入成功，1表示失败 (例如：无ACK)
 */
uint8_t i2c_master_write_byte(uint8_t reg_addr, uint8_t data) {
  uint8_t success = 1;
  IIC_Start();
  // 发送设备地址+写操作 (slave_addr << 1) | 0x00
  IIC_Send_Byte((HMC5883L_I2C_ADDRESS << 1) | 0x00);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  IIC_Send_Byte(reg_addr);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  IIC_Send_Byte(data);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  success = 0;
flag1:
  IIC_Stop();
  if (success != 0) {
    HMC5883L_DEBUG("I2C写HMC5883L寄存器失败-reg_addr:%d,data:%d\r\n", reg_addr,
                   data);
  }
  return success;
}

/**
 * @brief  I2C从HMC5883L寄存器读取数据(顺序读取多个数据)
 * @note
 * @param  reg_addr: 寄存器地址
 * @param  data: 指向存储读取数据的变量的指针
 * @param  length: 要读取的数据长度
 * @retval  返回0表示读取成功，1表示失败 (例如：无ACK)
 */
uint8_t i2c_master_read_data(uint8_t reg_addr, uint8_t *data, uint8_t length) {
  uint8_t success = 1;
  IIC_Start();
  // 1. 发送设备地址+写操作 (用于设置内部寄存器地址)
  IIC_Send_Byte((HMC5883L_I2C_ADDRESS << 1) | 0x00);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  // 2. 发送要读取的寄存器地址
  IIC_Send_Byte(reg_addr);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  // 3. 发送重复起始信号
  IIC_Start();
  // 4. 发送设备地址+读操作
  IIC_Send_Byte((HMC5883L_I2C_ADDRESS << 1) | 0x01);
  if (IIC_Wait_Ack() != IIC_ACK) {
    goto flag1;
  }
  /*5.接收数据并发送ACK*/
  for (uint8_t i = 0; i < length - 1; i++) {
    data[i] = IIC_Read_Byte(IIC_ACK); // true = 发送ACK
    // IIC_Ack();
  }
  // 6. 接收数据并发送NACK (表示这是最后一个要读取的字节)
  data[length - 1] = IIC_Read_Byte(IIC_NACK); // false = 发送NACK
  IIC_NAck();
  success = 0;
flag1:
  IIC_Stop();
  if (success != 0) {
    HMC5883L_DEBUG("I2C读HMC5883L寄存器失败-reg_addr:%d\r\n", reg_addr);
  }
  return success;
}

/**
 * @brief  设置传感器增益
 * @note
 * @param  无
 * @retval 无
 */
void set_sensor_gain(HMC5883L_Gain_t gain) {
  i2c_master_write_byte(HMC5883L_REG_CONFIG_B, gain);
  switch (gain) {
  case HMC5883L_GAIN_1370:
    HMC5883L_GAIN = 1370;
    break;
  case HMC5883L_GAIN_1090:
    HMC5883L_GAIN = 1090;
    break;
  case HMC5883L_GAIN_820:
    HMC5883L_GAIN = 820;
    break;
  case HMC5883L_GAIN_660:
    HMC5883L_GAIN = 660;
    break;
  case HMC5883L_GAIN_440:
    HMC5883L_GAIN = 440;
    break;
  case HMC5883L_GAIN_390:
    HMC5883L_GAIN = 390;
    break;
  case HMC5883L_GAIN_330:
    HMC5883L_GAIN = 330;
    break;
  case HMC5883L_GAIN_230:
    HMC5883L_GAIN = 230;
    break;
  default:
    break;
  }
}

/**
 * @brief  QMC5883L初始化
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_init(void) {
  /*等待qmc5883l上电稳定*/
  delay_ms(10);
  IIC_Init();
  /*写配置寄存器A*/
  /*采样平均次数8(11)，数据输出数据15Hz(默认)(100),正常测量模式(00)*/
  i2c_master_write_byte(HMC5883L_REG_CONFIG_A, 0x78);
  /*设置传感器增益*/
  set_sensor_gain(HMC5883L_GAIN_1090);
  /*写模式寄存器*/
  i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
  // 单次装换时间最快在160hz
  delay_ms(10);
  // 初始化默认校准参数
  HMC5883L_Data.offset[0] = -0.177f;
  HMC5883L_Data.offset[1] = -0.264f;
  HMC5883L_Data.offset[2] = -0.773f;
  HMC5883L_Data.scale[0] = 0.706f;
  HMC5883L_Data.scale[1] = 0.683f;
  HMC5883L_Data.scale[2] = 8.667f;
}

/**
 * @brief
 * HMC5883L启动测量一次数据(单次测量模式下，数据输出速率最大可达160Hz，连续测量模式则为75hz)
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_single_measurement(void) {
  /*写模式寄存器*/
  i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
}

/**
 * @brief  读取测量数据
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_read_data(HMC5883L_Data_t *data) {
  uint8_t temp[6];
  if (i2c_master_read_data(HMC5883L_REG_DOUT_X_MSB, temp, 6) != 0) {
    HMC5883L_DEBUG("读取HMC5883L测量数据失败\r\n");
    return;
  }
  // printf("data 1:%d,data 2:%d,data 3:%d,data 4:%d,data 5:%d,data 6:%d\r\n",
  //        temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
  data->raw_data[0] = (int16_t)((uint16_t)temp[0] << 8 | temp[1]);
  data->raw_data[2] = (int16_t)((uint16_t)temp[2] << 8 | temp[3]);
  data->raw_data[1] = (int16_t)((uint16_t)temp[4] << 8 | temp[5]);
  /*转换为高斯单位*/
  data->x = (float_t)data->raw_data[0] / HMC5883L_GAIN;
  data->y = (float_t)data->raw_data[1] / HMC5883L_GAIN;
  data->z = (float_t)data->raw_data[2] / HMC5883L_GAIN;

  /*应用硬铁/软铁补偿 - 临时禁用校准，直接使用原始数据*/
  data->x = (data->x - HMC5883L_Data.offset[0]) * HMC5883L_Data.scale[0];
  data->y = (data->y - HMC5883L_Data.offset[1]) * HMC5883L_Data.scale[1];
  data->z = (data->z - HMC5883L_Data.offset[2]) * HMC5883L_Data.scale[2];
}

/**
 * @brief  读取HMC5883L识别寄存器并输出信息
 * @note   HMC5883L有三个识别寄存器：A(0x0A)、B(0x0B)和C(0x0C)
 * @retval 0表示成功，1表示失败
 */
uint8_t hmc5883l_read_id_info(void) {
  uint8_t id_a, id_b, id_c;

  // 读取识别寄存器A
  if (i2c_master_read_data(HMC5883L_REG_CHIP_IDA, &id_a, 1) != 0) {
    printf("读取HMC5883L识别寄存器A失败\r\n");
    return 1;
  }

  // 读取识别寄存器B
  if (i2c_master_read_data(HMC5883L_REG_CHIP_IDB, &id_b, 1) != 0) {
    printf("读取HMC5883L识别寄存器B失败\r\n");
    return 1;
  }

  // 读取识别寄存器C
  if (i2c_master_read_data(HMC5883L_REG_CHIP_IDC, &id_c, 1) != 0) {
    printf("读取HMC5883L识别寄存器C失败\r\n");
    return 1;
  }

  // 输出识别信息
  printf("HMC5883L识别信息:\r\n");
  printf("ID A: 0x%02X (%c)\r\n", id_a, id_a);
  printf("ID B: 0x%02X (%c)\r\n", id_b, id_b);
  printf("ID C: 0x%02X (%c)\r\n", id_c, id_c);

  // 验证是否为HMC5883L
  if (id_a == 'H' && id_b == '4' && id_c == '3') {
    printf("检测到HMC5883L传感器\r\n");
    return 0;
  } else {
    printf("警告: 识别码不匹配，可能不是HMC5883L传感器\r\n");
    return 1;
  }
}
/**
 * @brief 计算方位角（0-360度）
 * @param pitch 俯仰角(弧度)
 * @param roll 横滚角(弧度)
 * @return 方位角(度)
 */
float HMC5883L_Get_Azimuth(float pitch, float roll) {
  // 计算方位角（0-360度）
  hmc5883l_read_data(&HMC5883L_Data);

  // 将磁力计数据从机体坐标系转换到地理坐标系
  float sin_roll = sinf(roll);
  float cos_roll = cosf(roll);
  float sin_pitch = sinf(pitch);
  float cos_pitch = cosf(pitch);
  // printf("sin_pitch:%f, cos_pitch:%f, sin_roll:%f, cos_roll:%f\r\n",
  // sin_pitch,
  //        cos_pitch, sin_roll, cos_roll);
  // printf("x:%.6f,y:%.6f,z:%.6f\r\n", HMC5883L_Data.x, HMC5883L_Data.y,
  //  HMC5883L_Data.z);
  // 旋转后的磁场分量
  float hx = HMC5883L_Data.x * cos_pitch +
             HMC5883L_Data.y * sin_pitch * sin_roll +
             HMC5883L_Data.z * sin_pitch * cos_roll;
  float hy = HMC5883L_Data.y * cos_roll - HMC5883L_Data.z * sin_roll;
  // printf("hx:%.6f,hy:%.6f\r\n", hx, hy);
  // 计算航向角(弧度)
  float heading_rad = atan2f(hy, hx);

  // 转换为角度
  float heading_deg = heading_rad * 180.0f / M_PI;

  // 确保航向角在0-360度范围内
  if (heading_deg < 0) {
    heading_deg += 360.0f;
  }

  return heading_deg;
}
float HMC5883L_Get_Azimuth2(void) {
  // 读取磁力计数据
  hmc5883l_read_data(&HMC5883L_Data);

  // 假设载体处于水平状态，直接使用x和y分量计算航向角
  // 计算航向角(弧度)
  float heading_rad = atan2f(HMC5883L_Data.y, HMC5883L_Data.x);

  // 转换为角度
  float heading_deg = heading_rad * 180.0f / M_PI;

  // 确保航向角在0-360度范围内
  if (heading_deg < 0) {
    heading_deg += 360.0f;
  }

  return heading_deg;
}

/**
 * @brief 磁力计校准
 * @note 通过收集不同方向的数据计算校准参数
 * @param samples 采样点数
 * @return 0表示成功，非0表示失败
 */
int hmc5883l_calibrate(uint16_t samples) {
  float x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
  HMC5883L_Data_t temp_data;

  // 收集数据
  for (uint16_t i = 0; i < samples; i++) {
    hmc5883l_single_measurement();
    delay_ms(10);
    hmc5883l_read_data(&temp_data);
    if (i % 5 == 0)
      printf("x:%6.3f,y:%6.3f,z:%6.3f\r\n", temp_data.x, temp_data.y,
             temp_data.z);
    // 更新最小值和最大值
    if (i == 0) {
      x_min = x_max = temp_data.x;
      y_min = y_max = temp_data.y;
      z_min = z_max = temp_data.z;
    } else {
      if (temp_data.x < x_min)
        x_min = temp_data.x;
      if (temp_data.x > x_max)
        x_max = temp_data.x;
      if (temp_data.y < y_min)
        y_min = temp_data.y;
      if (temp_data.y > y_max)
        y_max = temp_data.y;
      if (temp_data.z < z_min)
        z_min = temp_data.z;
      if (temp_data.z > z_max)
        z_max = temp_data.z;
    }
  }

  // 计算偏移量(硬铁补偿)
  HMC5883L_Data.offset[0] = (x_max + x_min) / 2.0f;
  HMC5883L_Data.offset[1] = (y_max + y_min) / 2.0f;
  HMC5883L_Data.offset[2] = (z_max + z_min) / 2.0f;

  // 计算缩放因子(软铁补偿)
  float x_range = x_max - x_min;
  float y_range = y_max - y_min;
  float z_range = z_max - z_min;
  float avg_range = (x_range + y_range + z_range) / 3.0f;

  HMC5883L_Data.scale[0] = avg_range / x_range;
  HMC5883L_Data.scale[1] = avg_range / y_range;
  HMC5883L_Data.scale[2] = avg_range / z_range;

  // 打印校准参数
  printf("磁力计校准完成:\r\n");
  printf("偏移量: X=%.2f, Y=%.2f, Z=%.2f\r\n", HMC5883L_Data.offset[0],
         HMC5883L_Data.offset[1], HMC5883L_Data.offset[2]);
  printf("缩放因子: X=%.2f, Y=%.2f, Z=%.2f\r\n", HMC5883L_Data.scale[0],
         HMC5883L_Data.scale[1], HMC5883L_Data.scale[2]);

  return 0;
}
/**
 * @brief 获取磁力计数据
 * @return 磁力计数据结构体指针
 */
HMC5883L_Data_t *hmc5883l_get_data(void) { return &HMC5883L_Data; }
