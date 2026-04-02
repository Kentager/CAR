#include "mpu6050.h"
#include "delay.h"
#include "myiic.h"
#include "sys.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
static int16_t ax, ay, az;
static int16_t gx, gy, gz;
static int16_t temperature;
static float gx_dps, gy_dps, gz_dps;
static float ax_dps, ay_dps, az_dps;
int16_t raw_data[6];
EulerAngles MPU_Attitude;

// 初始化MPU6050
// 返回值:0,成功
//     其他,错误代码
u8 MPU_Init(void) {
  u8 res;
  IIC_Init();                              // 初始化IIC总线
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
  delay_ms(100);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
  MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
  MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
  MPU_Set_Rate(200);                       // 设置采样率200Hz
  MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // 关闭所有中断
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
  MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
  MPU_Read_Byte(MPU_DEVICE_ID_REG, &res);
  printf("res is 0x%02x\r\n", res);
  if (res == MPU_ADDR) // 器件ID正确
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
    MPU_Set_Rate(100);                       // 设置采样率为50Hz
  } else
    return 1;
  return 0;
}
// 设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr) {
  return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr) {
  return MPU_Write_Byte(MPU_ACCEL_CFG_REG,
                        fsr << 3); // 设置加速度传感器满量程范围
}
// 设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_LPF(u16 lpf) {
  u8 data = 0;
  if (lpf >= 188)
    data = 1;
  else if (lpf >= 98)
    data = 2;
  else if (lpf >= 42)
    data = 3;
  else if (lpf >= 20)
    data = 4;
  else if (lpf >= 10)
    data = 5;
  else
    data = 6;
  return MPU_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}
// 设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
u8 MPU_Set_Rate(u16 rate) {
  u8 data;
  if (rate > 1000)
    rate = 1000;
  if (rate < 4)
    rate = 4;
  data = 1000 / rate - 1;
  data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
  return MPU_Set_LPF(rate / 2); // 自动设置LPF为采样率的一半
}

// 得到温度值
// 返回值:温度值(扩大了100倍)
u8 MPU_Get_Temperature(void) {
  u8 res = 0, buf[2];
  short raw;

  res = MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
  if (res) {
    return res;
  }
  raw = ((u16)buf[0] << 8) | buf[1];
  temperature = 36.53 + ((double)raw) / 340;
  temperature *= 100;
  return 0;
}
// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
u8 MPU_Get_Gyroscope(void) {
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
  if (res == 0) {
    gx = ((u16)buf[0] << 8) | buf[1];
    gy = ((u16)buf[2] << 8) | buf[3];
    gz = ((u16)buf[4] << 8) | buf[5];
  }
  return res;
  ;
}
// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
u8 MPU_Get_Accelerometer(void) {
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);

  if (res == 0) {
    ax = ((u16)buf[0] << 8) | buf[1];
    ay = ((u16)buf[2] << 8) | buf[3];
    az = ((u16)buf[4] << 8) | buf[5];
  }
  return res;
  ;
}
// return :0 success
// other fail
u8 MPU_Updata(void) {
  if (MPU_Get_Accelerometer())
    return 1;
  if (MPU_Get_Gyroscope()) {
    return 2;
  }
  if (MPU_Get_Temperature()) {
    return 3;
  }
  // 陀螺仪：±2000dps 对应 16.4 LSB/(°/s)
  gx_dps = gx * (2000.0f / 32768.0f); // 或者 gx * 0.061035f
  gy_dps = gy * (2000.0f / 32768.0f);
  gz_dps = gz * (2000.0f / 32768.0f);
  ax_dps = ax * (2.0f / 32768.0f); // 或者 ax * 0.061035f
  ay_dps = ay * (2.0f / 32768.0f);
  az_dps = az * (2.0f / 32768.0f);
  raw_data[0] = ax;
  raw_data[1] = ay;
  raw_data[2] = az;
  raw_data[3] = gx;
  raw_data[4] = gy;
  raw_data[5] = gz;

  return 0;
}
void MPU_Proc(void) {
  u8 res = 0;
  res = MPU_Updata();
  if (res) {
    printf("MPU数据更新失败%d\r\n", res);
    return;
  }
  // 积分计算欧拉角
  float pitch_g = MPU_Attitude.pitch + gx_dps * 0.005;
  float roll_g = MPU_Attitude.roll + gy_dps * 0.005;
  float yaw_g = MPU_Attitude.yaw + gz_dps * 0.005;
  float pitch_a = atan2(ay_dps, az_dps) / M_PI * 180;
  float roll_a = atan2(ax_dps, az_dps) / M_PI * 180;
  MPU_Attitude.pitch = MPU_ALPHA * pitch_g + (1.0f - MPU_ALPHA) * pitch_a -
                       MPU_Attitude.Initial_Attitude[0];
  MPU_Attitude.roll = MPU_ALPHA * roll_g + (1.0f - MPU_ALPHA) * roll_a -
                      MPU_Attitude.Initial_Attitude[1];
  MPU_Attitude.yaw = yaw_g - MPU_Attitude.Initial_Attitude[2];
}
void Get_RawData(void) {
  u8 res = 0;
  res = MPU_Updata();
  if (res) {
    printf("MPU数据更新失败%d\r\n", res);
    return;
  }
  printf("ax:%6.2f,ay:%6.2f,az:%6.2f,gx_dps:%6.2f,gy_dps:%6.2f,gz_dps:%6.2f,"
         "             pitch:%6.2f,roll:%6.2f,yaw:%6.2f\r\n",
         ax_dps, ay_dps, az_dps, gx_dps, gy_dps, gz_dps, MPU_Attitude.pitch,
         MPU_Attitude.roll, MPU_Attitude.yaw);
}
// IIC连续写
// addr:器件地址
// reg:寄存器地址
// len:写入长度
// buf:数据区
// 返回值:0,正常
//     其他,错误代码
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
  u8 i;
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
  if (IIC_Wait_Ack())             // 等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg); // 写寄存器地址
  IIC_Wait_Ack();     // 等待应答
  for (i = 0; i < len; i++) {
    IIC_Send_Byte(buf[i]); // 发送数据
    if (IIC_Wait_Ack())    // 等待ACK
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_Stop();
  return 0;
}
// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
  if (IIC_Wait_Ack())             // 等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg); // 写寄存器地址
  IIC_Wait_Ack();     // 等待应答
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
  IIC_Wait_Ack();                 // 等待应答
  while (len) {
    if (len == 1)
      *buf = IIC_Read_Byte(1); // 读数据,发送nACK
    else
      *buf = IIC_Read_Byte(0); // 读数据,发送ACK
    len--;
    buf++;
  }
  IIC_Stop(); // 产生一个停止条件
  return 0;
}
// IIC写一个字节
// reg:寄存器地址
// data:数据
// 返回值:0,正常
//     其他,错误代码
u8 MPU_Write_Byte(u8 reg, u8 data) {
  IIC_Start();
  IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
  if (IIC_Wait_Ack())                 // 等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);  // 写寄存器地址
  IIC_Wait_Ack();      // 等待应答
  IIC_Send_Byte(data); // 发送数据
  if (IIC_Wait_Ack())  // 等待ACK
  {
    IIC_Stop();
    return 1;
  }
  IIC_Stop();
  return 0;
}
// IIC读一个字节
// reg:寄存器地址
// 返回值:读到的数据
u8 MPU_Read_Byte(u8 reg, u8 *data) {
  u8 res;
  IIC_Start();
  IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  } // 等待应答
  IIC_Send_Byte(reg); // 写寄存器地址
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  } // 等待应答

  IIC_Start();
  IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  } // 等待应答
  *data = IIC_Read_Byte(0); // 读取数据,发送nACK
  IIC_Stop();               // 产生一个停止条件
  return res;
}
