#include "myiic.h"
#include "delay.h"
#include "stm32f4xx_gpio.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK STM32F407开发板
// IIC 驱动代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 创建日期:2014/5/6
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2014-2024
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// 初始化IIC
void IIC_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟

  // GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);             // 初始化
  IIC_SCL = 1;
  IIC_SDA = 1;
}
// 产生IIC起始信号
void IIC_Start(void) {
  SDA_OUT(); // sda线输出
  IIC_SDA = 1;
  IIC_SCL = 1;
  delay_us(4);
  IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
  delay_us(4);
  IIC_SCL = 0; // 钳住I2C总线，准备发送或接收数据
}
// 产生IIC停止信号
void IIC_Stop(void) {
  SDA_OUT(); // sda线输出
  IIC_SCL = 0;
  IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
  delay_us(4);
  IIC_SCL = 1;
  IIC_SDA = 1; // 发送I2C总线结束信号
  delay_us(4);
}
// 等待应答信号到来
// 返回值：1，接收应答失败
//         0，接收应答成功
u8 IIC_Wait_Ack(void) {
  u8 ucErrTime = 0;
  SDA_IN(); // SDA设置为输入
  IIC_SDA = 1;
  delay_us(1);
  IIC_SCL = 1;
  delay_us(1);
  while (READ_SDA) {
    ucErrTime++;
    if (ucErrTime > 250) {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL = 0; // 时钟输出0
  return 0;
}
// 产生ACK应答
void IIC_Ack(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  delay_us(2);
  IIC_SCL = 1;
  delay_us(2);
  IIC_SCL = 0;
}
// 不产生ACK应答
void IIC_NAck(void) {
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  delay_us(2);
  IIC_SCL = 1;
  delay_us(2);
  IIC_SCL = 0;
}
void IIC_Hang() {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    IIC_Stop();
    delay_us(10);
  }
}
// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void IIC_Send_Byte(u8 txd) {
  u8 t;
  SDA_OUT();
  IIC_SCL = 0; // 拉低时钟开始数据传输
  for (t = 0; t < 8; t++) {
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    delay_us(2); // 对TEA5767这三个延时都是必须的
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
    delay_us(2);
  }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack) {
  unsigned char i, receive = 0;
  SDA_IN(); // SDA设置为输入
  for (i = 0; i < 8; i++) {
    IIC_SCL = 0;
    delay_us(2);
    IIC_SCL = 1;
    receive <<= 1;
    if (READ_SDA)
      receive++;
    delay_us(1);
  }
  if (!ack)
    IIC_NAck(); // 发送nACK
  else
    IIC_Ack(); // 发送ACK
  return receive;
}

/**********************************************
// IIC Write byte
**********************************************/

void Write_IIC_Byte(unsigned char txd) {
  u8 t;
  SDA_OUT();
  IIC_SCL = 0; // 拉低时钟开始数据传输
  for (t = 0; t < 8; t++) {
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    delay_us(2); // 对TEA5767这三个延时都是必须的
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
    delay_us(2);
  }
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command) {
  IIC_Start();
  Write_IIC_Byte(0x78); // Slave address,SA0=0
  IIC_Wait_Ack();
  Write_IIC_Byte(0x00); // write command
  IIC_Wait_Ack();
  Write_IIC_Byte(IIC_Command);
  IIC_Wait_Ack();
  IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data) {
  IIC_Start();
  Write_IIC_Byte(0x78); // D/C#=0; R/W#=0
  IIC_Wait_Ack();
  Write_IIC_Byte(0x40); // write data
  IIC_Wait_Ack();
  Write_IIC_Byte(IIC_Data);
  IIC_Wait_Ack();
  IIC_Stop();
}

/**********************************************
// AK09911C 传感器支持函数
**********************************************/
/**
 * @brief 向 I2C 设备写入一个字节数据
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return 0:成功; 1:失败
 */
u8 IIC_Write_1_Byte(u8 slave_addr, u8 reg_addr, u8 data) {
  IIC_Start();
  IIC_Send_Byte(slave_addr << 1); // 发送器件地址 + 写命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg_addr); // 发送寄存器地址
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(data); // 发送数据
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Stop();
  return 0;
}

/**
 * @brief 从 I2C 设备读取一个字节数据
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param data 存储读取到的数据
 * @return 0:成功; 1:失败
 */
u8 IIC_Read_1_Byte(u8 slave_addr, u8 reg_addr, u8 *data) {
  IIC_Start();
  IIC_Send_Byte(slave_addr << 1); // 发送器件地址 + 写命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg_addr); // 发送寄存器地址
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  
  IIC_Start();
  IIC_Send_Byte((slave_addr << 1) | 0x01); // 发送器件地址 + 读命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  *data = IIC_Read_Byte(0); // 读取数据，发送 NACK
  IIC_Stop();
  return 0;
}

/**
 * @brief 从 I2C 设备连续读取多个字节数据
 * @param slave_addr 从机地址
 * @param start_reg 起始寄存器地址
 * @param n_bytes 要读取的字节数
 * @param r_data 存储读取到的数据缓冲区
 * @return 0:成功; 1:失败
 */
u8 IIC_Read_N_Bytes(u8 slave_addr, u8 start_reg, u8 n_bytes, u8 *r_data) {
  u8 i;
  
  IIC_Start();
  IIC_Send_Byte(slave_addr << 1); // 发送器件地址 + 写命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(start_reg); // 发送起始寄存器地址
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  
  IIC_Start();
  IIC_Send_Byte((slave_addr << 1) | 0x01); // 发送器件地址 + 读命令
  if (IIC_Wait_Ack()) {
    IIC_Stop();
    return 1;
  }
  
  for (i = 0; i < n_bytes; i++) {
    if (i == n_bytes - 1) {
      r_data[i] = IIC_Read_Byte(0); // 最后一个字节，发送 NACK
    } else {
      r_data[i] = IIC_Read_Byte(1); // 非最后一个字节，发送 ACK
    }
  }
  IIC_Stop();
  return 0;
}

/**
 * @brief 发送多次停止信号，确保 I2C 总线释放
 * @note 用于处理 I2C 总线挂死的情况
 */
void IIC_Stop_Hang(void) {
  u8 i;
  for (i = 0; i <= 3; i++) {
    IIC_Stop();
    delay_us(80);
  }
}
