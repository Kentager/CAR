#include "jy61p.h"
#include "delay.h"
#include "myiic.h"
#include <stdint.h>
#include <stdio.h>

#define delay_3us() Delay_us(1)

#define JY61p_IIC_SDA_IO_IN()                                                  \
  { GPIOB->MODER &= ~(0x03 << 2 * 9); } // 设置JY61p_W_SDA为输入模式
#define JY61p_IIC_SDA_IO_OUT()                                                 \
  { GPIOB->MODER |= (0x01 << 2 * 9); } // 设置JY61p_W_SDA为输出模式

#define JY61p_W_SCL(BIT) GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)BIT)
#define JY61p_W_SDA(BIT) GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)BIT)
#define JY61p_R_SDA() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

u8 JY61p_AngleData[9]; // 8位角度原始数据 1位完成位

void JY61p_Init(void) {
  JY61p_IIC_Init();
  u16 i = 0;
  // 检测 JY61p 是否存在
  while (JY61p_Check() && i < 1000) {
    i++;
    delay_ms(1);
  }
}

#include "delay.h"
#include "jy61p.h"
#include "myiic.h"

/***********************************************************
 *@fuction	: JY61p_IIC_Init
 *@brief		: JY61p 的软件 iic 的 gpio 初始化
 *@param		: None
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
void JY61p_IIC_Init(void) { IIC_Init(); }

// ... existing code ...

/***********************************************************
 *@fuction	: JY61p_IIC_Start
 *@brief		: 产生 MPU6050_IIC 起始信号
 *@param		: None
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
void JY61p_IIC_Start(void) { IIC_Start(); }

/***********************************************************
 *@fuction	: JY61p_IIC_Stop
 *@brief		: 产生 JY61p_IIC 停止信号
 *@param		: None
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
void JY61p_IIC_Stop(void) { IIC_Stop(); }

/***********************************************************
 *@fuction	: JY61p_IIC_SendAck
 *@brief		: 发送 ACK 应答
 *@param		: AckBit - ACK信号
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
uint8_t JY61p_IIC_SendAck(uint8_t AckBit) {
  if (!AckBit) {

    printf("send ACK %d\r\n", AckBit);
    IIC_Ack();
  } else {
    printf("send NACK %d\r\n", AckBit);
    IIC_NAck();
  }
  return 0;
}

/***********************************************************
 *@fuction	: JY61p_IIC_WaitAck
 *@brief		: 等待 JY61p 应答信号到来
 *@param		: None
 *@return	: 应答位，0:应答;  1:无应答
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
uint8_t JY61p_IIC_WaitAck(void) { return IIC_Wait_Ack(); }

/***********************************************************
 *@fuction	: JY61p_IIC_SendByte
 *@brief		: JY61p 软件 IIC 发送一个字节
 *@param		: 数据
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
void JY61p_IIC_SendByte(uint8_t dat) { IIC_Send_Byte(dat); }

/***********************************************************
 *@fuction	: JY61p_IIC_SendByte
 *@brief		: JY61p 软件 IIC 发送一个字节
 *@param		: 数据
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
uint8_t JY61p_IIC_ReceiveByte(void) { return IIC_Read_Byte(1); }
/**
 * @brief  对接官方i2c_write
 * @retval 0: 读成功
 */
u8 JY61p_Write(u8 addr, u8 reg, u8 len, u8 *buf) {
  u8 i;
  JY61p_IIC_Start();
  JY61p_IIC_SendByte((addr << 1) | 0x00); // 发送器件地址+写命令
  if (JY61p_IIC_WaitAck()) {
    JY61p_IIC_Stop();
    return 1;
  }
  JY61p_IIC_SendByte(reg);
  JY61p_IIC_WaitAck();
  for (i = 0; i < len; i++) {
    JY61p_IIC_SendByte(buf[i]);
    if (JY61p_IIC_WaitAck()) {
      JY61p_IIC_Stop();
      return 1;
    }
  }
  JY61p_IIC_Stop();
  return 0;
}

/**
 * @brief  对接官方dmp库的i2c_read函数
 * @retval 0: 读成功
 */
u8 JY61p_Read(u8 addr, u8 reg, u8 len, u8 *buf) {
  JY61p_IIC_Start();
  JY61p_IIC_SendByte((addr << 1) | 0x00); // 发送器件地址+写命令
  if (JY61p_IIC_WaitAck()) {
    JY61p_IIC_Stop();
    return 1;
  }
  JY61p_IIC_SendByte(reg);
  JY61p_IIC_WaitAck();
  JY61p_IIC_Start();
  JY61p_IIC_SendByte((addr << 1) | 0x01);
  JY61p_IIC_WaitAck();
  while (len) {
    if (len == 1) {
      *buf = IIC_Read_Byte(1); // 最后一个字节,发送nack
    } else {
      *buf = IIC_Read_Byte(0); // 非最后一个字节,发送ack
    }
    len--;
    buf++;
  }
  JY61p_IIC_Stop();
  return 0;
}

/***********************************************************
 *@fuction	: JY61p_Check
 *@brief		: JY61p 检测是否存在
 *@param		: None
 *@return	: 0: 存在
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
uint8_t JY61p_Check(void) {
  uint8_t ack = 0;
  JY61p_IIC_Start();
  JY61p_IIC_SendByte(JY61p_DeviceAddr << 1);
  ack = JY61p_IIC_WaitAck();
  JY61p_IIC_Stop();

  return ack;
}

/***********************************************************
 *@fuction	: JY61p_Get
 *@brief		: JY61p 获取欧拉角和角速度
 *@param		: 加速度, 欧拉角, 角速度
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.04.02
 ***********************************************************/
void JY61p_Get(Acc_Param *a, EulerAngle_Param *p, Gyro_Param *v) {
  uint8_t tmp[30];
  if (JY61p_Read(JY61p_DeviceAddr, JY61p_AX, 24, tmp) == 0) {
    // 读取成功，继续处理数据
  } else {
    // 读取失败，处理错误（例如：返回默认值或记录错误日志）
    a->Ax = a->Ay = a->Az = 0.0f;
    v->Gx = v->Gy = v->Gz = 0.0f;
    p->Roll = p->Pitch = p->Yaw = 0.0f;
    printf("get jy61p data erro\r\n");
    return;
  }
  for (uint8_t i = 0; i < 24; i++) {
    printf("%02X  ", tmp[i]);
  }
  printf("\r\n");

  /**** 加速度 ****/
  a->Ax = ((short)(tmp[1] << 8) | tmp[0]) / 32768.00f * 16;
  a->Ay = ((short)(tmp[3] << 8) | tmp[2]) / 32768.00f * 16;
  a->Az = ((short)(tmp[5] << 8) | tmp[4]) / 32768.00f * 16;

  /**** 角速度 ****/
  v->Gx = ((short)(tmp[7] << 8) | tmp[6]) / 32768.00f * 2000;
  v->Gy = ((short)(tmp[9] << 8) | tmp[8]) / 32768.00f * 2000;
  v->Gz = ((short)(tmp[11] << 8) | tmp[10]) / 32768.00f * 2000;

  /**** JY61p没有 ****/
  //   ((short)(tmp[13]<<8) | tmp[12]);
  //   ((short)(tmp[15]<<8) | tmp[14]);
  //   ((short)(tmp[17]<<8) | tmp[16]);

  /**** 欧拉角 ****/
  p->Roll = ((short)(tmp[19] << 8) | tmp[18]) / 32768.00f * 180;
  p->Pitch = ((short)(tmp[21] << 8) | tmp[20]) / 32768.00f * 180;
  p->Yaw = ((short)(tmp[23] << 8) | tmp[22]) / 32768.00f * 180;
}