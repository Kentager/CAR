#ifndef __JY61P_H_
#define __JY61P_H_

/**** 头文件 ****/
#include "stm32f4xx.h"

/**** 宏定义 ****/
#define JY61p_DeviceAddr 0x50

// JY61p 寄存器地址
#define JY61p_SAVE (0x00)
#define JY61p_CALSW (0x01)
#define JY61p_RSW (0x02)
#define JY61p_RRATE (0x03)
#define JY61p_BAUD (0x04)
#define JY61p_AXOFFSET (0x05)
#define JY61p_AYOFFSET (0x06)
#define JY61p_AZOFFSET (0x07)
#define JY61p_GXOFFSET (0x08)
#define JY61p_GYOFFSET (0x09)
#define JY61p_GZOFFSET (0x0a)
#define JY61p_HXOFFSET (0x0b)
#define JY61p_HYOFFSET (0x0c)
#define JY61p_HZOFFSET (0x0d)
#define JY61p_D0MODE (0x0e)
#define JY61p_D1MODE (0x0f)
#define JY61p_D2MODE (0x10)
#define JY61p_D3MODE (0x11)
#define JY61p_D0PWMH (0x12)
#define JY61p_D1PWMH (0x13)
#define JY61p_D2PWMH (0x14)
#define JY61p_D3PWMH (0x15)
#define JY61p_D0PWMT (0x16)
#define JY61p_D1PWMT (0x17)
#define JY61p_D2PWMT (0x18)
#define JY61p_D3PWMT (0x19)
#define JY61p_IICADDR (0x1a)
#define JY61p_LEDOFF (0x1b)
#define JY61p_GPSBAUD (0x1c)

#define JY61p_YYMM 0x30 // 年月
#define JY61p_DDHH 0x31 // 日时
#define JY61p_MMSS 0x32 // 分秒
#define JY61p_MS 0x33   // 毫秒
#define JY61p_AX 0x34   // 加速度
#define JY61p_AY 0x35
#define JY61p_AZ 0x36
#define JY61p_GX 0x37 // 角速度
#define JY61p_GY 0x38
#define JY61p_GZ 0x39
#define JY61p_HX 0x3a //
#define JY61p_HY 0x3b
#define JY61p_HZ 0x3c
#define JY61p_Roll 0x3d // 欧拉角
#define JY61p_Pitch 0x3e
#define JY61p_Yaw 0x3f
#define JY61p_TEMP 0x40 // 温度
#define JY61p_D0Status 0x41
#define JY61p_D1Status 0x42
#define JY61p_D2Status 0x43
#define JY61p_D3Status 0x44
#define JY61p_PressureL 0x45
#define JY61p_PressureH 0x46
#define JY61p_HeightL 0x47
#define JY61p_HeightH 0x48
#define JY61p_LonL 0x49
#define JY61p_LonH 0x4a
#define JY61p_LatL 0x4b
#define JY61p_LatH 0x4c
#define JY61p_GPSHeight 0x4d
#define JY61p_GPSYAW 0x4e
#define JY61p_GPSVL 0x4f
#define JY61p_GPSVH 0x50

#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5

typedef struct Time_Param {
  __IO u16 year;  // 年
  __IO u8 month;  // 月
  __IO u8 day;    // 日
  __IO u8 hour;   // 时
  __IO u8 minute; // 分
  __IO u8 sec;    // 秒
  __IO u16 Ms;    // 毫秒
} Time_Param;

typedef struct Acc_Param {
  __IO float Ax; // x轴 加速度
  __IO float Ay; // y
  __IO float Az; // z
} Acc_Param;

typedef struct Gyro_Param {
  __IO float Gx; // x轴 角速度
  __IO float Gy; // y
  __IO float Gz; // z
} Gyro_Param;

typedef struct EulerAngle_Param {
  __IO float Roll;  // 横滚角
  __IO float Pitch; // 俯仰角
  __IO float Yaw;   // 偏航角
} EulerAngle_Param;

/**** 函数 ****/
void JY61p_Init(void);
u8 JY61p_Check(void);
void JY61p_Get(Acc_Param *a, EulerAngle_Param *p, Gyro_Param *v);

// IIC时序
void JY61p_IIC_Init(void);
void JY61p_IIC_Start(void);
void JY61p_IIC_Stop(void);
u8 JY61p_IIC_SendAck(uint8_t AckBit);
u8 JY61p_IIC_WaitAck(void);
void JY61p_IIC_SendByte(uint8_t dat);
u8 JY61p_IIC_ReceiveByte(void);
u8 JY61p_Write(u8 addr, u8 reg, u8 len, u8 *buf);
u8 JY61p_Read(u8 addr, u8 reg, u8 len, u8 *buf);

#endif // __JY61P_H_