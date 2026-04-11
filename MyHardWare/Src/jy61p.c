#include "jy61p.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

u8 JY61p_TimeData[9];       // 8位时间原始数据 1位完成位
u8 JY61p_AccData[9];        // 8位加速度原始数据 1位完成位
u8 JY61p_AngleSpeedData[9]; // 8位角速度原始数据 1位完成位
u8 JY61p_AngleData[9];      // 8位角度原始数据 1位完成位
u8 JY61p_4Elements[9];      // 8位四元素原始数据 1位完成位
void jy61p_usart_init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,
                         ENABLE); // 使能USART3时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  // USART3 TX PB10, RX PB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  USART_InitStructure.USART_BaudRate = 115200; // 波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 使能接收中断
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_Cmd(USART3, ENABLE); // 使能USART3
}
void JY61p_Init(void) {
  jy61p_usart_init(); // 使用自己的串口初始化波特率按陀螺仪配置的来
}

/***********************************************************
 *@fuction	: JY61p_GetPack
 *@brief		: JY61p 获取数据包
 *@param		: 数据
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_GetPack(u8 *DR) {
  static u8 sta = 0;     // 数据包进程
  static u8 p = 0;       // 数据帧
  static u8 type = 0xff; // 数据类型

  if (sta == 0) {
    if (*DR == 0x55) // 帧头
      sta = 1;
  } else if (sta == 1) {
    if (*DR == 0x50)
      type = 0, sta = 2; // 时间
    else if (*DR == 0x51)
      type = 1, sta = 2; // 加速度
    else if (*DR == 0x52)
      type = 2, sta = 2; // 角速度
    else if (*DR == 0x53)
      type = 3, sta = 2; // 角度
    else if (*DR == 0x59)
      type = 9, sta = 2; // 四元数
    else
      sta = 0;
  } else if (sta == 2) {
    switch (type) // 选择数据类型存储空间
    {
    /* 时间输入*/
    case 0:
      JY61p_TimeData[p++] = *DR;
      if (p == 7) {
        p = 0;
        type = 0xff;
        sta = 3;
        JY61p_TimeData[8] = 1; // 完成
      }
      break;
    /* 加速度输入*/
    case 1:
      JY61p_AccData[p++] = *DR;
      if (p == 7) {
        p = 0;
        type = 0;
        sta = 3;
        JY61p_AccData[8] = 1; // 完成
      }
      break;
    /* 角速度输入*/
    case 2:
      JY61p_AngleSpeedData[p++] = *DR;
      if (p == 7) {
        p = 0;
        type = 0;
        sta = 3;
        JY61p_AngleSpeedData[8] = 1; // 完成
      }
      break;
    /* 角度输入*/
    case 3:
      JY61p_AngleData[p++] = *DR;
      if (p == 7) {
        p = 0;
        type = 0;
        sta = 3;
        JY61p_AngleData[8] = 1; // 完成
      }
      break;
      /* 四元数*/
    case 9:
      JY61p_4Elements[p++] = *DR;
      if (p == 7) {
        p = 0;
        type = 0;
        sta = 3;
        JY61p_4Elements[8] = 1; // 完成
      }
      break;
    default:
      sta = 0;
    }
  } else if (sta == 3) {
    //        if(*DR != SUM) // 校验
    //        {
    //
    //        }
    sta = 0;
  }
}
/**
 * @brief  USART3 的中断服务函数
 */
void USART3_IRQHandler(void) {
  if (USART_GetITStatus(USART3, USART_IT_RXNE)) {
    /* 获取数据流*/
    JY61p_GetPack((u8 *)&(USART3->DR));

    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
}

/***********************************************************
 *@fuction	: JY61p_GetTime
 *@brief		: JY61p 获取时间
 *@param		: 时间
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_GetTime(Time_Param *p) {
  if (JY61p_TimeData[8]) {
    p->year = JY61p_TimeData[0] + 2000;                 // 年
    p->month = JY61p_TimeData[1];                       // 月
    p->day = JY61p_TimeData[2];                         // 日
    p->hour = JY61p_TimeData[3];                        // 时
    p->minute = JY61p_TimeData[4];                      // 分
    p->sec = JY61p_TimeData[5];                         // 秒
    p->Ms = JY61p_TimeData[7] << 8 | JY61p_TimeData[6]; // 毫秒
    JY61p_TimeData[8] = 0;
  }
}

/***********************************************************
 *@fuction	: JY61p_GetAcc
 *@brief		: JY61p 获取加速度, 范围: 0.00 m/s^2 ~ 16.00g m/s^2
 *@param		: 加速度x\y\z方向, 温度
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_GetAcc(float *Ax, float *Ay, float *Az) {
  if (JY61p_AccData[8]) {
    *Ax = ((JY61p_AccData[1] << 8) | JY61p_AccData[0]) / 32768.00f * 16 *
          9.8f; // x加速度
    *Ay = ((JY61p_AccData[3] << 8) | JY61p_AccData[2]) / 32768.00f * 16 *
          9.8f; // y加速度
    *Az = ((JY61p_AccData[5] << 8) | JY61p_AccData[4]) / 32768.00f * 16 *
          9.8f; // z加速度
    //        *T  = ((JY61p_AccData[7]<<8) | JY61p_AccData[6]) /100.00f;   //
    //        温度
    JY61p_AccData[8] = 0;
  }
}

/***********************************************************
 *@fuction	: JY61p_GetAngleSpeed
 *@brief		: JY61p 获取角加速度 范围: 0.00度/s ~ 2000.00度/s
 *@param		: 角加速度x\y\z方向, 温度
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_GetAngleSpeed(float *Wx, float *Wy, float *Wz) {
  if (JY61p_AngleSpeedData[8]) {
    *Wx = ((JY61p_AngleSpeedData[1] << 8) | JY61p_AngleSpeedData[0]) /
          32768.00f * 2000; // x加速度
    *Wy = ((JY61p_AngleSpeedData[3] << 8) | JY61p_AngleSpeedData[2]) /
          32768.00f * 2000; // y加速度
    *Wz = ((JY61p_AngleSpeedData[5] << 8) | JY61p_AngleSpeedData[4]) /
          32768.00f * 2000; // z加速度
    //        *T  = ((JY61p_AngleSpeedData[7]<<8) | JY61p_AngleSpeedData[6])
    //        /100.00f;   // 温度
    JY61p_AngleSpeedData[8] = 0;
  }
}

/***********************************************************
 *@fuction	: JY61p_GetAngle
 *@brief		: JY61p 获取角度值, 范围: -180.00°~ 180.00°
 *@param		: 滚转角, 俯仰角, 偏航角, 温度
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_GetAngle(float *Roll, float *Pitch, float *Yaw) {
  if (JY61p_AngleData[8]) {
    *Roll = ((JY61p_AngleData[1] << 8) | JY61p_AngleData[0]) / 32768.00f *
            180; // 滚转角 x
    *Pitch = ((JY61p_AngleData[3] << 8) | JY61p_AngleData[2]) / 32768.00f *
             180; // 俯仰角 y
    *Yaw = ((JY61p_AngleData[5] << 8) | JY61p_AngleData[4]) / 32768.00f *
           180; // 偏航角 z
    //        *Temp  = ((JY61p_AngleData[7]<<8) | JY61p_AngleData[6]) /100.00f;
    //        // 温度
    if (*Roll > 180)
      *Roll -= 360.00f;
    if (*Pitch > 180)
      *Pitch -= 360.00f;
    if (*Yaw > 180)
      *Yaw -= 360.00f;

    *Roll = -*Roll;
    *Pitch = -*Pitch;
    *Yaw = -*Yaw;

    JY61p_AngleData[8] = 0;
  }
}

/***********************************************************
 *@fuction	: JY61p_Get4Elements
 *@brief		: JY61p 获取四元数, 范围: 0~1, 精度0.00
 *@param		: 四元数
 *@return	: None
 *@author	: HongScholar
 *@date		: 2025.03.22
 ***********************************************************/
void JY61p_Get4Elements(float *Q0, float *Q1, float *Q2, float *Q3) {
  if (JY61p_4Elements[8]) {
    *Q0 = ((JY61p_4Elements[1] << 8) | JY61p_4Elements[0]) / 32768.00f;
    *Q1 = ((JY61p_4Elements[3] << 8) | JY61p_4Elements[2]) / 32768.00f;
    *Q2 = ((JY61p_4Elements[5] << 8) | JY61p_4Elements[4]) / 32768.00f;
    *Q3 = ((JY61p_4Elements[7] << 8) | JY61p_4Elements[6]) / 32768.00f;
    JY61p_4Elements[8] = 0;
  }
}