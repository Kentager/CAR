

#ifndef __JY61P_H_
#define __JY61P_H_

/**** 头文件 ****/
#include "stm32f4xx.h"
#include "usart.h"
/**** 宏定义 ****/
typedef struct Time_Param {
  __IO u16 year;  // 年
  __IO u8 month;  // 月
  __IO u8 day;    // 日
  __IO u8 hour;   // 时
  __IO u8 minute; // 分
  __IO u8 sec;    // 秒
  __IO u16 Ms;    // 毫秒
} Time_Param;

/**** 函数 ****/
void JY61p_Init(void);

void JY61p_GetTime(Time_Param *p);
void JY61p_GetAcc(float *Ax, float *Ay, float *Az);
void JY61p_GetAngleSpeed(float *Wx, float *Wy, float *Wz);
void JY61p_GetAngle(float *Roll, float *Pitch, float *Yaw);
void JY61p_Get4Elements(float *Q0, float *Q1, float *Q2, float *Q3);

#endif // __JY61P_H_