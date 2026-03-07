#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"

typedef struct
{
	float SetSpeed;//设定值
	float ActualSpeed;//实际值
	float err;
	float err_last;
	float Kp;
	float Ki;
	float Kd;
	float out;//执行器的变量
	float integral;//积分值
}pid_typedef;

void PID_init(void);
float PID_realize(float ActualSpeed);


#endif
