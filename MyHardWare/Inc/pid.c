#include "pid.h"

pid_typedef pid;

void PID_init(void)

{
   
    pid.SetSpeed=50;
    pid.ActualSpeed=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.out=0.0;
    pid.integral=0.0;
    pid.Kp=0.125;
    pid.Ki=1.70;
    pid.Kd=0.523;
  
}



float PID_realize(float ActualSpeed)
{
	pid.ActualSpeed=ActualSpeed;
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    pid.integral+=pid.err;
//	if(pid.integral>3000)pid.integral=3000;
//	if(pid.integral<0)pid.integral=0;
    pid.out=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	if(pid.out<0)pid.out=0;
	if(pid.out>10000)pid.out=10000;
    pid.err_last=pid.err;

    return pid.out;
}

