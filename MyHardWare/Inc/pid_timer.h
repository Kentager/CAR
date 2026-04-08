#ifndef __PID_TIMER_H
#define __PID_TIMER_H
#include "stm32f4xx.h"
#include <stdint.h>

// 外部变量声明
extern volatile uint8_t pid_timer_flag;    // 定时器中断标志
extern volatile uint32_t pid_timer_count;  // 定时器中断计数

// 函数声明
void TIM3_5ms_Init(void);      // 初始化TIM3 5ms定时器
void TIM3_5ms_Start(void);     // 启动定时器
void TIM3_5ms_Stop(void);      // 停止定时器
uint8_t TIM3_5ms_GetFlag(void);    // 获取中断标志
uint32_t TIM3_5ms_GetCount(void);  // 获取中断计数

#endif