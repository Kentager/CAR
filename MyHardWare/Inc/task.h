#ifndef __TASK_H
#define __TASK_H

#include "stm32f4xx.h"

// 函数声明
void Task_Init(void);
int add_task(void (*task_func)(void), uint32_t period_ms);
void remove_task(uint8_t task_id);
void start_task(uint8_t task_id);
void stop_task(uint8_t task_id);
void Task_Handler(void);
uint32_t GetSysTick(void);
void tdelay_ms(uint32_t ms);

#endif /* __TASK_H */