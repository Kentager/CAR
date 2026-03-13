#ifndef __TASK_H
#define __TASK_H

#include "stm32f4xx.h"

// 任务状态定义
typedef enum {
  TASK_STATE_IDLE,    // 空闲态（未激活）
  TASK_STATE_READY,   // 就绪态（等待执行）
  TASK_STATE_RUNNING  // 运行态（正在执行）
} task_state_t;

// 任务结构体定义
typedef struct {
  void (*task_func)(void);    // 任务函数指针
  uint32_t period_ms;         // 任务执行周期（毫秒）
  uint32_t last_run_tick;     // 上次运行时的tick值
  task_state_t state;         // 任务状态
  uint8_t active;             // 任务是否激活
} task_item_t;

// 函数声明
void Task_Init(void);
int add_task(void (*task_func)(void), uint32_t period_ms);
void remove_task(uint8_t task_id);
void start_task(uint8_t task_id);
void stop_task(uint8_t task_id);
void Task_Handler(void);      // SysTick中断调用，仅标记就绪态
void Task_Scheduler(void);    // 主循环调用，执行就绪态任务
uint32_t GetSysTick(void);
void tdelay_ms(uint32_t ms);

#endif /* __TASK_H */