#include "task.h"
#include <stddef.h>
// 系统节拍计数器
static volatile uint32_t sysTickCounter = 0;

// 任务结构体定义
typedef struct {
  void (*task_func)(void); // 任务函数指针
  uint32_t period_ms;      // 任务执行周期（毫秒）
  uint32_t last_run_tick;  // 上次运行时的tick值
  uint8_t active;          // 任务是否激活
} task_item_t;

// 定义最大任务数量
#define MAX_TASKS 10

// 任务池
static task_item_t task_pool[MAX_TASKS];
static uint8_t task_count = 0;

// SysTick初始化函数
void Task_Init(void) {
  // 初始化任务池
  for (int i = 0; i < MAX_TASKS; i++) {
    task_pool[i].task_func = NULL;
    task_pool[i].period_ms = 0;
    task_pool[i].last_run_tick = 0;
    task_pool[i].active = 0;
  }

  // 配置SysTick为1ms中断（基于系统时钟频率）
  if (SysTick_Config(SystemCoreClock / 1000)) {
    while (1)
      ; // 初始化失败处理
  }
}

// 添加任务到调度器
int add_task(void (*task_func)(void), uint32_t period_ms) {
  if (task_count >= MAX_TASKS) {
    return -1; // 任务池已满
  }

  task_pool[task_count].task_func = task_func;
  task_pool[task_count].period_ms = period_ms;
  task_pool[task_count].last_run_tick = 0;
  task_pool[task_count].active = 1;
  task_count++;

  return task_count - 1; // 返回任务ID
}

// 移除任务
void remove_task(uint8_t task_id) {
  if (task_id < task_count && task_pool[task_id].active) {
    task_pool[task_id].active = 0;
    task_pool[task_id].task_func = NULL;
  }
}

// 启动任务
void start_task(uint8_t task_id) {
  if (task_id < task_count) {
    task_pool[task_id].active = 1;
  }
}

// 停止任务
void stop_task(uint8_t task_id) {
  if (task_id < task_count) {
    task_pool[task_id].active = 0;
  }
}

// SysTick中断服务函数 - 实际上是系统节拍处理函数
void Task_Handler(void) {
  sysTickCounter++; // 每毫秒递增计数器

  // 检查并执行到期的任务
  for (int i = 0; i < task_count; i++) {
    if (task_pool[i].active && task_pool[i].task_func != NULL &&
        (sysTickCounter - task_pool[i].last_run_tick >=
         task_pool[i].period_ms)) {
      task_pool[i].task_func();                    // 执行任务
      task_pool[i].last_run_tick = sysTickCounter; // 更新最后执行时间
    }
  }
}

// 获取系统节拍值
uint32_t GetSysTick(void) {
  uint32_t tick;
  __disable_irq();       // 禁止中断以确保原子操作
  tick = sysTickCounter; // 读取当前计数值
  __enable_irq();        // 恢复中断
  return tick;
}

// 简单延时函数（基于系统节拍）
void tdelay_ms(uint32_t ms) {
  uint32_t start_tick = sysTickCounter;
  while ((sysTickCounter - start_tick) < ms) {
    // 可以在此处添加低功耗模式以节省能源
  }
}