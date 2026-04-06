/**
 ******************************************************************************
 * @file    tim4_delay.c
 * @brief   TIM4微秒级延时函数实现
 ******************************************************************************
 */

#include "delay.h"

/**
 * @brief  初始化TIM4用于微秒级延时
 * @param  None
 * @retval None
 */
void delay_init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  /* 使能TIM4时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* TIM4配置 */
  /* 系统时钟为72MHz，APB1时钟为36MHz，TIM4时钟为72MHz */
  /* 预分频器设置为72-1 = 71，这样定时器时钟为1MHz（1us计数一次） */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; /* 自动重装载值设为最大 */
  TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  /* 72MHz/72 = 1MHz */
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

  /* 使能TIM4 */
  TIM_Cmd(TIMx, ENABLE);
}

/**
 * @brief  微秒级延时函数
 * @param  us: 延时的微秒数 (最大延时约4294秒)
 * @retval None
 */
void delay_us(uint32_t us) {
  uint32_t start_counter, elapsed;
  uint32_t tick;

  /* 如果延时时间小于10us，使用简单的循环延时 */
  if (us < 10) {
    /* 72MHz时钟下，每个while循环约4个时钟周期 */
    volatile uint32_t count = us * 18; /* 72MHz/4 = 18次循环/微秒 */
    while (count--)
      ;
    return;
  }

  /* 获取当前计数器值 */
  start_counter = TIM_GetCounter(TIMx);
  tick = start_counter;

  do {
    uint32_t current = TIM_GetCounter(TIMx);

    /* 处理计数器溢出 */
    if (current >= tick) {
      elapsed = current - start_counter;
    } else {
      /* 计数器溢出 */
      elapsed = (0xFFFFFFFF - start_counter) + current + 1;
    }

    tick = current;
  } while (elapsed < us);
}

/**
 * @brief  毫秒级延时函数（基于TIM4）
 * @param  ms: 延时的毫秒数
 * @retval None
 */
void delay_ms(uint32_t ms) {
  while (ms--) {
    delay_us(1000); /* 1ms = 1000us */
  }
}