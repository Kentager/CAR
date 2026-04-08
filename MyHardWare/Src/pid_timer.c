#include "pid_timer.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

// 全局中断标志，供主循环检测
volatile uint8_t pid_timer_flag = 0;
volatile uint32_t pid_timer_count = 0;

/**
 * @brief TIM3 5ms定时器初始化
 * @note  配置TIM3为5ms周期中断
 *
 * 定时器配置计算：
 * - TIM3时钟 = 84MHz (APB1 42MHz × 2)
 * - 目标周期 = 5ms = 0.005s
 * - 目标频率 = 200Hz
 * - 计算：84MHz / (PSC+1) / (ARR+1) = 200Hz
 * - 设置：PSC = 839, ARR = 499
 * - 实际频率：84MHz / 840 / 500 = 200Hz (5ms)
 *
 * @warning 此配置会与adc1.c中的ADC TIM3配置冲突，请根据实际需求选择
 */
void TIM3_5ms_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 使能TIM3时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  // 定时器时基配置
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // 时钟不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // 向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;              // 重复计数器
  TIM_TimeBaseStructure.TIM_Period = 500 - 1;                   // 自动重装载值 ARR = 499
  TIM_TimeBaseStructure.TIM_Prescaler = 840 - 1;                // 预分频器值 PSC = 839

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // 清除更新标志位
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);

  // 使能TIM3更新中断
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  // 配置TIM3中断优先级
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;       // 响应优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // 使能TIM3
  TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief 启动TIM3定时器
 */
void TIM3_5ms_Start(void) {
  TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief 停止TIM3定时器
 */
void TIM3_5ms_Stop(void) {
  TIM_Cmd(TIM3, DISABLE);
}

/**
 * @brief 获取定时器中断标志（查询方式）
 * @retval 1: 发生过中断，0: 未发生中断
 */
uint8_t TIM3_5ms_GetFlag(void) {
  if (pid_timer_flag) {
    pid_timer_flag = 0;  // 清除标志
    return 1;
  }
  return 0;
}

/**
 * @brief 获取定时器中断计数
 * @retval 中断次数
 */
uint32_t TIM3_5ms_GetCount(void) {
  return pid_timer_count;
}
