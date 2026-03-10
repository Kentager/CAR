#include "led.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void led_Init(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 初始化时默认熄灭 LED
  GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/**
 * @brief  点亮 LED
 *         PC13 引脚输出低电平，LED 导通点亮
 */
void LED_On(void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); }

/**
 * @brief  熄灭 LED
 *         PC13 引脚输出高电平，LED 截止熄灭
 */
void LED_Off(void) { GPIO_SetBits(GPIOC, GPIO_Pin_13); }

/**
 * @brief  翻转 LED 状态
 *         如果当前是点亮状态则熄灭，如果当前是熄灭状态则点亮
 */
void LED_Toggle(void) { GPIOC->ODR ^= GPIO_Pin_13; }