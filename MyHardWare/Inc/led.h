#ifndef __LED_H
#define __LED_H
#include "stm32f4xx.h"

// LED 引脚定义
#define LED_PIN GPIO_Pin_13
#define LED_PORT GPIOC
#define LED_CLK RCC_AHB1Periph_GPIOC

void led_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

#endif // !__LED_H