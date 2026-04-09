#ifndef __KEY_H
#define __KEY_H

#include "delay.h"
#include "stm32f4xx.h"

#define KEY_ON 1  // 按键按下
#define KEY_OFF 0 // 按键放开

/*---------------------- 按键配置宏 ------------------------*/

#define KEY_PIN (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3) // KEY 引脚

#define KEY_PORT GPIOC               // KEY GPIO端口
#define KEY_CLK RCC_AHB1Periph_GPIOC // KEY GPIO端口时钟

/*---------------------- LED和蜂鸣器配置宏 ------------------------*/

#define LED_PIN GPIO_Pin_6 // LED 引脚

#define LED_PORT GPIOE               // LED GPIO端口
#define LED_CLK RCC_AHB1Periph_GPIOE // LED GPIO端口时钟

/*---------------------- 函数声明 ----------------------------*/

void KEY_Init(void);    // 按键IO口初始化函数
uint8_t KEY_Scan(void); // 按键扫描

#endif //__KEY_H