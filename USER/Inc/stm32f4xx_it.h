/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h
 * @author  MCD Application Team
 * @version V1.8.1
 * @date    27-January-2022
 * @brief   This file contains the headers of the interrupt handlers.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern volatile uint8_t Key_Pressed_Flag_0;
extern volatile uint8_t Key_Pressed_Flag_1;
extern volatile uint8_t Key_Pressed_Flag_2;
extern volatile uint8_t Key_Pressed_Flag_3;
extern volatile uint8_t count_it;
void KEY_Interrupt_Init(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* 外部中断处理函数 */
void EXTI2_IRQHandler(void);        // K1按键 (PD2)
void EXTI0_IRQHandler(void);        // K2按键 (PD0)
void TIM3_IRQHandler(void);         // 1ms定时器
void DMA2_Stream0_IRQHandler(void); // ADC DMA中断
void ADC_IRQnHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */