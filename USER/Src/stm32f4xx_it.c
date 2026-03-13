/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.8.1
 * @date    27-January-2022
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "sdio_sd.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
  /* 获取故障发生时的寄存器状态 */
  uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
  uint32_t fault_addr;
  uint32_t cfsr, hfsr, dfsr, afsr;

  /* 内联汇编获取寄存器值 */
  __asm volatile("TST lr, #4\n\t"
                 "ITE EQ\n\t"
                 "MRSEQ %0, MSP\n\t"
                 "MRSNE %0, PSP\n\t"
                 "MOV %1, %0\n\t"
                 : "=r"(r0), "=r"(r1)
                 :
                 : "memory");

  /* 从堆栈中提取寄存器值 */
  r0 = ((uint32_t *)r0)[0];
  r1 = ((uint32_t *)r0)[1];
  r2 = ((uint32_t *)r0)[2];
  r3 = ((uint32_t *)r0)[3];
  r12 = ((uint32_t *)r0)[4];
  lr = ((uint32_t *)r0)[5];
  pc = ((uint32_t *)r0)[6];
  psr = ((uint32_t *)r0)[7];

  /* 获取故障相关信息寄存器 */
  cfsr = SCB->CFSR; /* Configurable Fault Status Register */
  hfsr = SCB->HFSR; /* Hard Fault Status Register */
  dfsr = SCB->DFSR; /* Debug Fault Status Register */
  afsr = SCB->AFSR; /* Auxiliary Fault Status Register */

  /* 尝试获取MMAR和BFAR（如果有效） */
  fault_addr = 0xFFFFFFFF;
  if (SCB->CFSR & (1 << 7)) { /* MMFARVALID bit */
    fault_addr = SCB->MMFAR;
  } else if (SCB->CFSR & (1 << 15)) { /* BFARVALID bit */
    fault_addr = SCB->BFAR;
  }

  /* 通过串口输出故障信息 */
  printf("\r\n======= Hard Fault Occurred =======\r\n");
  printf("R0  = 0x%08X\r\n", r0);
  printf("R1  = 0x%08X\r\n", r1);
  printf("R2  = 0x%08X\r\n", r2);
  printf("R3  = 0x%08X\r\n", r3);
  printf("R12 = 0x%08X\r\n", r12);
  printf("LR  = 0x%08X\r\n", lr);
  printf("PC  = 0x%08X\r\n", pc);
  printf("PSR = 0x%08X\r\n", psr);
  printf("\r\n");
  printf("CFSR = 0x%08X\r\n", cfsr);
  printf("HFSR = 0x%08X\r\n", hfsr);
  printf("DFSR = 0x%08X\r\n", dfsr);
  printf("AFSR = 0x%08X\r\n", afsr);

  if (fault_addr != 0xFFFFFFFF) {
    printf("Fault Address = 0x%08X\r\n", fault_addr);
  }

  /* 解析具体错误类型 */
  printf("\r\n--- Fault Analysis ---\r\n");
  if (cfsr & 0x0001)
    printf("Instruction Misaligned Access\r\n");
  if (cfsr & 0x0002)
    printf("Data Misaligned Access\r\n");
  if (cfsr & 0x0004)
    printf("Invalid Instruction Execution\r\n");
  if (cfsr & 0x0008)
    printf("Invalid State Usage\r\n");
  if (cfsr & 0x0100)
    printf("Memory Management Fault\r\n");
  if (cfsr & 0x0200)
    printf("Bus Fault on Instruction Fetch\r\n");
  if (cfsr & 0x0400)
    printf("Bus Fault on Precise Data Access\r\n");
  if (cfsr & 0x0800)
    printf("Bus Fault on Imprecise Data Access\r\n");
  if (cfsr & 0x1000)
    printf("Undefined Instruction Usage\r\n");
  if (cfsr & 0x2000)
    printf("Invalid EPSR Usage\r\n");
  if (cfsr & 0x4000)
    printf("Invalid PC Load Usage\r\n");
  if (cfsr & 0x8000)
    printf("No Coprocessor Usage\r\n");

  if (hfsr & (1 << 30))
    printf("Forced Hard Fault\r\n");
  if (hfsr & (1 << 1))
    printf("Vector Table Read Fault\r\n");

  printf("===============================\r\n");

  /* 进入无限循环 */
  while (1) {
    /* 可以添加LED闪烁或其他指示 */
  }
}
/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void) {}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void) {}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) { Task_Handler(); }

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles K1 key interrupt request (PD2).
 * @param  None
 * @retval None
 */
void EXTI2_IRQHandler(void) {}

/**
 * @brief  This function handles K2 key interrupt request (PD0).
 * @param  None
 * @retval None
 */

/**
 * @brief  This function handles TIM4 interrupt request.
 * @param  None
 * @retval None
 */
uint8_t Led_flag = 0;
uint8_t DMA2_Flag;
uint8_t AWD_Flag;
void TIM3_IRQHandler(void) {
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
    // 清除中断标志
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
  }
}

/**
 * @brief  This function handles TIM4 interrupt request.
 * @param  None
 * @retval None
 */
void TIM4_IRQHandler(void) {}

// DMA传输错误标志（在主循环中处理）
volatile uint8_t DMA_Error_Flag = 0;

void DMA2_Stream0_IRQHandler(void) {
  // 传输完成中断
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    DMA2_Flag = 1;
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
    // DMA传输完成，重新使能看门狗中断，准备下一次触发
    // ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
  }

  // 传输错误中断 - 仅设置标志位（快进快出）
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TEIF0) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TEIF0);
    DMA_Error_Flag = 1; // 设置错误标志，在主循环中处理
  }
}

void ADC_IRQHandler(void) {
  // 检查模拟看门狗中断
  if (ADC_GetITStatus(ADC1, ADC_IT_AWD) != RESET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    AWD_Flag++;
    if (AWD_Flag == 3)
      ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_None);

    // LED闪烁提示看门狗触发
    // Led_flag = ~Led_flag;
    // GPIO_WriteBit(GPIOC, GPIO_Pin_13;, Led_flag ? Bit_SET : Bit_RESET);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_None);
    // 启动DMA传输，记录接下来的1024个采样点
    // ADC_DMA_Start();
  }
  // 检查转换结束中断（快进快出，不调用printf）
  if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    // 仅清除标志位，不进行耗时操作
    // Led_flag = ~Led_flag;
    // GPIO_WriteBit(GPIOB, GPIO_Pin_8, Led_flag);
  }
}
/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
//
/**
 * @brief  This function handles SDIO global interrupt request.
 * @param  None
 * @retval None
 */
void SDIO_IRQHandler(void) {
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/**
 * @brief  This function handles DMA2 Stream3 or DMA2 Stream6 global interrupts
 *         requests.
 * @param  None
 * @retval None
 */
void SD_SDIO_DMA_IRQHANDLER(void) {
  /* Process DMA2 Stream3 or DMA2 Stream6 Interrupt Sources */
  SD_ProcessDMAIRQ();
}
