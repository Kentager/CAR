#include "adc1.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include <stdint.h>
/*
使用TIM3
采样频率100KHz
采样间隔10us
adc转换时间1.9us（adcclk:21mhz,adc采样周期28,转换时间12）
1024点采集时间10.24
*/
static volatile uint16_t ADC_DMA_Buffer[ADC_DMA_BufferSize];

void ADC_GPIO_Init(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2对应ADC1通道2
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_DMA_Init(void) {
  // DMA配置，用于在看门狗触发后传输1024个数据
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream0);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr =
      (uint32_t)ADC_DMA_Buffer; // 需要在头文件或外部定义这个数组
  //  // 3. 配置双缓冲区模式
  //    Memory1BaseAddr: 第二个缓冲区地址
  //    DMA_CurrentMemory: DMA_Memory_0 或 DMA_Memory_1 (当前使用的内存)
  // DMA_DoubleBufferModeConfig(DMA2_Stream0,
  //                            (uint32_t)Buffer1,
  //                            DMA_Memory_0);
  //
  // 4. 使能双缓冲区模式
  // DMA_DoubleBufferModeCmd(DMA2_Stream0, ENABLE);
  DMA_InitStructure.DMA_MemoryDataSize =
      DMA_MemoryDataSize_HalfWord; // 存储器数据宽度：16位
  DMA_InitStructure.DMA_PeripheralDataSize =
      DMA_PeripheralDataSize_HalfWord;     // 外设数据宽度：16位
  DMA_InitStructure.DMA_BufferSize = 1024; // 缓冲区大小：1024个数据
  DMA_InitStructure.DMA_MemoryInc =
      DMA_MemoryInc_Enable; // 存储器地址递增：使能
  DMA_InitStructure.DMA_PeripheralInc =
      DMA_PeripheralInc_Disable; // 外设地址递增：禁用
  DMA_InitStructure.DMA_DIR =
      DMA_DIR_PeripheralToMemory; // 数据传输方向：外设到存储器
  DMA_InitStructure.DMA_Channel =
      DMA_Channel_0; // ADC1使用DMA2 Stream 0 Channel 0

  DMA_InitStructure.DMA_PeripheralBurst =
      DMA_PeripheralBurst_Single; // 外设突发传输：单次
  DMA_InitStructure.DMA_MemoryBurst =
      DMA_MemoryBurst_Single;                           // 存储器突发传输：单次
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; // FIFO模式：使能
  DMA_InitStructure.DMA_Mode =
      DMA_Mode_Circular; // 正常模式（传输1024次后停止）
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; // DMA优先级：高
  DMA_InitStructure.DMA_FIFOThreshold =
      DMA_FIFOThreshold_HalfFull; // FIFO阈值：半满
  DMA_InitStructure.DMA_MemoryBurst =
      DMA_MemoryBurst_Single; // 存储器突发：单次
  DMA_InitStructure.DMA_PeripheralBurst =
      DMA_PeripheralBurst_Single; // 外设突发：单次

  DMA_Init(DMA2_Stream0, &DMA_InitStructure); // 注意：ADC1应该使用DMA2_Stream0

  // 使能DMA传输完成中断
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

  // 注意：这里不使能DMA，等看门狗触发后再使能
  DMA_Cmd(DMA2_Stream0, ENABLE);
}
void ADC_TIM3_Init(void) {
  // 配置时基单元，使ADC1可以以100kHz的速度进行采样
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  // 配置100kHz触发频率：
  // TIM3时钟 = 84MHz (APB1 42MHz × 2)
  // 目标频率 = 100kHz
  // 计算：84MHz / (PSC+1) / (ARR+1) = 100kHz
  // 设置：PSC = 83, ARR = 9
  // 实际频率：84MHz / 84 / 10 = 100kHz

  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟不分频
  TIM_TimeBaseInitStructure.TIM_CounterMode =
      TIM_CounterMode_Up; // 向上计数模式
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter =
      0; // 重复计数器（基本定时器不使用）
  TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;    // 自动重装载值 ARR = 9
  TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1; // 预分频器值 PSC = 83

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

  // 使能TIM3的主模式输出，触发ADC
  TIM_SelectOutputTrigger(TIM3,
                          TIM_TRGOSource_Update); // 使用更新事件作为触发输出

  TIM_Cmd(TIM3, ENABLE); // 使能TIM3
}
void ADC1_NVIC_IT_Init(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // 恢复DMA中断配置
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // NVIC_InitStructure.NVIC_IRQChannel =  TIM3_IRQn;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void ADC1_Init(void) {

  ADC_GPIO_Init();
  // 恢复DMA初始化（但不使能DMA）
  ADC_DMA_Init();
  ADC_TIM3_Init();
  ADC1_NVIC_IT_Init();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  // ADC1根据TIM3的触发周期性地对GPIO_A2进行采样。
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // 单通道模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 非连续转换，由外部触发
  ADC_InitStructure.ADC_ExternalTrigConv =
      ADC_ExternalTrigConv_T3_TRGO; // TIM3触发
  ADC_InitStructure.ADC_ExternalTrigConvEdge =
      ADC_ExternalTrigConvEdge_Rising;                   // 上升沿触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率
  ADC_InitStructure.ADC_NbrOfConversion = 1;             // 1个转换通道

  ADC_Init(ADC1, &ADC_InitStructure);

  // 配置采样通道和采样周期
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_28Cycles);

  // 配置模拟看门狗
  ADC_AnalogWatchdogThresholdsConfig(ADC1, 500, 0x000);
  ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_2);
  ADC_AnalogWatchdogCmd(
      ADC1, ADC_AnalogWatchdog_SingleRegEnable); // 模拟看门狗监测通道使能

  ADC_DMACmd(ADC1, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  // 使能看门狗中断
  ADC_ITConfig(
      ADC1, ADC_IT_EOC,
      DISABLE); // 便于观测adc转换是否正确,注意dma开启的时候会屏蔽eoc中断
  ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
  // 最后使能ADC
  ADC_Cmd(ADC1, ENABLE);
}
/**
 * @brief  启动ADC DMA传输，在看门狗触发后调用
 * @param  None
 * @retval None
 */
void ADC_DMA_Start(void) {
  // 1. 禁用 DMA (直接操作寄存器)
  DMA2_Stream0->CR &= ~DMA_SxCR_EN;

  // 2. 等待 DMA 真正停止
  while (DMA2_Stream0->CR & DMA_SxCR_EN)
    ;

  // 3. 使能 ADC DMA 请求 (直接操作寄存器)
  ADC1->CR2 |= ADC_CR2_DMA;
  ADC1->CR2 |= ADC_CR2_DDS;
  // 4. 清除 DMA 中断标志 (直接操作 LIFCR 寄存器)
  // DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0;

  // 5. 重新配置传输计数 (直接操作 CNDTR 寄存器)
  DMA2_Stream0->NDTR = 1024;

  // 6. 使能 DMA (直接操作寄存器)
  DMA2_Stream0->CR |= DMA_SxCR_EN;
}
