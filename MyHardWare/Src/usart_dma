/***
        *************************************************************************************************
        *	@version V1.0
        *  @date    2026-3-8
        *	@author  反客科技
        *	@brief   usart 接口相关函数
   *************************************************************************************************
>>>>> 文件说明：
        *
        *  1.初始化 USART1 的引脚 PA9/PA10，
        *  2.配置 USART1 工作在收发模式、数位 8 位、停止位
            1位、无校验、不使用硬件控制流控制，
        *
串口的波特率设置为 115200，若需要更改波特率直接修改 usart.h 里的宏定义
USART1_BaudRate。
        *  3.重定义 fputc 函数，用以支持使用 printf 函数打印数据
        *  4.使用 TIM7 基本定时器每 100us 检查环形缓冲队列，实现智能 DMA
传输控制
        *
        ************************************************************************************************
***/

#include "usart.h"
#include "delay.h"
#include "led.h"
#include "oled.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include <stdint.h>
#include <stdio.h>

// DMA 发送缓冲区，大小为 256 字节
static uint8_t dma_tx_buffer[USART_DMA_TX_BUFFER_SIZE];
// DMA 接收缓冲区，大小为 256 字节
static uint8_t dma_rx_buffer[USART_DMA_RX_BUFFER_SIZE];
// 发送环形缓冲区，用于存储待发送的数据
RingBuffer tx_queue;
// 接收环形缓冲区，用于存储接收到的数据
RingBuffer rx_queue;

// DMA 发送完成标志位
volatile uint8_t dma_tx_transfer_complete = 0;
// DMA 接收完成标志位
volatile uint8_t dma_rx_transfer_complete = 0;
// 记录上次 DMA 传输的数据量
static uint16_t last_dma_tx_count = 0;
// 函数声明
static void USART1_DMA_Init(void);
static void TIM7_Config(void);
static void TIM7_EnableIRQ(void);
static void TIM7_DisableIRQ(void);

static void USART_StartDMA_TX(void);
/**
 * @brief  USART1 DMA 初始化配置
 *         配置 DMA2 Stream7 用于 USART1 TX，DMA2 Stream2 用于 USART1 RX
 *         使用 DMA Channel4，字节传输模式，使能中断
 */
static void USART1_DMA_Init(void) {
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 使能 DMA2 时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // 配置 DMA2 Stream2 (RX) 的中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // 配置 DMA2 Stream7 (TX) 的中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // DMA TX 配置 (DMA2 Stream7, Channel4) - 内存到外设
  DMA_DeInit(DMA2_Stream7);
  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
    ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);

  // DMA RX 配置 (DMA2 Stream2, Channel4) - 外设为内存
  DMA_DeInit(DMA2_Stream2);
  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
    ;
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);

  // 使能 DMA2 Stream2 (RX) 的传输完成中断
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

  // 使能 DMA 通道
  DMA_Cmd(DMA2_Stream7, DISABLE);
  DMA_Cmd(DMA2_Stream2, DISABLE);
  // DMA_Cmd(DMA2_Stream7, ENABLE);
  // DMA_Cmd(DMA2_Stream2, ENABLE);
}

//=============================================================================
// NOTE:TIM7定时器是不需要的，在printf去看dma是否完成，为完成调用它即可
//=============================================================================

/**
 * @brief  TIM7 定时器配置，定时 100us
 *         STM32F407 系统时钟为 168MHz，APB1 时钟为 42MHz
 *         prescaler = 4200 - 1, 则计数器频率 = 42MHz / 4200 = 10kHz (周期 0.1ms
 * = 100us) period = 1 - 1, 则溢出时间 = 10kHz / 1 = 10kHz，即 100us
 *         用于定期检查环形缓冲队列，有数据时启动 DMA 传输
 */
/*
static void TIM7_Config(void) {
 TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
 NVIC_InitTypeDef NVIC_InitStructure;

 // 使能 TIM7 时钟
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

 // 配置 TIM7 时基参数
 TIM_TimeBaseInitStruct.TIM_Prescaler =
     420 - 1; // 预分频器，42MHz / 4200 = 10kHz
 TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInitStruct.TIM_Period =
     10 - 1; // 自动重装载值，10kHz / 1 = 10kHz (100us)
 TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
 TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);

 // 清除更新中断标志，防止立即触发中断
 TIM_ClearFlag(TIM7, TIM_FLAG_Update);

 // 使能更新中断
 TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

 // 配置 NVIC 中断优先级
 NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
}
 */
/**
 * @brief  USART1 GPIO 引脚配置
 *         配置 PA9 为 TX，PA10 为 RX，复用模式，推挽输出，上拉输入
 */
void USART_GPIO_Config(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // 使能 GPIOA 时钟
  RCC_AHB1PeriphClockCmd(USART1_TX_CLK | USART1_RX_CLK, ENABLE);

  // IO 配置参数
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // 复用模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // 推挽输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // 上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // 速度等级 2MHz

  // 初始化 TX 引脚 (PA9)
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
  GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);

  // 初始化 RX 引脚 (PA10)
  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
  GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);

  // IO 复用，将 PA9/PA10 复用到 USART1
  GPIO_PinAFConfig(USART1_TX_PORT, USART1_TX_PinSource, GPIO_AF_USART1);
  GPIO_PinAFConfig(USART1_RX_PORT, USART1_RX_PinSource, GPIO_AF_USART1);
}

/**
 * @brief  USART1 串口参数配置
 *         波特率 115200，8 位数据位，1 位停止位，无校验，无硬件流控制
 */
void Usart_Config(void) {
  USART_InitTypeDef USART_InitStructure;

  // 使能 USART1 时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  // IO 口初始化
  USART_GPIO_Config();

  // 配置串口各项参数
  USART_InitStructure.USART_BaudRate = USART1_BaudRate; // 波特率 115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 数据位 8 位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 停止位 1 位
  USART_InitStructure.USART_Parity = USART_Parity_No;         // 无校验
  USART_InitStructure.USART_Mode =
      USART_Mode_Rx | USART_Mode_Tx; // 发送和接收模式
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None; // 不使用硬件流控制
  // 配置 USART1 使用 DMA
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

  USART_Init(USART1, &USART_InitStructure); // 初始化串口 1
  USART_Cmd(USART1, ENABLE);                // 使能串口 1
}

/**
 * @brief  初始化 USART1 及 TIM7
 *         需要在 main 函数中调用此函数来完成 USART1 和 TIM7 的初始化
 *         包括：DMA 初始化、串口配置、TIM7 配置、启动 DMA 接收、使能 TIM7 中断
 */
void USART1_Init(void) {
  // 初始化发送和接收环形缓冲区
  RingBuffer_Init(&tx_queue, dma_tx_buffer, USART_DMA_TX_BUFFER_SIZE);
  RingBuffer_Init(&rx_queue, dma_rx_buffer, USART_DMA_RX_BUFFER_SIZE);

  // 初始化 USART1 DMA
  USART1_DMA_Init();

  // 配置 USART1 串口参数和 GPIO
  Usart_Config();

  // 启动 DMA 接收
  UART1_DMA_Start_RX();
}

/**
 * @brief  UART1 发送一个字节（轮询方式）
 * @param  data: 要发送的数据
 * @retval None
 */
void UART1_SendByte(uint8_t data) {
  /* 等待发送缓冲区为空 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    ;

  /* 强制转换为 16 位并发送数据，确保正确的数据位传输 */
  USART_SendData(USART1, (uint16_t)data);
}

/**
 * @brief  重定义 fputc 函数，支持 printf 打印
 *         通过环形缓冲区 + DMA 方式发送字符，无需 CPU 等待
 * @param  ch: 要发送的字符
 * @retval 返回发送的字符
 */
/**/
int __io_putchar(int ch) {
  // 尝试将数据写入环形缓冲区
  uint16_t count = 0;
  while (RingBuffer_Push(&tx_queue, (uint8_t)ch) == 0) {
    // 缓冲区满，等待有空闲空间
    delay_us(1); // 等待 100 微秒
    if ((count++) >= 1000) {
      break; // 等待超过 1ms，放弃发送
    }
    // RingBuffer_Push 会在有空间时自动写入并返回 1
  }

  // 如果 DMA 未在工作，触发 TIM7 中断来启动 DMA 传输
  if (!tx_queue.is_dma_enabled && !RingBuffer_IsEmpty(&tx_queue)) {
    USART_StartDMA_TX();
  }

  return (ch);
}

/**
 * @brief  UART1 发送多个 16 位数据（轮询方式）
 *         每个 16 位数据先发送高字节，再发送低字节
 * @param  buffer: 16 位数据缓冲区指针
 * @param  size: 数据个数
 */
void UART1_SendData(uint16_t *buffer, uint16_t size) {
  for (uint16_t i = 0; i < size; i++) {
    uint16_t data = buffer[i]; // 获取当前 16 位数据

    // 发送高字节（高位在前）
    USART_SendData(USART1, (data >> 8) & 0xFF);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      ; // 等待发送完成

    // 发送低字节
    USART_SendData(USART1, data & 0xFF);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      ; // 等待发送完成
  }
}

/**
 * @brief  启动 UART1 DMA 接收
 *         配置 DMA2 Stream5 将 USART1 DR 寄存器的数据传输到 rx_queue 缓冲区
 */
void UART1_DMA_Start_RX(void) {
  DMA2_Stream2->M0AR = (uint32_t)rx_queue.buffer;
  DMA2_Stream2->NDTR = rx_queue.size;
  DMA_Cmd(DMA2_Stream2, ENABLE);
  rx_queue.is_dma_enabled = 1;
}

/**
 * @brief  启动 USART DMA 发送
 *         从 tx_queue 中读取数据，通过 DMA2 Stream7 发送到 USART1
 *         自动计算连续可读数据长度，支持环形缓冲区的循环特性
 */
static void USART_StartDMA_TX(void) {
  // 如果队列为空，不启动 DMA
  if (tx_queue.head == tx_queue.tail) {
    return;
  }

  uint16_t data_count;
  // 计算从 tail 到 head 的连续可读数据长度
  if (tx_queue.head > tx_queue.tail) {
    // 情况 1：数据未跨越末尾，直接传输 tail 到 head
    data_count = tx_queue.head - tx_queue.tail;
  } else {
    // 情况 2：数据跨越末尾，只传输从 tail 到缓冲区末尾的部分
    // 这是 DMA 能安全访问的连续内存区域
    data_count = tx_queue.size - tx_queue.tail;
  }

  // 配置 DMA 源地址和数据长度
  DMA2_Stream7->M0AR = (uint32_t)&tx_queue.buffer[tx_queue.tail];
  DMA_SetCurrDataCounter(DMA2_Stream7, data_count);

  // 保存本次 DMA 传输的数据量，用于中断中计算
  last_dma_tx_count = data_count;

  DMA_Cmd(DMA2_Stream7, ENABLE);
  tx_queue.is_dma_enabled = 1;
}

/**
 * @brief  DMA2 Stream5 中断服务函数（USART1 RX）
 *         处理 DMA 接收完成中断，重新启动 DMA 接收
 */
void DMA2_Stream2_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
    dma_rx_transfer_complete = 1;
    DMA_Cmd(DMA2_Stream5, DISABLE);
    UART1_DMA_Start_RX();
  }
}

/**
 * @brief  DMA2 Stream7 中断服务函数（USART1 TX）
 *         处理 DMA 发送完成中断，更新读指针，根据队列状态决定是否继续发送
 */

void DMA2_Stream7_IRQHandler(void) {
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET) {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
    dma_tx_transfer_complete = 1;
    DMA_Cmd(DMA2_Stream7, DISABLE);

    // 计算已传输的数据长度 = 初始值 - 剩余值
    // DMA 传输完成后 NDTR 应该为 0，但为了保险起见还是计算一下
    uint16_t transferred = last_dma_tx_count - DMA2_Stream7->NDTR;

    // 跳过已读取的数据（移动 tail 指针）
    // 注意：这里只是移动逻辑指针，不会清除数据
    RingBuffer_SkipRead(&tx_queue, transferred);
    tx_queue.is_dma_enabled = 0;

    // 关键：重新检查队列状态，而不是简单地判断是否为空
    // 因为在 DMA 传输期间，printf 可能又写入了新数据
    if (!RingBuffer_IsEmpty(&tx_queue)) {
      // 队列还有数据，立即启动下一段 DMA 传输
      // 这会自动处理以下情况：
      // 1. 之前是分段传输的第二段（tail 已回绕到 0）
      // 2. DMA 传输期间 printf 新写入的数据
      USART_StartDMA_TX();
    }
  }
}
/**
 * @brief  获取 UART1 可读取的数据长度（DMA 接收模式）
 * @retval 环形缓冲区中可用的数据字节数
 */
uint16_t UART1_ReadAvailable_DMA(void) {
  return RingBuffer_Available(&rx_queue);
}
