#ifndef __USART_H
#define __USART_H

#include "ring_buffer.h"
#include "stdio.h"
#include "stm32f4xx.h"

/*----------------------USART 配置宏 ------------------------*/

#define USART1_BaudRate 115200

#define USART1_TX_PIN GPIO_Pin_9            // TX 引脚
#define USART1_TX_PORT GPIOA                // TX 引脚端口
#define USART1_TX_CLK RCC_AHB1Periph_GPIOA  // TX 引脚时钟
#define USART1_TX_PinSource GPIO_PinSource9 // 引脚源

#define USART1_RX_PIN GPIO_Pin_10            // RX 引脚
#define USART1_RX_PORT GPIOA                 // RX 引脚端口
#define USART1_RX_CLK RCC_AHB1Periph_GPIOA   // RX 引脚时钟
#define USART1_RX_PinSource GPIO_PinSource10 // 引脚源

/*---------------------- DMA 缓冲区配置 ------------------------*/
#define USART_DMA_TX_BUFFER_SIZE 256
#define USART_DMA_RX_BUFFER_SIZE 256

/*---------------------- 循环队列结构体 ------------------------*/
typedef struct {
  uint8_t *buffer;
  uint16_t size;
  uint16_t head;
  uint16_t tail;
  uint8_t is_dma_enabled;
} CircularQueue;

/*---------------------- 函数声明 ----------------------------*/

void Usart_Config(void);
void UART1_SendData(uint16_t *buffer, uint16_t size);
void UART1_SendByte(uint8_t data);

void UART1_DMA_Start_RX(void);
uint16_t UART1_ReadAvailable_DMA(void);

void USART1_Init(void);
extern RingBuffer tx_queue;
extern RingBuffer rx_queue;
#endif //__USART_H