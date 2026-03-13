/**
 * @file uart.h
 * @brief UART1驱动头文件 - 串口数据传输
 * @version 1.0
 * @date 2025-12-15
 */

#ifndef __UART_H
#define __UART_H

#include "stm32f4xx.h"

/* UART1配置宏定义 */
#define UART1_GPIO_PORT GPIOA
#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO_CLK RCC_AHB1Periph_GPIOA

#define UART1 USART1
#define UART1_CLK RCC_APB2Periph_USART1

/* UART参数配置 */
#define UART_BAUDRATE 115200 // 波特率
#define UART_BUFFER_SIZE 512 // 发送缓冲区大小

/* 函数声明 */
void UART1_Init(void);
void UART1_SendByte(uint8_t data);
void UART1_SendString(const char *str);
void UART1_SendData(uint16_t *data, uint16_t size);
void UART1_Send_ADC_Data(void);
void UART1_Test_Sequence(void);

/* 中断回调函数 */
void UART1_TX_Complete_Callback(void);

#endif /* __UART_H */
