/**
 * @file uart.c
 * @brief UART1驱动实现文件 - 串口数据传输
 * @version 1.0
 * @date 2025-12-15
 */

#include "uart.h"
#include <stdint.h>

/* 私有变量 */
/* static uint8_t uart_tx_buffer[UART_BUFFER_SIZE]; // 保留供将来使用 */
/* static volatile uint16_t uart_tx_index = 0;    // 保留供将来使用 */
/* static volatile uint16_t uart_tx_length = 0;   // 保留供将来使用 */

/* 私有函数声明 */
static void Int16ToString(uint16_t num, char *str);

/**
 * @brief  UART1 GPIO初始化函数
 * @param  None
 * @retval None
 */
static void UART1_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /* 使能GPIOA时钟 */
  RCC_AHB1PeriphClockCmd(UART1_GPIO_CLK, ENABLE);

  /* 配置PA9为复用功能推挽输出 */
  GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  /* 配置PA10为复用功能输入 */
  GPIO_InitStructure.GPIO_Pin = UART1_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStructure);

  /* 配置GPIO复用功能 */
  GPIO_PinAFConfig(UART1_GPIO_PORT, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(UART1_GPIO_PORT, GPIO_PinSource10, GPIO_AF_USART1);
}

/**
 * @brief  UART1初始化函数
 * @param  None
 * @retval None
 */
void UART1_Init(void) {
  USART_InitTypeDef USART_InitStructure;

  /* 使能USART1时钟 */
  RCC_APB2PeriphClockCmd(UART1_CLK, ENABLE);

  /* GPIO初始化 */
  UART1_GPIO_Init();

  /* UART1配置 */
  USART_InitStructure.USART_BaudRate = UART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(UART1, &USART_InitStructure);

  /* 使能USART1 */
  USART_Cmd(UART1, ENABLE);
}

/**
 * @brief  UART1发送一个字节
 * @param  data: 要发送的数据
 * @retval None
 */
void UART1_SendByte(uint8_t data) {
  /* 等待发送缓冲区为空 */
  while (USART_GetFlagStatus(UART1, USART_FLAG_TXE) == RESET)
    ;

  /* 强制转换为16位并发送数据，确保正确的数据位传输 */
  USART_SendData(UART1, (uint16_t)data);
}

/**
 * @brief  UART1发送字符串
 * @param  str: 要发送的字符串
 * @retval None
 */
void UART1_SendString(const char *str) {
  while (*str) {
    UART1_SendByte(*str++);
  }
}

/**
 * @brief  UART1发送数据数组
 * @param  data: 要发送的数据数组
 * @param  size: 数据大小
 * @retval None
 */
void UART1_SendData(uint16_t *data, uint16_t size) {
  uint16_t i, j;
  char temp_str[16];

  for (i = 0; i * 10 < size; i++) {

    for (j = 0; (i * 10 + j) < size; i++) {
      /* 将16位数据转换为字符串 */
      Int16ToString(data[i * 10 + j], temp_str);
      UART1_SendString(temp_str);
      UART1_SendString(" ");

      /* 添加小延时避免发送过快 */
      /* 可以根据需要调整延时 */
    }
    UART1_SendString("\r\n");
  }
}

/**
 * @brief  UART1发送ADC数据缓冲区
 * @param  None
 * @retval None
 */

/**
 * @brief  将16位整数转换为字符串
 * @param  num: 要转换的数字
 * @param  str: 存储结果的字符串缓冲区
 * @retval None
 */
static void Int16ToString(uint16_t num, char *str) {
  uint16_t i = 0;
  uint16_t j = 0;
  char digits[6]; // 16位最大值65535，需要5位+结束符

  /* 处理0的特殊情况 */
  if (num == 0) {
    str[0] = '0';
    str[1] = '\0';
    return;
  }

  /* 分解数字到各位 */
  while (num > 0) {
    digits[i++] = (num % 10) + '0';
    num /= 10;
  }
  digits[i] = '\0';

  /* 反转字符串 */
  for (j = 0; j < i; j++) {
    str[j] = digits[i - 1 - j];
  }
  str[i] = '\0';
}

/**
 * @brief  UART1测试函数 - 发送测试字节序列
 * @param  None
 * @retval None
 */
void UART1_Test_Sequence(void) {
  /* 发送测试字节序列来验证数据完整性 */
  uint8_t test_bytes[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
                          0x40, 0x80, 0xFF, 0x00, 0xF1};
  uint16_t i;

  UART1_SendString("=== UART Test Sequence ===\r\n");
  for (i = 0; i < sizeof(test_bytes); i++) {
    UART1_SendByte(test_bytes[i]);
    /* 添加小延时 */
    for (volatile uint32_t delay = 0; delay < 1000; delay++)
      ;
  }
  UART1_SendString("\r\n=== End Test ===\r\n");
}

/**
 * @brief  UART1发送完成回调函数
 * @param  None
 * @retval None
 */
void UART1_TX_Complete_Callback(void) {
  /* 发送完成回调函数 */
  /* 可以在这里添加发送完成后的处理逻辑 */
}

int __io_putchar(int ch) {
  UART1_SendByte((uint8_t)ch);
  return ch;
}