#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "stdint.h"

/**
 * @brief  环形缓冲区结构体
 */
typedef struct {
  uint8_t *buffer;        /**< 数据缓冲区指针 */
  uint16_t size;          /**< 缓冲区总大小 */
  uint16_t head;          /**< 写指针（指向下一个可写位置） */
  uint16_t tail;          /**< 读指针（指向下一个可读位置） */
  uint8_t is_dma_enabled; /**< DMA 使能标志 */
} RingBuffer;

/*---------------------- 函数声明 ----------------------------*/

uint8_t RingBuffer_Init(RingBuffer *rb, uint8_t *buffer, uint16_t size);
uint8_t RingBuffer_Push(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Pop(RingBuffer *rb, uint8_t *data);
uint16_t RingBuffer_Available(RingBuffer *rb);
uint16_t RingBuffer_SpaceAvailable(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);
uint8_t RingBuffer_IsFull(RingBuffer *rb);
void RingBuffer_Clear(RingBuffer *rb);
uint16_t RingBuffer_ContiguousRead(RingBuffer *rb, uint8_t **ptr);
uint16_t RingBuffer_ContiguousWrite(RingBuffer *rb, uint8_t **ptr);
void RingBuffer_SkipRead(RingBuffer *rb, uint16_t len);
void RingBuffer_SkipWrite(RingBuffer *rb, uint16_t len);

#endif //__RING_BUFFER_H