#include "ring_buffer.h"
#include <string.h>

/**
 * @brief  初始化环形缓冲区
 * @param  rb: 环形缓冲区结构体指针
 * @param  buffer: 数据缓冲区指针
 * @param  size: 缓冲区大小
 * @retval 成功返回 1，失败返回 0
 */
uint8_t RingBuffer_Init(RingBuffer *rb, volatile uint8_t *buffer,
                        uint16_t size) {
  if (rb == NULL || buffer == NULL || size == 0) {
    return 0;
  }
  rb->buffer = buffer;
  rb->size = size;
  rb->head = 0;
  rb->tail = 0;
  rb->is_dma_enabled = 0;
  return 1;
}

/**
 * @brief  向环形缓冲区写入一个字节
 * @param  rb: 环形缓冲区结构体指针
 * @param  data: 要写入的数据
 * @retval 成功返回 1，失败返回 0（缓冲区满）
 */
uint8_t RingBuffer_Push(RingBuffer *rb, uint8_t data) {
  if (rb == NULL) {
    return 0;
  }

  uint16_t next_head = (rb->head + 1) % rb->size;
  if (next_head == rb->tail) {
    return 0;
  }

  rb->buffer[rb->head] = data;
  rb->head = next_head;
  return 1;
}

/**
 * @brief  从环形缓冲区读取一个字节
 * @param  rb: 环形缓冲区结构体指针
 * @param  data: 存储读取数据的指针
 * @retval 成功返回 1，失败返回 0（缓冲区空）
 */
uint8_t RingBuffer_Pop(RingBuffer *rb, uint8_t *data) {
  if (rb == NULL || data == NULL) {
    return 0;
  }

  if (rb->tail == rb->head) {
    return 0;
  }

  *data = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % rb->size;
  return 1;
}

/**
 * @brief  获取环形缓冲区中已用空间大小
 * @param  rb: 环形缓冲区结构体指针
 * @retval 已用空间大小（字节数）
 */
uint16_t RingBuffer_Available(RingBuffer *rb) {
  if (rb == NULL) {
    return 0;
  }

  if (rb->head >= rb->tail) {
    return rb->head - rb->tail;
  } else {
    return rb->size - rb->tail + rb->head;
  }
}

/**
 * @brief  获取环形缓冲区中剩余可用空间大小
 * @param  rb: 环形缓冲区结构体指针
 * @retval 剩余可用空间大小（字节数）
 */
uint16_t RingBuffer_SpaceAvailable(RingBuffer *rb) {
  if (rb == NULL) {
    return 0;
  }

  uint16_t used;
  if (rb->head >= rb->tail) {
    used = rb->head - rb->tail;
  } else {
    used = rb->size - rb->tail + rb->head;
  }
  return rb->size - used - 1;
}

/**
 * @brief  判断环形缓冲区是否为空
 * @param  rb: 环形缓冲区结构体指针
 * @retval 空返回 1，非空返回 0
 */
uint8_t RingBuffer_IsEmpty(RingBuffer *rb) {
  if (rb == NULL) {
    return 1;
  }
  return (rb->head == rb->tail) ? 1 : 0;
}

/**
 * @brief  判断环形缓冲区是否已满
 * @param  rb: 环形缓冲区结构体指针
 * @retval 满返回 1，未满返回 0
 */
uint8_t RingBuffer_IsFull(RingBuffer *rb) {
  if (rb == NULL) {
    return 0;
  }
  return (((rb->head + 1) % rb->size) == rb->tail) ? 1 : 0;
}

/**
 * @brief  清空环形缓冲区
 * @param  rb: 环形缓冲区结构体指针
 * @retval 无
 */
void RingBuffer_Clear(RingBuffer *rb) {
  if (rb != NULL) {
    rb->head = 0;
    rb->tail = 0;
    rb->is_dma_enabled = 0;
  }
}

/**
 * @brief  获取连续可读区域指针和长度
 * @param  rb: 环形缓冲区结构体指针
 * @param  ptr: 输出参数，指向连续可读区域的指针
 * @retval 连续可读区域的长度（字节数），无法读取返回 0
 * @note   配合 RingBuffer_SkipRead() 使用，实现零拷贝读取
 */
uint16_t RingBuffer_ContiguousRead(RingBuffer *rb, uint8_t **ptr) {
  if (rb == NULL || ptr == NULL || rb->head == rb->tail) {
    *ptr = NULL;
    return 0;
  }

  *ptr = &rb->buffer[rb->tail];

  if (rb->head > rb->tail) {
    return rb->head - rb->tail;
  } else {
    return rb->size - rb->tail;
  }
}

/**
 * @brief  获取连续可写区域指针和长度
 * @param  rb: 环形缓冲区结构体指针
 * @param  ptr: 输出参数，指向连续可写区域的指针
 * @retval 连续可写区域的长度（字节数），无法写入返回 0
 * @note   配合 RingBuffer_SkipWrite() 使用，实现零拷贝写入
 */
uint16_t RingBuffer_ContiguousWrite(RingBuffer *rb, uint8_t **ptr) {
  if (rb == NULL || ptr == NULL) {
    *ptr = NULL;
    return 0;
  }

  uint16_t space = RingBuffer_SpaceAvailable(rb);
  if (space == 0) {
    *ptr = NULL;
    return 0;
  }

  *ptr = &rb->buffer[rb->head];

  if (rb->head >= rb->tail) {
    return rb->size - rb->head;
  } else {
    return rb->tail - rb->head - 1;
  }
}

/**
 * @brief  跳过已读取的数据（移动读指针）
 * @param  rb: 环形缓冲区结构体指针
 * @param  len: 要跳过的字节数
 * @retval 无
 * @note   通常在 DMA 直接写入缓冲区后调用此函数更新读指针
 */
void RingBuffer_SkipRead(RingBuffer *rb, uint16_t len) {
  if (rb == NULL) {
    return;
  }

  uint16_t available = RingBuffer_Available(rb);
  if (len > available) {
    len = available;
  }

  rb->tail = (rb->tail + len) % rb->size;
}

/**
 * @brief  跳过已写入的数据（移动写指针）
 * @param  rb: 环形缓冲区结构体指针
 * @param  len: 要跳过的字节数
 * @retval 无
 * @note   通常在 CPU 直接写入缓冲区后调用此函数更新写指针
 */
void RingBuffer_SkipWrite(RingBuffer *rb, uint16_t len) {
  if (rb == NULL) {
    return;
  }

  uint16_t space = RingBuffer_SpaceAvailable(rb);
  if (len > space) {
    len = space;
  }

  rb->head = (rb->head + len) % rb->size;
}