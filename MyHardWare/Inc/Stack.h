#ifndef STACK_H
#define STACK_H

#include "sys.h"

#define Stack_ElemType int8_t

typedef struct Stack {
  Stack_ElemType *Data;
  int8_t SP;
  u8 MaxSize;
} Stack;

void Stack_Init(struct Stack *s, u8 MaxSize);
u8 PUSH(struct Stack *s, Stack_ElemType PushData);
Stack_ElemType POP(struct Stack *s);
FlagStatus Stack_Empety(struct Stack *s);

#endif
