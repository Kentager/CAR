#ifndef __TARGET_SPEED_SET_APP_H__
#define __TARGET_SPEED_SET_APP_H__

#include "pid_speed.h"
#include "stm32f4xx.h"

typedef enum {
  MODE_NONE = 0,       // 无模式
  MODE_ANGLE_LOOP = 1, // 角度环模式
  MODE_TRACKING = 2    // 循迹模式
} TargetSpeedMode_e;

typedef struct {
  float speed_left;
  float speed_right;
  TargetSpeedMode_e mode;
} TargetSpeed_t;

extern TargetSpeed_t TargetSpeed;

void TargetSpeed_Init(void);

void TargetSpeedMode_Set(TargetSpeedMode_e mode);

void TargetSpeed_Set(float speed_left, float speed_right);

void TargetSpeed_Update(void);
#endif
