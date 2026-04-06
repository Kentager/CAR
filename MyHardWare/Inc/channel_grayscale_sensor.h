#ifndef __CHANNEL_GRAYSCALE_SENSOR_H
#define __CHANNEL_GRAYSCALE_SENSOR_H

#include <stdint.h>

#define DIFF_SPEED_MAX 0.2f
static const float SENSOR_DIFF[8] = {-50.0f, -25.0f, -15.0f, -5.0f,
                                     5.0f,   15.0f,  25.0f,  50.0f};

typedef struct {
  uint8_t sensorFlag;     // 循迹标志位（0：不在线上 1：在线上）
  float sensorData[8];    // 传感器数据
  uint8_t sensorState[8]; // 传感器状态（0：白区 1：黑区）
  float diffSpeedMax;     // 输出最大差速值 (单位m/s)
  float diffSpeed;        // 输出差速值 (单位m/s)
  float sensorDiff[8];    // 差分值(单位%)
} irSensorData_t;

extern irSensorData_t irSensorData;

void irSensor_HwInit(void);
void irSensor_DataInit(irSensorData_t *sensor);
void irSensor_Update(irSensorData_t *sensor);
float irSensor_GetDiffSpeed(irSensorData_t *sensor);
int irSensor_GetSensorFlag(irSensorData_t *sensor);
#endif
