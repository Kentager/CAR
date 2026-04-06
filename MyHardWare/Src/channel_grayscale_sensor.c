#include "channel_grayscale_sensor.h"
#include "stm32f4xx.h"

irSensorData_t irSensorData;

/**
 * @brief 初始化连接8路巡线模块的 IO 口 (PF0~PF7)
 * @note  配置为上拉输入模式
 */
void irSensor_HwInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
                                GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void irSensor_DataInit(irSensorData_t *sensor) {
  sensor->sensorFlag = 0;
  sensor->diffSpeedMax = DIFF_SPEED_MAX;
  for (int i = 0; i < 8; i++) {
    sensor->sensorState[i] = 0;
    sensor->sensorData[i] = 0.0f;
    sensor->sensorDiff[i] = SENSOR_DIFF[i];
  }
}

/**
 * @brief 读取8路红外传感器数据并更新结构体
 * @param sensor 指向传感器结构体的指针
 * @note  假设传感器检测到黑线时，IO口输出低电平(0)
 */
void irSensor_Update(irSensorData_t *sensor) {
  // 逐个读取8个传感器引脚的状态并直接处理
  for (int i = 0; i < 8; i++) {
    // 读取第i个引脚的状态
    uint8_t pinState = GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_0 << i);

    // 检查引脚状态
    if (pinState == Bit_RESET) {
      sensor->sensorState[i] = 1; // 检测到黑线
    } else {
      sensor->sensorState[i] = 0; // 检测到白区
    }
  }

  // 2. 更新循迹标志位
  uint8_t onlineCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensor->sensorState[i] == 1) {
      onlineCount++;
    }
  }

  if (onlineCount > 0) {
    sensor->sensorFlag = 1; // 在线上
  } else {
    sensor->sensorFlag = 0; // 丢线
  }

  // 3. 更新差速值
  float diffSpeed = 0.0f;
  for (int i = 0; i < 8; i++) {
    if (sensor->sensorState[i])
      diffSpeed += sensor->sensorDiff[i] * 0.01f;
  }
  diffSpeed = diffSpeed * sensor->diffSpeedMax;
  sensor->diffSpeed = diffSpeed;
}

int irSensor_GetSensorFlag(irSensorData_t *sensor) {
  return sensor->sensorFlag;
}