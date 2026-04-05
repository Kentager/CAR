#include "channel_grayscale_sensor.h"
#include "stm32f4xx.h"
#include "task.h"
#include "usart.h"

// 灰度传感器数据打印任务
void Task_PrintGraySensorData(void) {
  // 打印灰度传感器数据
  for (int i = 0; i < 8; i++) {
    printf("%d,", irSensorData.sensorState[i]);
  }
  printf(" %d", irSensorData.sensorFlag);
  // 换行
  printf("\r\n");
}
// 灰度传感器数据更新任务
void Task_UpdateGraySensorData(void) {
  // 更新灰度传感器数据
  irSensor_Update(&irSensorData);
}
int main(void) {
  // 初始化串口
  USART1_Init();
  // 初始化灰度传感器驱动
  irSensor_HwInit();
  // 初始化灰度传感器数据
  irSensor_DataInit(&irSensorData);
  // 初始化任务
  Task_Init();
  // 添加任务
  add_task(Task_PrintGraySensorData, 10);
  add_task(Task_UpdateGraySensorData, 10);

  while (1) {
    // 执行任务
    Task_Scheduler();
  }
}