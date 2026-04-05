#include "TargetSpeed_Set_App.h"
#include "channel_grayscale_sensor.h"
#include "pid_angle.h"
#include "pid_speed.h"

TargetSpeed_t TargetSpeed;

void TargetSpeed_Init(void) {
  TargetSpeed.speed_left = 0.0f;
  TargetSpeed.speed_right = 0.0f;
  Target TargetSpeed.mode = MODE_NONE;
}
void TargetSpeedMode_Set(TargetSpeedMode_e mode) { TargetSpeed.mode = mode; }

void TargetSpeed_Set(float speed_left, float speed_right) {
  TargetSpeed.speed_left = speed_left;
  TargetSpeed.speed_right = speed_right;
}

void TargetSpeed_Update(void) {
  switch (TargetSpeed.mode) {
  case MODE_NONE:
    break;
  case MODE_ANGLE_LOOP:
    TargetSpeed.speed_right += irSensorData.diffSpeed / 2;
    TargetSpeed.speed_left -= irSensorData.diffSpeed / 2;
    break;
  case MODE_TRACKING:

    break;
  }
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, TargetSpeed.speed_left);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, TargetSpeed.speed_right);
}
