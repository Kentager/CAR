#include "TargetSpeed_Set_App.h"
#include "channel_grayscale_sensor.h"
#include "pid_angle.h"
#include "pid_speed.h"
TargetSpeed_t TargetSpeed;

void TargetSpeed_Init(void) {
  TargetSpeed.speed_left = 0.0f;
  TargetSpeed.speed_right = 0.0f;
  TargetSpeed.start_angle = 0.0f;
  TargetSpeed.target_angle = 0.0f;
  TargetSpeed.yaw_angle = 0.0f;
  TargetSpeed.mode = MODE_NONE;
}
void TargetSpeedMode_Set(TargetSpeedMode_e mode) { TargetSpeed.mode = mode; }

void TargetSpeed_SetSpeed(float speed_left, float speed_right) {
  TargetSpeed.speed_left = speed_left;
  TargetSpeed.speed_right = speed_right;
}

void TargetSpeed_SetStartAngle() {
  TargetSpeed.start_angle = TargetSpeed.yaw_angle;
}

void TargetSpeed_SetTargetAngle(float target_angle) {
  TargetSpeed.target_angle = target_angle;
}

void TargetSpeed_SetYawAngle(float yaw_angle) {
  TargetSpeed.yaw_angle = yaw_angle;
}
void TargetSpeed_Update(void) {
  float speed_left_x = TargetSpeed.speed_left;
  float speed_right_x = TargetSpeed.speed_right;
  float pid_angle_diff;
  float angle_diff;
  switch (TargetSpeed.mode) {
  case MODE_NONE:
    break;
  case MODE_ANGLE_LOOP:
    // pid_angle_diff =
    //     Angle_PID_Update(&Angle_PID_Yaw,
    //                      TargetSpeed.target_angle + TargetSpeed.start_angle)
    //                      /
    //     2;

    angle_diff = TargetSpeed.target_angle + TargetSpeed.start_angle;
    if (angle_diff > 180.0f) {
      angle_diff = angle_diff - 360.0f;
    } else if (angle_diff < -180.0f) {
      angle_diff = angle_diff + 360.0f;
    }
    angle_diff = angle_diff - TargetSpeed.yaw_angle;

    // if (angle_diff < 1.0f && angle_diff > -1.0f)
    //   angle_diff = 0.0f;
    pid_angle_diff = angle_diff / 100.0f * 0.1f;
    speed_left_x -= pid_angle_diff;
    speed_right_x += pid_angle_diff;
    break;
  case MODE_TRACKING:
    speed_right_x -= irSensorData.diffSpeed;
    speed_left_x += irSensorData.diffSpeed;
    break;
  }
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, speed_left_x);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, speed_right_x);
}
