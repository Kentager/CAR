#include "TargetSpeed_Set_App.h"
#include "channel_grayscale_sensor.h"
#include "pid_angle.h"
#include "pid_speed.h"
TargetSpeed_t TargetSpeed;

static float speed_left_x = 0.0f;
static float speed_right_x = 0.0f;
static uint32_t count = 0;
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
  speed_left_x = TargetSpeed.speed_left;
  speed_right_x = TargetSpeed.speed_right;

  float pid_angle_diff;
  float angle_target_h;
  float error;
  switch (TargetSpeed.mode) {
  case MODE_NONE:
    break;
  case MODE_ANGLE_LOOP:
    angle_target_h = TargetSpeed.target_angle + TargetSpeed.start_angle;
    // 对设定值进行归一化，确保在 [-180, 180] 范围内

    while (angle_target_h > 180.0f)
      angle_target_h -= 360.0f;
    while (angle_target_h < -180.0f)
      angle_target_h += 360.0f;

    // Angle_PID_Yaw.pid_state.target_value = angle_target_h;
    // pid_angle_diff = Angle_PID_Update(&Angle_PID_Yaw, TargetSpeed.yaw_angle);
    error = angle_target_h - TargetSpeed.yaw_angle;
    // 对误差进行归一化，确保在 [-180, 180] 范围内
    while (error > 180.0f)
      error -= 360.0f;

    while (error < -180.0f)
      error += 360.0f;

    pid_angle_diff = error / 100 * 0.35;

    speed_right_x += pid_angle_diff;
    speed_left_x -= pid_angle_diff;
    break;
  case MODE_TRACKING:
    speed_right_x -= irSensorData.diffSpeed;
    speed_left_x += irSensorData.diffSpeed;
    break;
  }
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, speed_left_x);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, speed_right_x);
}
