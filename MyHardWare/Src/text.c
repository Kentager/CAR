#include "text.h"
#include "motor.h"
#include "pid_speed.h"

float Text(float speed_set_text) {
    Speed_PID_SetTargetSpeed(Speed_PID_Right,0.2f);
    // Motor_SetSpeed(MOTOR_LEFT, 3000);  // 左电机正转，速度 4000
    // Motor_SetSpeed(MOTOR_RIGHT, 3000); // 右电机正转，速度 4000

    // Speed_PID_Init(&Speed_PID_Right, Speed_PID_Right.encoder_id, Speed_PID_Right.motor_id, 0.0f, 0.0f, 0.0f);

    // Motor_SetDirection(MOTOR_LEFT, MOTOR_DIR_FORWARD);
    // Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);

    // // 更新电机状态，将配置应用到硬件
    // Motor_Update(MOTOR_LEFT);
    // Motor_Update(MOTOR_RIGHT);
}
