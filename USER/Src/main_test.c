#include "control.h"
#include "delay.h"
#include "encoder.h"
#include "main.h"
#include "pid_speed.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* ==================== GoStraight 状态机 ==================== */
typedef enum {
  GOSTRAIGHT_STATE_IDLE = 0,     // 空闲态
  GOSTRAIGHT_STATE_RUNNING = 1,  // 运行态
  GOSTRAIGHT_STATE_COMPLETED = 2 // 完成态
} GoStraight_State_e;

typedef struct {
  GoStraight_State_e state; // 当前状态
  float target_distance;    // 目标距离 (m)
  float start_distance;     // 起始距离 (m)
  float target_speed;       // 目标速度 (m/s)
  float target_angle;       // 目标角度 (度)
  uint8_t is_active;        // 是否激活
} GoStraight_Control_t;

static GoStraight_Control_t gostraight_ctrl = {0};

void tControl_updagte(void) { Control_Update(); }

void tSpeedControl_update(void) {

  // 更新右轮速度环PID控制器
  //   printf("speed_control_task tim:%d\r\n", GetSysTick());
  // if (step % 4 == 0) {

  //   printf("%f,%f,%f,%f,%f,%f\r\n", Speed_PID_Right.target_speed_m_s,
  //          Speed_PID_Left.current_speed_m_s,
  //          Speed_PID_Right.current_speed_m_s, Speed_PID_Left.pid_state.kp,
  //          Speed_PID_Left.pid_state.ki, Speed_PID_Left.pid_state.kd);
  // }

  static uint32_t control_step = 0;
  if (control_step++ % 4 == 0)
    printf("speed:%f,%f\r\n", Speed_PID_Left.current_speed_m_s,
           Speed_PID_Right.current_speed_m_s);
  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);
}
/**
 * @brief 前行distance米
 *
 * @param distance 前行距离 (米)
 *
 * @retval 无
 * @note 此函数可以用在task中使用，且每次执行必小于1ms即无阻塞
 *       默认速度0.3 m/s，角度保持0度（直线行驶）
 */
void GoStraight(float distance) {
  // 参数检查
  if (distance <= 0) {
    return;
  }

  // 获取当前左轮编码器里程作为基准
  Encoder_Data_t left_encoder_data = Encoder_GetData(ENCODER_LEFT);

  // 初始化控制参数
  gostraight_ctrl.state = GOSTRAIGHT_STATE_RUNNING;
  gostraight_ctrl.target_distance = distance;
  gostraight_ctrl.start_distance = left_encoder_data.total_distance;
  gostraight_ctrl.target_speed = 0.15f; // 默认速度 0.3 m/s
  gostraight_ctrl.target_angle = 0.0f;  // 保持直线行驶
  gostraight_ctrl.is_active = 1;

  // 启用速度环PID并设置目标速度
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, gostraight_ctrl.target_speed);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, gostraight_ctrl.target_speed);

  // 设置控制模块参数
  Control_SetSpeed(gostraight_ctrl.target_speed);
  Control_SetAngle(gostraight_ctrl.target_angle);
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_Enable(); // 启用角度环控制
}

/**
 * @brief GoStraight 任务更新函数 (在任务调度器中周期调用)
 *
 * @retval 无
 * @note 建议调用周期: 10ms
 */
void tGoStraight_Update(void) {
  // 检查是否激活
  if (!gostraight_ctrl.is_active) {
    return;
  }

  switch (gostraight_ctrl.state) {
  case GOSTRAIGHT_STATE_IDLE:
    // 空闲态，不做任何操作
    break;

  case GOSTRAIGHT_STATE_RUNNING: {
    // 获取当前左轮编码器里程
    Encoder_Data_t left_encoder_data = Encoder_GetData(ENCODER_LEFT);

    // 计算已行驶距离
    float current_distance =
        left_encoder_data.total_distance - gostraight_ctrl.start_distance;

    // 判断是否到达目标距离
    if (current_distance >= gostraight_ctrl.target_distance) {
      // 到达目标，切换到完成态
      gostraight_ctrl.state = GOSTRAIGHT_STATE_COMPLETED;

      // 停止电机
      Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.0f);
      Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.0f);
      Control_Stop();

      printf("GoStraight 完成: 目标距离=%.2fm, 实际距离=%.2fm\r\n",
             gostraight_ctrl.target_distance, current_distance);
    }
    break;
  }

  case GOSTRAIGHT_STATE_COMPLETED:
    // 完成态，重置为空闲态
    gostraight_ctrl.state = GOSTRAIGHT_STATE_IDLE;
    gostraight_ctrl.is_active = 0;
    break;

  default:
    gostraight_ctrl.state = GOSTRAIGHT_STATE_IDLE;
    break;
  }
}
void task_necesarry(void) {
  add_task(Encoder_Update, 5);
  add_task(tSpeedControl_update, 5);
  //   add_task(Control_Update, 5);
}
void task_app(void) {
  // 注册 GoStraight 状态更新任务（建议周期10ms）
  add_task(tGoStraight_Update, 10);

  // 示例：启动后延迟3秒，然后前行1米
  // 注意：实际使用时应该通过串口命令或按键触发
  /*
  delay_ms(3000);
  GoStraight(1.0f); // 前行1米
  */
}
void system_init(void) {
  delay_init();
  USART1_Init();
  Task_Init();
  Motor_Driver_Init();
  Encoder_Driver_Init();
  while (mpu_dmp_init() != 0) {
    printf("MPU6050 DMP初始化失败，重试中...\r\n");
    delay_ms(500);
  }
  printf("MPU6050 DMP初始化成功\r\n");
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.1f);

  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.1f);
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);
  Control_Init();
  Control_SetSpeed(0.05);             // 设置速度 0.5 m/s
  Control_SetAngle(0.05f);            // 设置目标角度 0度
  Control_SetMode(CONTROL_MODE_TURN); // 设置模式
  Control_Enable();                   // 启用控制
}
int main(void) {
  system_init();
  delay_ms(2000);
  task_necesarry();
  task_app();
  GoStraight(1);
  while (1) {
    Task_Scheduler();
  }
}