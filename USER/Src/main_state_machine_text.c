#include "TargetSpeed_Set_App.h"
#include "channel_grayscale_sensor.h"
#include "delay.h"
#include "encoder.h"
#include "hmc5883l.h"
#include "inv_mpu.h"
#include "key.h"
#include "math.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_angle.h"
#include "pid_speed.h"
#include "pid_timer.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_syscfg.h"
#include "task.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>

typedef enum {
  STATE_STOP = 0,    // 待机
  STATE_RUN = 1,     // 前进
  STATE_TRACK = 2,   // 循迹
  STATE_TRACK_X = 3, // 循迹
} State_Machine_State_enum;

typedef struct {
  uint8_t state;       // 状态机状态
  uint8_t last_state;  // 上一个状态
  uint8_t state_count; // 状态计数
} State_Machine_Typedef;

State_Machine_Typedef state_machine; // 状态机结构体
static float yaw_angle = 0.0f;
static uint32_t count = 0;
static uint32_t count_x = 0;
//===========================================初始化函数=========================================//初始化驱动和配置
void Start_Init(void) { // 初始化所有驱动
  // 初始化延时定时器
  delay_init();

  TIM3_5ms_Init(); // 初始化TIM3 5ms定时器
  // 初始化按键
  KEY_Init();
  // 初始化按键中断
  KEY_Interrupt_Init();
  // 初始化编码器
  Encoder_Driver_Init();
  // 初始化电机驱动
  Motor_Driver_Init();
  // 初始化串口
  USART1_Init();
  // 初始化任务
  Task_Init();
  // 初始化灰度传感器驱动
  irSensor_HwInit();
  // 初始化灰度传感器数据
  irSensor_DataInit(&irSensorData);
  // 初始化速度环PID控制器
  //// 右轮: 关联ENCODER_RIGHT编码器和MOTOR_RIGHT电机
  Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);

  //// 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
  Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT,
                 SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT,
                 SPEED_PID_KD_DEFAULT);
  //// 启动角度环PID控制器
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);

  // 初始化角度环PID控制器
  //// 初始化偏航角的角度环PID控制器
  Angle_PID_Init(&Angle_PID_Yaw, ANGLE_AXIS_YAW, ANGLE_PID_KP_DEFAULT,
                 ANGLE_PID_KI_DEFAULT, ANGLE_PID_KD_DEFAULT);
  //// 启动偏航角的角度环PID控制器
  Angle_PID_Enable(&Angle_PID_Yaw);

  // 初始化MPU6050
  // MPU_Init();

  while (mpu_dmp_init()) {
    printf("MPU6050 init failed!\r\n");
  }
  printf("MPU6050 init success|\r\n");
  // // 初始化HMC5883L
  // hmc5883l_init();
  // // HMC5883L第一次测量
  // hmc5883l_single_measurement();
}

void State_Machine_Init(State_Machine_Typedef *state_machine) { // 状态机初始化
  state_machine->state = STATE_STOP;
  state_machine->last_state = STATE_STOP;
  state_machine->state_count = 0;
}

//===========================================指令函数==========================================//设置小车的速度以及循迹模块和灰度模块的开关

void State_Stop_Action(void) {      // 待机 无模式 设置速度为0 0
  TargetSpeedMode_Set(MODE_NONE);   // 无模式
  TargetSpeed_SetSpeed(0.0f, 0.0f); // 设置目标速度（左轮为0，右轮为0）
}

void State_Run_Action(void) {           // 前进 角度环模式 设置速度为 0.2 0.2
  TargetSpeedMode_Set(MODE_ANGLE_LOOP); // 角度环模式
  TargetSpeed_SetSpeed(0.4f, 0.4f); // 设置目标速度（左轮为0.12，右轮为0.12）
}

void State_Track_Action(void) {       // 循迹 循迹模式 设置速度为 0.2 0.2
  TargetSpeedMode_Set(MODE_TRACKING); // 循迹模式
  TargetSpeed_SetSpeed(0.34f, 0.45f); // 设置目标速度（左轮为0.2，右轮为0.2）
}
void State_Track_X_Action(void) {     // 循迹 循迹模式 设置速度为 0.2 0.2
  TargetSpeedMode_Set(MODE_TRACKING); // 循迹模式
  TargetSpeed_SetSpeed(0.45f, 0.34f); // 设置目标速度（左轮为0.2，右轮为0.2）
}

void Reset_Action(void) {         // 无模式 重置初始角度
  TargetSpeedMode_Set(MODE_NONE); // 无模式
  TargetSpeed_SetStartAngle();
}

//===========================================状态更新函数==========================================//检测状态更新，调用指令函数
// 状态机状态更新（状态变化后调用相应的指令函数）
void State_Update(State_Machine_Typedef *state_machine) {
  // 更新上次状态
  state_machine->last_state = state_machine->state;
  // 根据状态机状态执行不同的操作
  switch (state_machine->state) {
  case STATE_STOP:
    State_Stop_Action();
    break;
  case STATE_RUN:
    State_Run_Action();
    break;
  case STATE_TRACK:
    State_Track_Action();
    break;
  case STATE_TRACK_X:
    State_Track_X_Action();
    break;
  }
}
// 状态机状态计数更新（按照流程完成问题）//XXXXXXXXXXXXXXXXXXXXXXXX左左左左左左左左左左左左左左左左左左
void State_Count_Updata(State_Machine_Typedef *state_machine) {

  switch (count_it) {
    //====================待机状态====================//
  case 0:
    state_machine->state = STATE_STOP; // 待机
    state_machine->state_count++;
    count = 0; // 清除计数器
    Reset_Action();
    break;
    //====================题目1====================//
  case 1:
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(0.0f);            // 设置角度为0
    state_machine->state = STATE_RUN;            // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 2:
    state_machine->state = STATE_STOP; // 待机
    Reset_Action();
    break;
    //====================题目2====================//
  case 3:
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(0.0f);            // 设置角度为0
    state_machine->state = STATE_RUN;            // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 4:
    state_machine->state = STATE_TRACK_X;
    if (++count < 100)                            // 延时1000ms
      return;                                     // 循迹
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 5:
    TargetSpeed_SetTargetAngle(-180.0f);         // 设置角度为180
    state_machine->state = STATE_RUN;            // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
    }
    break;
  case 6:
    state_machine->state = STATE_TRACK_X; // 循迹
    if (++count < 100)                    // 延时1000ms
      return;                             // 循迹
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 7:
    state_machine->state = STATE_STOP; // 待机
    Reset_Action();
    break;
    //====================题目3====================//
  case 8:
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(-50.0f); // 设置角度为-45
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 250)
      return;
    TargetSpeed_SetTargetAngle(0.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
      count_x = 0;
    }
    break;
  case 9:
    state_machine->state = STATE_TRACK; // 循迹
    if (++count < 100)                  // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 10:
    TargetSpeed_SetTargetAngle(230.0f); // 设置角度为225
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 210)
      return;
    TargetSpeed_SetTargetAngle(180.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count_x = 0;
    }
    break;
  case 11:
    state_machine->state = STATE_TRACK_X; // 循迹
    if (++count < 100)                    // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 12:
    state_machine->state = STATE_STOP; // 待机
    Reset_Action();
    break;
    //====================题目4====================//
  case 13:             // 1
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(-50.0f); // 设置角度为-45
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 250)
      return;
    TargetSpeed_SetTargetAngle(0.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
      count_x = 0;
    }
    break;
  case 14:
    state_machine->state = STATE_TRACK; // 循迹
    if (++count < 100)                  // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 15:                              // 1.5
    TargetSpeed_SetTargetAngle(230.0f); // 设置角度为225
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 205)
      return;
    TargetSpeed_SetTargetAngle(180.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count_x = 0;
    }
    break;
  case 16:
    state_machine->state = STATE_TRACK_X; // 循迹
    if (++count < 100)                    // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 17:                              // 2
    TargetSpeed_SetTargetAngle(-50.0f); // 设置角度为-45
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 225)
      return;
    TargetSpeed_SetTargetAngle(0.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
      count_x = 0;
    }
    break;
  case 18:
    state_machine->state = STATE_TRACK; // 循迹
    if (++count < 100)                  // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 19:                              // 2.5
    TargetSpeed_SetTargetAngle(230.0f); // 设置角度为225
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 220)
      return;
    TargetSpeed_SetTargetAngle(180.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count_x = 0;
    }
    break;
  case 20:
    state_machine->state = STATE_TRACK_X; // 循迹
    if (++count < 100)                    // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 21:                              // 3
    TargetSpeed_SetTargetAngle(-50.0f); // 设置角度为-45
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 220)
      return;
    TargetSpeed_SetTargetAngle(0.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count = 0; // 清除计数器
      count_x = 0;
    }
    break;
  case 22:
    state_machine->state = STATE_TRACK; // 循迹
    if (++count < 100)                  // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 23:                              // 3.5
    TargetSpeed_SetTargetAngle(230.0f); // 设置角度为225
    state_machine->state = STATE_RUN;   // 前进
    if (++count_x < 210)
      return;
    TargetSpeed_SetTargetAngle(180.0f);
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      count_it++;
      count_x = 0;
    }
    break;
  case 24:
    state_machine->state = STATE_TRACK_X; // 循迹
    if (++count < 100)                    // 延时1000ms
      return;
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      count_it++;
      count = 0; // 清除计数器
    }
    break;
  case 25:
    state_machine->state = STATE_STOP; // 待机
    Reset_Action();
    break;
  }
}
//===========================================任务调度函数==========================================//设置Task任务
void State_Update_Task(void) {
  if (state_machine.state != state_machine.last_state) {
    state_machine.last_state = state_machine.state;
    State_Update(&state_machine);
  }
}

void State_Count_Updata_Task(void) { State_Count_Updata(&state_machine); }

// void PID_Speed_Updata_Task(void) {
//   Speed_PID_Update(&Speed_PID_Right);
//   Speed_PID_Update(&Speed_PID_Left);
// }

void Channel_Grayscale_Sensor_Updata_Task(void) {
  irSensor_Update(&irSensorData);
}

void Yaw_Angle_Data_filtering(void) {
  float pitch, roll, yaw;
  static float last_yaw = 0.0f;          // 保存上一次的yaw值
  static float yaw_calibration = 1.160f; // 角度校准系数：90/78.69 ≈ 1.144
  float yaw_delta;                       // 角度变化量

  mpu_dmp_get_data(&pitch, &roll, &yaw);
  if (yaw == 0.000f)
    return;
  // 计算角度变化量
  yaw_delta = yaw - last_yaw;

  // 处理角度跳变（从180到-180或从-180到180）
  if (yaw_delta > 180.0f) {
    yaw_delta -= 360.0f;
  } else if (yaw_delta < -180.0f) {
    yaw_delta += 360.0f;
  }

  // 对角度变化量应用校准系数并累积
  yaw_angle += yaw_delta * yaw_calibration;

  // 更新上一次的yaw值
  last_yaw = yaw;

  TargetSpeed_SetYawAngle(yaw_angle);
  printf("%.2f,%.2f,%.2f\r\n", TargetSpeed.start_angle,
         TargetSpeed.target_angle, TargetSpeed.yaw_angle);
}
//===========================================主函数==========================================//
// pid_speed_update和encoder_update需要定期调用
int main(void) {
  // 初始化
  Start_Init();
  // 状态机初始化
  State_Machine_Init(&state_machine);
  // 添加任务
  // add_task(Encoder_Update, 2); // 编码器更新任务
  // add_task(PID_Speed_Updata_Task,
  //          SPEED_PID_SAMPLE_PERIOD_MS);              // 速度PID更新任务

  add_task(Channel_Grayscale_Sensor_Updata_Task, 1); // 循迹更新任务
  add_task(TargetSpeed_Update, 2);                   // 目标速度更新任务
  add_task(State_Update_Task, 10);                   // 状态机状态更新任务
  add_task(State_Count_Updata_Task, 10);             // 状态机状态计数更新任务
  add_task(Yaw_Angle_Data_filtering, 10);            // 角度滤波任务
  // add_task(hmc5883l_single_measurement, 5);
  TIM3_5ms_Start(); // 启动定时器
  while (1) {
    // 任务调度
    Task_Scheduler();
  }
}