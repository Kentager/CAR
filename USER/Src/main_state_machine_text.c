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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_syscfg.h"
#include "task.h"
#include "usart.h"
#include <stdio.h>

typedef enum {
  STATE_STOP = 0,        // 待机
  STATE_RESET = 1,       // 重置初始角度
  STATE_GO_FORWARD = 2,  // 前进
  STATE_TURN_LEFT = 3,   // 左转
  STATE_TURN_RIGNT = 4,  // 右转
  STATE_TRACK_RIGHT = 5, // 向右循迹
  STATE_TRACK_LEFT = 6,  // 向左循迹
} State_Machine_State_enum;

typedef struct {
  uint8_t state;       // 状态机状态
  uint8_t last_state;  // 上一个状态
  uint8_t state_count; // 状态计数
} State_Machine_Typedef;

State_Machine_Typedef state_machine; // 状态机结构体
static float yaw_angle = 0.0f;
static uint32_t count = 0;
//===========================================初始化函数=========================================//初始化驱动和配置
void Start_Init(void) { // 初始化所有驱动
  // 初始化延时定时器
  delay_init();
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

void State_GO_Forward_Action(void) {    // 前进 角度环模式 设置速度为 0.2 0.2
  TargetSpeedMode_Set(MODE_ANGLE_LOOP); // 角度环模式
  TargetSpeed_SetSpeed(0.25f, 0.25f); // 设置目标速度（左轮为0.12，右轮为0.12）
}

void State_Track_Right_Action(void) { // 向右循迹 循迹模式 设置速度为 0.2 0.2
  TargetSpeedMode_Set(MODE_TRACKING); // 循迹模式
  TargetSpeed_SetSpeed(0.2f, 0.2f);   // 设置目标速度（左轮为0.2，右轮为0.2）
}

void Reset_Action(void) {         // 无模式 重置初始角度
  TargetSpeedMode_Set(MODE_NONE); // 无模式
  TargetSpeed_SetStartAngle();
}

void State_Turn_Right_Action(void) {
  TargetSpeedMode_Set(MODE_ANGLE_LOOP);
  TargetSpeed_SetSpeed(0.25f, 0.25f);
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
  case STATE_GO_FORWARD:
    State_GO_Forward_Action();
    break;
  case STATE_TRACK_RIGHT:
    State_Track_Right_Action();
    break;
  case STATE_TURN_RIGNT:
    State_Turn_Right_Action();
    break;
  }
}
// 状态机状态计数更新（按照流程完成问题）
void State_Count_Updata(State_Machine_Typedef *state_machine) {

  switch (state_machine->state_count) {
    //====================待机状态====================//
  case 0:
    state_machine->state = STATE_STOP; // 待机
    if (Key_Pressed_Flag_0) {
      Key_Pressed_Flag_0 = 0; // 清除标志位
      state_machine->state_count++;
      count = 0; // 清除计数器
    }
    break;
    //====================题目1====================//
  case 1:
    Reset_Action();    // 重置初始角度
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(0.0f);            // 设置角度为0
    state_machine->state = STATE_GO_FORWARD;     // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      state_machine->state_count++;
      count = 0; // 清除计数器
    }
    break;
  case 2:
    state_machine->state = STATE_STOP; // 待机
    if (Key_Pressed_Flag_0) {
      Key_Pressed_Flag_0 = 0; // 清除标志位
      state_machine->state_count++;
    }
    break;
    //====================题目2====================//
  case 3:
    Reset_Action();    // 重置初始角度
    if (++count < 100) // 延时1000ms
      return;
    TargetSpeed_SetTargetAngle(0.0f);            // 设置角度为0
    state_machine->state = STATE_TURN_RIGNT;     // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      state_machine->state_count++;
      count = 0; // 清除计数器
    }
    break;
  case 4:
    state_machine->state = STATE_TRACK_RIGHT;     // 向右循迹
    if (!irSensor_GetSensorFlag(&irSensorData)) { // 等待出线
      state_machine->state_count++;
    }
    break;
  case 5:
    TargetSpeed_SetTargetAngle(180.0f);          // 设置角度为180
    state_machine->state = STATE_GO_FORWARD;     // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) { // 等待进线
      state_machine->state_count++;
    }
    break;
  case 6:
    state_machine->state = STATE_TRACK_RIGHT; // 向右循迹
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      state_machine->state_count++;
    }
    break;
  case 7:
    state_machine->state = STATE_STOP; // 待机
    if (Key_Pressed_Flag_1) {
      Key_Pressed_Flag_1 = 0; // 清除标志位
      state_machine->state_count++;
    }
    break;
  }
  //====================题目3====================//
  //====================题目4====================//
}
//===========================================任务调度函数==========================================//设置Task任务
void State_Update_Task(void) {
  if (state_machine.state != state_machine.last_state) {
    state_machine.last_state = state_machine.state;
    State_Update(&state_machine);
  }
}

void State_Count_Updata_Task(void) { State_Count_Updata(&state_machine); }

void PID_Speed_Updata_Task(void) {
  Speed_PID_Update(&Speed_PID_Right);
  Speed_PID_Update(&Speed_PID_Left);
}

void Channel_Grayscale_Sensor_Updata_Task(void) {
  irSensor_Update(&irSensorData);
}

void Yaw_Angle_Data_filtering(void) {
  float pitch, roll, yaw;
  mpu_dmp_get_data(&pitch, &roll, &yaw);
  yaw_angle = yaw;

  TargetSpeed_SetYawAngle(yaw_angle);
  printf("yaw_angle:%f\r\n", TargetSpeed.yaw_angle);
}
//===========================================主函数==========================================//
// pid_speed_update和encoder_update需要定期调用
int main(void) {
  // 初始化
  Start_Init();
  // 状态机初始化
  State_Machine_Init(&state_machine);
  // 添加任务
  add_task(Channel_Grayscale_Sensor_Updata_Task, 1);
  add_task(State_Update_Task, 10);
  add_task(State_Count_Updata_Task, 10);
  add_task(Encoder_Update, 2);
  add_task(PID_Speed_Updata_Task, SPEED_PID_SAMPLE_PERIOD_MS);
  add_task(TargetSpeed_Update, 2);
  add_task(Yaw_Angle_Data_filtering, 10);
  // add_task(hmc5883l_single_measurement, 5);

  while (1) {
    // 任务调度
    Task_Scheduler();
  }
}