#include "channel_grayscale_sensor.h"
#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "pid_angle.h"
#include "pid_speed.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_syscfg.h"
#include "task.h"
#include "usart.h"

typedef enum {
  STATE_STOP = 0,        // 待机
  STATE_RESET = 1,       // 重置
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

//===========================================初始化函数=========================================//初始化驱动和配置
void Start_Init(void) {
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
}

void State_Machine_Init(State_Machine_Typedef *state_machine) {
  state_machine->state = STATE_STOP;
  state_machine->last_state = STATE_STOP;
  state_machine->state_count = 0;
}

//===========================================指令函数==========================================//设置小车的速度以及循迹模块和灰度模块的开关
void State_Stop_Action(void) {
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.0f); // 设置右轮速度环目标速度为0
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.0f);  // 设置左轮速度环目标速度为0
}

void State_GO_Forward_Action(void) {
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.12f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.12f);
}

void State_Track_Left_Action(void) {
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.1f);
  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.138f);
}
//===========================================状态更新函数==========================================//检测状态更新，调用指令函数
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
  }
}

void State_Count_Updata(State_Machine_Typedef *state_machine) {
  switch (state_machine->state_count) {
  case 0:
    state_machine->state = STATE_STOP; // 待机
    if (Key_Pressed_Flag) {
      Key_Pressed_Flag = 0; // 清除标志位
      state_machine->state_count++;
    }
    break;
  case 1:
    state_machine->state = STATE_GO_FORWARD; // 前进
    if (irSensor_GetSensorFlag(&irSensorData)) {
      state_machine->state_count++;
    }
    break;
  case 2:
    state_machine->state = STATE_TRACK_RIGHT; // 向右循迹
    if (!irSensor_GetSensorFlag(&irSensorData)) {
      state_machine->state_count++;
    }
  case 3:
    state_machine->state = STATE_STOP; // 待机
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

void PID_Speed_Updata_Task(void) {
  Speed_PID_Update(&Speed_PID_Right);
  Speed_PID_Update(&Speed_PID_Left);
}

void Channel_Grayscale_Sensor_Updata_Task(void) {
  irSensor_Update(&irSensorData);
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

  while (1) {
    // 任务调度
    Task_Scheduler();
  }
}