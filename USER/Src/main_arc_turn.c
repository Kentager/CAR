#include "control.h"
#include "delay.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "main.h"
#include "motor.h"
#include "pid_angle.h"
#include "pid_speed.h"
#include "task.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
float_t pitch, roll, yaw;
uint8_t angle_changed = 0;
uint8_t speed_changed = 0;
char rx_data[256];
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
  Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.00f);

  Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.00f);
  Speed_PID_Enable(&Speed_PID_Right);
  Speed_PID_Enable(&Speed_PID_Left);
  Control_Init();
  Control_SetSpeed(0.0f);             // 设置速度 0.5 m/s
  Control_SetAngle(0.0f);             // 设置目标角度 0度
  Control_SetMode(CONTROL_MODE_TURN); // 设置模式
  Control_Enable();                   // 启用控制
}
void tChangeSpeed(void) {
  if (speed_changed) {
    Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.3);
    Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.3);

  } else {
    Speed_PID_SetTargetSpeed(&Speed_PID_Right, 0.00);
    Speed_PID_SetTargetSpeed(&Speed_PID_Left, 0.00);
  }
  speed_changed = !speed_changed;
}

void tChangeAngle(void) {
  if (!angle_changed) {
    Control_SetAngle(20.0f); // 设置目标角度 30度
    angle_changed = 1;
  } else {
    Control_SetAngle(0.0f); // 设置目标角度 0度
    angle_changed = 0;
  }
}
void tReadYaw(void) {
  if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
    printf("%.2f,%.2f,%.2f\r\n", pitch, roll, yaw);
  }
}
void tGetPID(void) {

  //   if (USART1_ReceiveByte(rx_data) == 1) {
  //     for (uint8_t i = 1; i < 11; i++) {
  //       USART1_ReceiveByte(&rx_data[i]);
  //     }
  //     float_t kp, ki, kd;
  //     kp = ((float_t *)rx_data)[0];
  //     ki = ((float_t *)rx_data)[1];
  //     kd = ((float_t *)rx_data)[2];}
  if (USART1_ReceiveLine((uint8_t *)rx_data) == 1) {
    float kp, ki, kd;
    printf("recieved:%s", rx_data);
    sscanf(rx_data, "%f,%f,%f\r\n", &kp, &ki, &kd);
    if (kp == 0 && ki == 0 && kd == 0) {
      return;
    }
    // Speed_PID_Init(&Speed_PID_Right, ENCODER_RIGHT, MOTOR_RIGHT, kp, ki, kd);
    // // // 左轮: 关联ENCODER_LEFT编码器和MOTOR_LEFT电机
    // Speed_PID_Init(&Speed_PID_Left, ENCODER_LEFT, MOTOR_LEFT, kp, ki, kd);
    // Speed_PID_Enable(&Speed_PID_Left);
    // Speed_PID_Enable(&Speed_PID_Right);
    Angle_PID_Init(&Angle_PID_Yaw, ANGLE_AXIS_YAW, kp, ki, kd);
    Angle_PID_Enable(&Angle_PID_Yaw);
    printf("PID: %.2f,%.2f,%.2f\r\n", kp, ki, kd);
  }
}

void tSpeedControl_update(void) {

  // 更新右轮速度环PID控制器
  //   printf("speed_control_task tim:%d\r\n", GetSysTick());
  // if (step % 4 == 0) {

  //   printf("%f,%f,%f,%f,%f,%f\r\n", Speed_PID_Right.target_speed_m_s,
  //          Speed_PID_Left.current_speed_m_s,
  //          Speed_PID_Right.current_speed_m_s, Speed_PID_Left.pid_state.kp,
  //          Speed_PID_Left.pid_state.ki, Speed_PID_Left.pid_state.kd);
  // }

  Speed_PID_Update(&Speed_PID_Right);

  // 更新左轮速度环PID控制器
  Speed_PID_Update(&Speed_PID_Left);
}
void tControl_updagte(void) { Control_Update(); }

void task_list(void) {
  add_task(Encoder_Update, 5);
  add_task(tSpeedControl_update, 5);
  add_task(Control_Update, 5);
  // add_task(tChangeSpeed, 800);
  // add_task(tReadYaw, 5);
  add_task(tGetPID, 20);
  add_task(tChangeAngle, 2000);
}
void PrintSystemClockFromRegisters(void) {
  uint32_t hse_value =
      8000000; // HSE 外部晶振频率，根据你的硬件修改（常见 8MHz 或 25MHz）

  // 读取 RCC 时钟配置寄存器
  uint32_t cfgr = RCC->CFGR;
  uint32_t cr = RCC->CR;
  uint32_t pllcfr = RCC->PLLCFGR;

  // 1. 确定 PLL 输入时钟源
  uint32_t pll_source =
      (pllcfr & RCC_PLLCFGR_PLLSRC) ? hse_value : 16000000; // HSE 或 HSI(16MHz)

  // 2. 获取 PLL 分频系数 M, N, P
  uint32_t pll_m = pllcfr & RCC_PLLCFGR_PLLM;
  uint32_t pll_n = (pllcfr & RCC_PLLCFGR_PLLN) >> 6;
  uint32_t pll_p = (((pllcfr & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2; // 2, 4, 6, 8

  // 3. 计算 PLL 输出时钟 (PLLCLK)
  uint32_t pllclk = (pll_source / pll_m) * pll_n / pll_p;

  // 4. 确定系统时钟源
  uint32_t sws = (cfgr & RCC_CFGR_SWS) >> 2;
  uint32_t sysclk;
  const char *clk_source_str;

  switch (sws) {
  case 0: // HSI
    sysclk = 16000000;
    clk_source_str = "HSI";
    break;
  case 1: // HSE
    sysclk = hse_value;
    clk_source_str = "HSE";
    break;
  case 2: // PLL
    sysclk = pllclk;
    clk_source_str = "PLL";
    break;
  default:
    sysclk = 16000000;
    clk_source_str = "Unknown";
    break;
  }

  // 5. 计算 AHB 预分频 (HCLK)
  uint32_t hpre = (cfgr & RCC_CFGR_HPRE) >> 4;
  uint32_t ahb_div[] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
  uint32_t hclk = sysclk / ahb_div[hpre];

  // 6. 计算 APB1 预分频 (PCLK1)
  uint32_t ppre1 = (cfgr & RCC_CFGR_PPRE1) >> 10;
  uint32_t apb1_div[] = {1, 1, 1, 1, 2, 4, 8, 16};
  uint32_t pclk1 = hclk / apb1_div[ppre1];

  // 7. 计算 APB2 预分频 (PCLK2)
  uint32_t ppre2 = (cfgr & RCC_CFGR_PPRE2) >> 13;
  uint32_t apb2_div[] = {1, 1, 1, 1, 2, 4, 8, 16};
  uint32_t pclk2 = hclk / apb2_div[ppre2];

  // 8. 打印结果
  printf("\r\n========== 系统时钟寄存器读取 ==========\r\n");
  printf("时钟源: %s\r\n", clk_source_str);
  printf("SYSCLK (系统时钟): %lu Hz (%.2f MHz)\r\n", sysclk,
         sysclk / 1000000.0f);
  printf("HCLK   (AHB总线):  %lu Hz (%.2f MHz)\r\n", hclk, hclk / 1000000.0f);
  printf("PCLK1  (APB1总线): %lu Hz (%.2f MHz)\r\n", pclk1, pclk1 / 1000000.0f);
  printf("PCLK2  (APB2总线): %lu Hz (%.2f MHz)\r\n", pclk2, pclk2 / 1000000.0f);
  printf("\r\n--- PLL 配置 ---\r\n");
  printf("PLL Source: %s (%lu Hz)\r\n",
         (pll_source == hse_value) ? "HSE" : "HSI", pll_source);
  printf("PLL M: %lu, N: %lu, P: %lu\r\n", pll_m, pll_n, pll_p);
  printf("PLLCLK: %lu Hz\r\n", pllclk);
  printf("========================================\r\n\r\n");
}

int main(void) {
  system_init();
  printf("hello,world\r\n");
  // PrintSystemClockFromRegisters();
  task_list();
  while (1) {
    Task_Scheduler();
  }
}