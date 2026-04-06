/**
 * @file main_control_test.c
 * @brief 智能车控制模块测试案例
 * @version 1.0
 * @date 2026-04-06
 *
 * @details 本文件提供控制模块的完整测试案例，包括：
 *          1. 基础初始化测试
 *          2. 直线行驶测试
 *          3. 转弯控制测试
 *          4. 原地旋转测试
 *          5. 速度响应测试
 *          6. 角度控制精度测试
 */

#include "delay.h"
#include "encoder.h"
#include "led.h"
#include "motor.h"
#include "pid_speed.h"
#include "pid_angle.h"
#include "control.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include "task.h"
#include <stdio.h>
#include <math.h>

/* ==================== 测试配置 ==================== */
#define TEST_SPEED_NORMAL 0.5f      // 测试速度：正常速度 (m/s)
#define TEST_SPEED_SLOW 0.2f        // 测试速度：慢速 (m/s)
#define TEST_SPEED_FAST 0.8f        // 测试速度：快速 (m/s)

#define TEST_ANGLE_SMALL 15.0f      // 测试角度：小角度 (度)
#define TEST_ANGLE_MEDIUM 45.0f     // 测试角度：中等角度 (度)
#define TEST_ANGLE_LARGE 90.0f      // 测试角度：大角度 (度)

#define TEST_DURATION_MS 5000       // 测试持续时间 (ms)
#define TEST_PAUSE_MS 2000          // 测试间隔时间 (ms)

/* ==================== 测试状态枚举 ==================== */
typedef enum {
  TEST_STATE_IDLE = 0,        // 空闲状态
  TEST_STATE_INIT,            // 初始化测试
  TEST_STATE_STRAIGHT,        // 直线行驶测试
  TEST_STATE_TURN_SMALL,      // 小角度转弯测试
  TEST_STATE_TURN_LARGE,      // 大角度转弯测试
  TEST_STATE_SPIN,            // 原地旋转测试
  TEST_STATE_SPEED_RESPONSE,  // 速度响应测试
  TEST_STATE_ANGLE_ACCURACY,  // 角度控制精度测试
  TEST_STATE_COMPLETE         // 测试完成
} Test_State_e;

/* ==================== 全局变量 ==================== */
static Test_State_e test_state = TEST_STATE_IDLE;
static uint32_t test_start_time = 0;
static uint32_t test_current_time = 0;
static uint8_t mpu6050_dmp_ready = 0;

/* ==================== 私有函数声明 ==================== */
static void print_test_header(const char *test_name);
static void print_test_info(const char *info);
static void print_control_state(void);
static uint8_t init_mpu6050_dmp(void);
static void run_init_test(void);
static void run_straight_test(void);
static void run_turn_small_test(void);
static void run_turn_large_test(void);
static void run_spin_test(void);
static void run_speed_response_test(void);
static void run_angle_accuracy_test(void);

/* ==================== 公有函数实现 ==================== */

/**
 * @brief MPU6050 DMP初始化
 * @retval 0=成功, 非0=失败
 */
static uint8_t init_mpu6050_dmp(void) {
  uint8_t res = 0;

  printf("正在初始化MPU6050 DMP...\r\n");
  res = mpu_dmp_init();
  if (res != 0) {
    printf("MPU6050 DMP初始化失败，错误码：%d\r\n", res);
    return 1;
  }

  printf("MPU6050 DMP初始化成功！\r\n");
  return 0;
}

/**
 * @brief 打印测试标题
 * @param test_name 测试名称
 */
static void print_test_header(const char *test_name) {
  printf("\r\n========================================\r\n");
  printf("测试: %s\r\n", test_name);
  printf("========================================\r\n");
}

/**
 * @brief 打印测试信息
 * @param info 测试信息
 */
static void print_test_info(const char *info) {
  printf("[%lu ms] %s\r\n", GetSysTick(), info);
}

/**
 * @brief 打印控制状态
 */
static void print_control_state(void) {
  Control_State_t *state = Control_GetState();

  printf("控制状态:\r\n");
  printf("  模式: %d\r\n", state->mode);
  printf("  启用: %d\r\n", state->enabled);
  printf("  目标速度: %.3f m/s\r\n", state->target_speed_m_s);
  printf("  当前速度: %.3f m/s\r\n", state->current_speed_m_s);
  printf("  目标角度: %.2f 度\r\n", state->target_angle_deg);
  printf("  当前角度: %.2f 度\r\n", state->current_angle_deg);
  printf("  角度补偿: %.3f\r\n", state->angle_compensation);

#ifdef QUAD_MOTOR_DRIVE
  printf("  前右速度: %.3f m/s\r\n", state->speed_fr);
  printf("  前左速度: %.3f m/s\r\n", state->speed_fl);
  printf("  后右速度: %.3f m/s\r\n", state->speed_br);
  printf("  后左速度: %.3f m/s\r\n", state->speed_bl);
#else
  printf("  右轮速度: %.3f m/s\r\n", state->speed_right);
  printf("  左轮速度: %.3f m/s\r\n", state->speed_left);
#endif
  printf("----------------------------------------\r\n");
}

/**
 * @brief 运行初始化测试
 */
static void run_init_test(void) {
  print_test_header("初始化测试");

  // 初始化控制系统
  print_test_info("初始化控制系统...");
  Control_Init();
  print_test_info("控制系统初始化完成");

  // 检查控制状态
  Control_State_t *state = Control_GetState();
  if (state->enabled == 0 && state->mode == CONTROL_MODE_STOP) {
    print_test_info("✓ 初始化状态正确");
  } else {
    print_test_info("✗ 初始化状态错误");
  }

  // 启用控制系统
  print_test_info("启用控制系统...");
  Control_Enable();

  // 检查启用状态
  if (state->enabled == 1) {
    print_test_info("✓ 控制系统启用成功");
  } else {
    print_test_info("✗ 控制系统启用失败");
  }

  print_test_info("初始化测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行直线行驶测试
 */
static void run_straight_test(void) {
  print_test_header("直线行驶测试");

  // 设置直线行驶模式
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_SetTargetAngle(0.0f);
  print_test_info("设置模式: 直线行驶");

  // 设置速度
  Control_SetSpeed(TEST_SPEED_NORMAL);
  print_test_info("设置速度: 0.5 m/s");

  // 开始测试
  test_start_time = GetSysTick();
  print_test_info("开始直线行驶测试");

  while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
    // 使用DMP角度更新
    Control_UpdateWithDMP();

    // 每500ms打印一次状态
    if ((GetSysTick() % 500) < 10) {
      print_control_state();
    }

    delay_ms(10);
  }

  // 停止
  Control_Stop();
  print_test_info("直线行驶测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行小角度转弯测试
 */
static void run_turn_small_test(void) {
  print_test_header("小角度转弯测试");

  // 设置小角度转弯模式
  Control_SetMode(CONTROL_MODE_TURN_SMALL);
  Control_SetTurnAngle(TEST_ANGLE_SMALL);
  print_test_info("设置模式: 小角度转弯 (15度)");

  // 设置速度
  Control_SetSpeed(TEST_SPEED_NORMAL);
  print_test_info("设置速度: 0.5 m/s");

  // 启用控制
  Control_Enable();

  // 开始测试
  test_start_time = GetSysTick();
  print_test_info("开始小角度转弯测试");

  while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
    // 使用DMP角度更新
    Control_UpdateWithDMP();

    // 每500ms打印一次状态
    if ((GetSysTick() % 500) < 10) {
      print_control_state();
    }

    delay_ms(10);
  }

  // 停止
  Control_Stop();
  print_test_info("小角度转弯测试完成");
  delay_ms(TEST_PAUSE_MS);

  // 反向转弯测试
  print_test_info("开始反向小角度转弯测试 (-15度)");
  Control_SetMode(CONTROL_MODE_TURN_SMALL);
  Control_SetTurnAngle(-TEST_ANGLE_SMALL);
  Control_SetSpeed(TEST_SPEED_NORMAL);
  Control_Enable();

  test_start_time = GetSysTick();
  while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
    Control_UpdateWithDMP();
    if ((GetSysTick() % 500) < 10) {
      print_control_state();
    }
    delay_ms(10);
  }

  Control_Stop();
  print_test_info("反向小角度转弯测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行大角度转弯测试
 */
static void run_turn_large_test(void) {
  print_test_header("大角度转弯测试");

  // 设置大角度转弯模式
  Control_SetMode(CONTROL_MODE_TURN_LARGE);
  Control_SetTurnAngle(TEST_ANGLE_LARGE);
  print_test_info("设置模式: 大角度转弯 (90度)");

  // 设置速度（大角度转弯用低速）
  Control_SetSpeed(TEST_SPEED_SLOW);
  print_test_info("设置速度: 0.2 m/s");

  // 启用控制
  Control_Enable();

  // 开始测试
  test_start_time = GetSysTick();
  print_test_info("开始大角度转弯测试");

  while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
    // 使用DMP角度更新
    Control_UpdateWithDMP();

    // 每500ms打印一次状态
    if ((GetSysTick() % 500) < 10) {
      print_control_state();
    }

    delay_ms(10);
  }

  // 停止
  Control_Stop();
  print_test_info("大角度转弯测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行原地旋转测试
 */
static void run_spin_test(void) {
  print_test_header("原地旋转测试");

  // 设置原地旋转模式
  Control_SetMode(CONTROL_MODE_SPIN);
  Control_SetTurnAngle(180.0f);
  print_test_info("设置模式: 原地旋转 (180度)");

  // 设置速度
  Control_SetSpeed(TEST_SPEED_SLOW);
  print_test_info("设置速度: 0.2 m/s");

  // 启用控制
  Control_Enable();

  // 开始测试
  test_start_time = GetSysTick();
  print_test_info("开始原地旋转测试");

  while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
    // 使用DMP角度更新
    Control_UpdateWithDMP();

    // 每500ms打印一次状态
    if ((GetSysTick() % 500) < 10) {
      print_control_state();
    }

    delay_ms(10);
  }

  // 停止
  Control_Stop();
  print_test_info("原地旋转测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行速度响应测试
 */
static void run_speed_response_test(void) {
  print_test_header("速度响应测试");

  // 设置直线行驶模式
  Control_SetMode(CONTROL_MODE_STRAIGHT);
  Control_SetTargetAngle(0.0f);

  print_test_info("测试速度变化响应:");

  // 测试低速
  print_test_info("设置低速: 0.2 m/s");
  Control_SetSpeed(TEST_SPEED_SLOW);
  Control_Enable();
  delay_ms(2000);
  print_control_state();

  // 测试中速
  print_test_info("设置中速: 0.5 m/s");
  Control_SetSpeed(TEST_SPEED_NORMAL);
  delay_ms(2000);
  print_control_state();

  // 测试高速
  print_test_info("设置高速: 0.8 m/s");
  Control_SetSpeed(TEST_SPEED_FAST);
  delay_ms(2000);
  print_control_state();

  // 测试反向
  print_test_info("设置反向: -0.5 m/s");
  Control_SetSpeed(-TEST_SPEED_NORMAL);
  delay_ms(2000);
  print_control_state();

  // 停止
  Control_Stop();
  print_test_info("速度响应测试完成");
  delay_ms(TEST_PAUSE_MS);
}

/**
 * @brief 运行角度控制精度测试
 */
static void run_angle_accuracy_test(void) {
  print_test_header("角度控制精度测试");

  float target_angles[] = {10.0f, 20.0f, 30.0f, -15.0f, -30.0f};
  int num_angles = sizeof(target_angles) / sizeof(target_angles[0]);

  for (int i = 0; i < num_angles; i++) {
    char test_info[64];
    snprintf(test_info, sizeof(test_info), "测试角度: %.1f 度", target_angles[i]);
    print_test_info(test_info);

    // 设置小角度转弯模式
    Control_SetMode(CONTROL_MODE_TURN_SMALL);
    Control_SetTurnAngle(target_angles[i]);
    Control_SetSpeed(TEST_SPEED_NORMAL);
    Control_Enable();

    // 开始测试
    test_start_time = GetSysTick();
    float angle_error_sum = 0.0f;
    int sample_count = 0;

    while ((GetSysTick() - test_start_time) < TEST_DURATION_MS) {
      // 使用DMP角度更新
      Control_UpdateWithDMP();

      // 每100ms采样一次
      if ((GetSysTick() % 100) < 10) {
        Control_State_t *state = Control_GetState();
        float error = fabsf(state->target_angle_deg - state->current_angle_deg);
        angle_error_sum += error;
        sample_count++;
      }

      delay_ms(10);
    }

    // 计算平均误差
    float avg_error = angle_error_sum / sample_count;
    printf("角度 %.1f 度的平均误差: %.2f 度\r\n", target_angles[i], avg_error);

    if (avg_error < 2.0f) {
      print_test_info("✓ 角度控制精度良好");
    } else {
      print_test_info("✗ 角度控制精度需要优化");
    }

    // 停止
    Control_Stop();
    delay_ms(TEST_PAUSE_MS);
  }

  print_test_info("角度控制精度测试完成");
}

/**
 * @brief 测试主循环
 */
void control_test_main_loop(void) {
  switch (test_state) {
  case TEST_STATE_IDLE:
    // 等待MPU6050 DMP准备就绪
    if (mpu6050_dmp_ready) {
      test_state = TEST_STATE_INIT;
    }
    break;

  case TEST_STATE_INIT:
    run_init_test();
    test_state = TEST_STATE_STRAIGHT;
    break;

  case TEST_STATE_STRAIGHT:
    run_straight_test();
    test_state = TEST_STATE_TURN_SMALL;
    break;

  case TEST_STATE_TURN_SMALL:
    run_turn_small_test();
    test_state = TEST_STATE_TURN_LARGE;
    break;

  case TEST_STATE_TURN_LARGE:
    run_turn_large_test();
    test_state = TEST_STATE_SPIN;
    break;

  case TEST_STATE_SPIN:
    run_spin_test();
    test_state = TEST_STATE_SPEED_RESPONSE;
    break;

  case TEST_STATE_SPEED_RESPONSE:
    run_speed_response_test();
    test_state = TEST_STATE_ANGLE_ACCURACY;
    break;

  case TEST_STATE_ANGLE_ACCURACY:
    run_angle_accuracy_test();
    test_state = TEST_STATE_COMPLETE;
    break;

  case TEST_STATE_COMPLETE:
    print_test_header("所有测试完成");
    print_test_info("测试程序结束");
    Control_Stop();
    LED_Off();
    while (1) {
      delay_ms(1000);
    }
    break;

  default:
    break;
  }
}

/**
 * @brief 主函数
 */
int main(void) {
  // 系统初始化
  delay_init();
  USART1_Init();
  led_Init();
  LED_Off();

  // 初始化硬件
  Motor_Driver_Init();
  Encoder_Driver_Init();

  // 初始化MPU6050 DMP
  if (init_mpu6050_dmp() != 0) {
    printf("MPU6050 DMP初始化失败，无法继续测试！\r\n");
    while (1) {
      delay_ms(1000);
    }
  }
  mpu6050_dmp_ready = 1;

  // 打印测试开始信息
  printf("\r\n========================================\r\n");
  printf("智能车控制模块测试程序\r\n");
  printf("版本: 1.0\r\n");
  printf("日期: 2026-04-06\r\n");
  printf("========================================\r\n");

  // 初始化任务调度系统
  Task_Init();

  // 添加测试任务
  add_task(control_test_main_loop, 10);  // 10ms周期

  // 主循环
  while (1) {
    Task_Scheduler();
  }
}
