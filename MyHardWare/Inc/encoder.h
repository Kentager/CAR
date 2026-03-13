/**
 * @file encoder.h
 * @brief 编码器驱动头文件 - 智能车专用（面向对象风格）
 * @version 2.0
 * @date 2026-03-04
 *
 * @details 本驱动实现正交编码器接口功能
 *          - 编码器模式：TIM2/TIM3/TIM4/TIM5 正交解码模式
 *          - 编码器类型：11线编码器，减速比1:30
 *          - 计数范围：0-65535（16位定时器）
 *          - 功能：测速、里程计算、方向检测
 *
 * @note  正交解码模式配置：
 *        - TI1FP1和TI2FP2连接到TIMx_CH1和TIMx_CH2
 *        - 上升沿和下降沿都计数（4倍频）
 *        - 自动方向检测
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "HardwareConfig.h"
#include "stm32f4xx.h"
#include "task.h"

/* ==================== 编码器枚举定义 ==================== */
/**
 * @brief 编码器编号枚举（根据硬件模式自动调整）
 */
typedef enum {
#ifdef QUAD_MOTOR_DRIVE
  ENCODER_FR = 0, // 前右电机编码器 (Front Right) - TIM2
  ENCODER_FL = 1, // 前左电机编码器 (Front Left) - TIM3
  ENCODER_BR = 2, // 后右电机编码器 (Back Right) - TIM4
  ENCODER_BL = 3, // 后左电机编码器 (Back Left) - TIM5
  ENCODER_MAX = 4 // 编码器总数
#else
  ENCODER_RIGHT = 0, // 右电机编码器 (对应前右) - TIM2
  ENCODER_LEFT = 1,  // 左电机编码器 (对应前左) - TIM5
  ENCODER_MAX = 2    // 编码器总数
#endif
} Encoder_Id_e;

/**
 * @brief 编码器方向枚举
 */
typedef enum {
  ENCODER_DIR_STOPPED = 0, // 停止
  ENCODER_DIR_FORWARD = 1, // 正转（计数增加）
  ENCODER_DIR_BACKWARD = 2 // 反转（计数减少）
} Encoder_Direction_e;

/* ==================== 编码器参数配置 ==================== */
// 编码器硬件参数
#define ENCODER_LINES 500          // 编码器线数
#define ENCODER_REDUCTION_RATIO 28 // 减速比
#define ENCODER_PPR                                                            \
  (ENCODER_LINES * 2 * ENCODER_REDUCTION_RATIO) // 每转脉冲数 (500*2*28=2800)

// 轮子参数
#define WHEEL_DIAMETER 0.065f // 轮子直径 65mm（根据实际修改）
#define GEAR_RATIO 1.0f       // 齿轮比（根据实际修改）

// 测速相关参数
#define ENCODER_SAMPLE_PERIOD_MS 5 // 采样周期 (ms)
#define ENCODER_SAMPLE_FREQ_HZ 200 // 采样频率 (Hz) (Hz)

// 速度计算相关
#define RPM_TO_RPS (1.0f / 60.0f)             // RPM转RPS
#define PULSE_TO_RAD (6.28318f / ENCODER_PPR) // 脉冲转弧度

/* ==================== 右编码器 (ENCODER_RIGHT) - TIM5 ==================== */
// 双驱模式下使用 TIM5（32 位定时器）
#define ENCODER_RIGHT_TIM TIM5
#define ENCODER_RIGHT_TIM_CLK RCC_APB1Periph_TIM5
#define ENCODER_RIGHT_CH1_PORT GPIOA
#define ENCODER_RIGHT_CH1_PIN GPIO_Pin_0 // TIM5_CH1
#define ENCODER_RIGHT_CH2_PORT GPIOA
#define ENCODER_RIGHT_CH2_PIN GPIO_Pin_1 // TIM5_CH2
#define ENCODER_RIGHT_GPIO_CLK RCC_AHB1Periph_GPIOA
#define ENCODER_RIGHT_AF GPIO_AF_TIM5

/* ==================== 左编码器 (ENCODER_LEFT) - TIM5 ==================== */
// 双驱模式下使用 TIM2（32 位定时器）
#define ENCODER_LEFT_TIM TIM2
#define ENCODER_LEFT_TIM_CLK RCC_APB1Periph_TIM2
#define ENCODER_LEFT_CH1_PORT GPIOA
#define ENCODER_LEFT_CH1_PIN GPIO_Pin_5 // TIM2_CH1
#define ENCODER_LEFT_CH2_PORT GPIOB
#define ENCODER_LEFT_CH2_PIN GPIO_Pin_3 // TIM2_CH2
#define ENCODER_LEFT_GPIO_CLK (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define ENCODER_LEFT_AF GPIO_AF_TIM2

/* ==================== 四驱模式专用编码器定义 ==================== */
#ifdef QUAD_MOTOR_DRIVE
/* ==================== 前右编码器 (ENCODER_FR) - TIM2 ==================== */
#define ENCODER_FR_TIM TIM2
#define ENCODER_FR_TIM_CLK RCC_APB1Periph_TIM2
#define ENCODER_FR_CH1_PORT GPIOA
#define ENCODER_FR_CH1_PIN GPIO_Pin_15 // TIM2_CH1
#define ENCODER_FR_CH2_PORT GPIOB
#define ENCODER_FR_CH2_PIN GPIO_Pin_3 // TIM2_CH2
#define ENCODER_FR_GPIO_CLK (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define ENCODER_FR_AF GPIO_AF_TIM2

/* ==================== 前左编码器 (ENCODER_FL) - TIM3 ==================== */
#define ENCODER_FL_TIM TIM3
#define ENCODER_FL_TIM_CLK RCC_APB1Periph_TIM3
#define ENCODER_FL_CH1_PORT GPIOA
#define ENCODER_FL_CH1_PIN GPIO_Pin_6 // TIM3_CH1
#define ENCODER_FL_CH2_PORT GPIOA
#define ENCODER_FL_CH2_PIN GPIO_Pin_7 // TIM3_CH2
#define ENCODER_FL_GPIO_CLK RCC_AHB1Periph_GPIOA
#define ENCODER_FL_AF GPIO_AF_TIM3

/* ==================== 后右编码器 (ENCODER_BR) - TIM4 ==================== */
#define ENCODER_BR_TIM TIM4
#define ENCODER_BR_TIM_CLK RCC_APB1Periph_TIM4
#define ENCODER_BR_CH1_PORT GPIOD
#define ENCODER_BR_CH1_PIN GPIO_Pin_12 // TIM4_CH1
#define ENCODER_BR_CH2_PORT GPIOD
#define ENCODER_BR_CH2_PIN GPIO_Pin_13 // TIM4_CH2
#define ENCODER_BR_GPIO_CLK RCC_AHB1Periph_GPIOD
#define ENCODER_BR_AF GPIO_AF_TIM4

/* ==================== 后左编码器 (ENCODER_BL) - TIM5 ==================== */
#define ENCODER_BL_TIM TIM5
#define ENCODER_BL_TIM_CLK RCC_APB1Periph_TIM5
#define ENCODER_BL_CH1_PORT GPIOH
#define ENCODER_BL_CH1_PIN GPIO_Pin_10 // TIM5_CH1
#define ENCODER_BL_CH2_PORT GPIOH
#define ENCODER_BL_CH2_PIN GPIO_Pin_11 // TIM5_CH2
#define ENCODER_BL_GPIO_CLK RCC_AHB1Periph_GPIOH
#define ENCODER_BL_AF GPIO_AF_TIM5
#endif

/* ==================== 编码器数据结构体 ==================== */
/**
 * @brief 编码器数据结构体（单个编码器的数据）
 */
typedef struct {
  // 当前状态
  int32_t count;       // 当前计数值 (支持 32 位定时器)
  int32_t last_count;  // 上次计数值
  int32_t delta_count; // 计数增量

  // 速度相关
  float speed_rpm;   // 转速 (RPM)
  float speed_rps;   // 转速 (RPS)
  float speed_rad_s; // 角速度 (rad/s)
  float speed_m_s;   // 线速度 (m/s)

  // 里程相关
  int32_t total_count;  // 总计数值（累计）
  float total_distance; // 总里程 (m)

  // 方向
  Encoder_Direction_e direction; // 当前方向

  // 参数
  float wheel_diameter; // 轮子直径 (m)
  float gear_ratio;     // 齿轮比

  // 时间戳
  uint32_t last_update_time; // 上次更新时间
} Encoder_Data_t;

/* ==================== 编码器类结构体 ==================== */
/**
 * @brief 编码器控制类结构体（面向对象风格，代表单个编码器实例）
 */
typedef struct {
  // 私有变量（带下划线前缀）
  Encoder_Data_t _data; // 编码器数据
  uint8_t _initialized; // 初始化标志

  // 公有变量
  Encoder_Id_e id;  // 编码器ID
  TIM_TypeDef *tim; // 定时器指针

} Encoder_Class_t;

/* ==================== 全局编码器实例声明 ==================== */
#ifdef QUAD_MOTOR_DRIVE
// 四驱模式 - 四个独立编码器实例
extern Encoder_Class_t Encoder_FR; // 前右编码器实例
extern Encoder_Class_t Encoder_FL; // 前左编码器实例
extern Encoder_Class_t Encoder_BR; // 后右编码器实例
extern Encoder_Class_t Encoder_BL; // 后左编码器实例
#else
// 双驱模式 - 两个独立编码器实例
// extern Encoder_Class_t Encoder_Right; // 右编码器实例（对应前右硬件）
// extern Encoder_Class_t Encoder_Left;  // 左编码器实例（对应前左硬件）
#endif

/* ==================== 全局函数声明 ==================== */

/**
 * @brief 编码器驱动初始化
 * @note  初始化所有编码器实例的函数指针
 */
void Encoder_Driver_Init(void);

Encoder_Data_t Encoder_GetData(Encoder_Id_e encoder);
void Encoder_Update(void);
#endif /* __ENCODER_H */
