/**
 * @file motor.h
 * @brief 四路电机PWM驱动头文件 - 智能车专用
 * @version 1.0
 * @date 2025-02-25
 *
 * @details 本驱动实现四路直流电机PWM控制功能
 *          - 驱动芯片：两路L298N（共四路H桥）
 *          - PWM输出：TIM1_CH1/CH2 (前右/前左), TIM8_CH1/CH2 (后右/后左)
 *          - PWM频率：20kHz，1000级分辨率
 *          - 方向控制：GPIO独立控制每路电机正反转
 *          - 电机命名：MOTOR_FR(前右)、MOTOR_FL(前左)、MOTOR_BR(后右)、MOTOR_BL(后左)
 *
 * @note  时钟计算：
 *        - TIM1/TIM8时钟 = APB2时钟 (168MHz)
 *        - PWM频率 = 168MHz / (PSC+1) / (ARR+1) = 20kHz
 *        - 配置：PSC=0, ARR=8399 (168MHz/1/8400 = 20kHz)
 *        - 分辨率：8400级 (0-8399)
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"
#include "HardwareConfig.h"

/* ==================== 电机枚举定义 ==================== */
/**
 * @brief 电机编号枚举（根据硬件模式自动调整）
 */
typedef enum {
#ifdef QUAD_MOTOR_DRIVE
    MOTOR_FR = 0,    // 前右电机 (Front Right)
    MOTOR_FL = 1,    // 前左电机 (Front Left)
    MOTOR_BR = 2,    // 后右电机 (Back Right)
    MOTOR_BL = 3,    // 后左电机 (Back Left)
    MOTOR_MAX = 4    // 电机总数
#else
    MOTOR_RIGHT = 0,  // 右电机 (对应前右)
    MOTOR_LEFT = 1,   // 左电机 (对应前左)
    MOTOR_MAX = 2     // 电机总数
#endif
} Motor_Id_e;

/**
 * @brief 电机方向枚举
 */
typedef enum {
    MOTOR_DIR_STOP = 0,     // 停止
    MOTOR_DIR_FORWARD = 1,  // 正转
    MOTOR_DIR_BACKWARD = 2, // 反转
    MOTOR_DIR_BRAKE = 3     // 刹车
} Motor_Direction_e;

/* ==================== PWM配置参数 ==================== */
#define MOTOR_PWM_FREQUENCY_HZ    20000   // 20kHz
#define MOTOR_PWM_PRESCALER       0       // 预分频器
#define MOTOR_PWM_PERIOD          8399    // 自动重装载值 (ARR)
#define MOTOR_PWM_MAX_DUTY        8399    // 最大占空比值

/* ==================== 前右电机 (MOTOR_FR)同时也是双驱的左电机 - TIM1_CH1 ==================== */
#define MOTOR_FR_PWM_PORT         GPIOE
#define MOTOR_FR_PWM_PIN          GPIO_Pin_9
#define MOTOR_FR_PWM_CLK          RCC_AHB1Periph_GPIOE
#define MOTOR_FR_PWM_TIM          TIM1
#define MOTOR_FR_PWM_TIM_CLK      RCC_APB2Periph_TIM1
#define MOTOR_FR_PWM_CHANNEL      TIM_Channel_1
#define MOTOR_FR_PWM_AF           GPIO_AF_TIM1

// 方向控制引脚
#define MOTOR_FR_DIR_PORT         GPIOE
#define MOTOR_FR_DIR1_PIN         GPIO_Pin_10  // IN1
#define MOTOR_FR_DIR2_PIN         GPIO_Pin_11  // IN2
#define MOTOR_FR_DIR_CLK          RCC_AHB1Periph_GPIOE

/* ==================== 前左电机 (MOTOR_FL)同时也是双驱的左电机 - TIM1_CH2 ==================== */
#define MOTOR_FL_PWM_PORT         GPIOE
#define MOTOR_FL_PWM_PIN          GPIO_Pin_11
#define MOTOR_FL_PWM_CLK          RCC_AHB1Periph_GPIOE
#define MOTOR_FL_PWM_TIM          TIM1
#define MOTOR_FL_PWM_TIM_CLK      RCC_APB2Periph_TIM1
#define MOTOR_FL_PWM_CHANNEL      TIM_Channel_2
#define MOTOR_FL_PWM_AF           GPIO_AF_TIM1

// 方向控制引脚
#define MOTOR_FL_DIR_PORT         GPIOE
#define MOTOR_FL_DIR1_PIN         GPIO_Pin_12  // IN1
#define MOTOR_FL_DIR2_PIN         GPIO_Pin_13  // IN2
#define MOTOR_FL_DIR_CLK          RCC_AHB1Periph_GPIOE

/* ==================== 后右电机 (MOTOR_BR) - TIM8_CH1 ==================== */
#ifdef QUAD_MOTOR_DRIVE
#define MOTOR_BR_PWM_PORT         GPIOC
#define MOTOR_BR_PWM_PIN          GPIO_Pin_6
#define MOTOR_BR_PWM_CLK          RCC_AHB1Periph_GPIOC
#define MOTOR_BR_PWM_TIM          TIM8
#define MOTOR_BR_PWM_TIM_CLK      RCC_APB2Periph_TIM8
#define MOTOR_BR_PWM_CHANNEL      TIM_Channel_1
#define MOTOR_BR_PWM_AF           GPIO_AF_TIM8

// 方向控制引脚
#define MOTOR_BR_DIR_PORT         GPIOC
#define MOTOR_BR_DIR1_PIN         GPIO_Pin_7   // IN1
#define MOTOR_BR_DIR2_PIN         GPIO_Pin_8   // IN2
#define MOTOR_BR_DIR_CLK          RCC_AHB1Periph_GPIOC
#endif

/* ==================== 后左电机 (MOTOR_BL) - TIM8_CH2 ==================== */
#ifdef QUAD_MOTOR_DRIVE
#define MOTOR_BL_PWM_PORT         GPIOC
#define MOTOR_BL_PWM_PIN          GPIO_Pin_7
#define MOTOR_BL_PWM_CLK          RCC_AHB1Periph_GPIOC
#define MOTOR_BL_PWM_TIM          TIM8
#define MOTOR_BL_PWM_TIM_CLK      RCC_APB2Periph_TIM8
#define MOTOR_BL_PWM_CHANNEL      TIM_Channel_2
#define MOTOR_BL_PWM_AF           GPIO_AF_TIM8

// 方向控制引脚
#define MOTOR_BL_DIR_PORT         GPIOC
#define MOTOR_BL_DIR1_PIN         GPIO_Pin_9   // IN1
#define MOTOR_BL_DIR2_PIN         GPIO_Pin_10  // IN2
#define MOTOR_BL_DIR_CLK          RCC_AHB1Periph_GPIOC
#endif

/* ==================== 电机控制类结构体 ==================== */
/**
 * @brief 电机控制类结构体（面向对象风格，代表单个电机实例）
 */
typedef struct {
    // 私有变量（带下划线前缀）
    int16_t _speed;               // 当前速度 (-MOTOR_PWM_MAX_DUTY ~ +MOTOR_PWM_MAX_DUTY)
    Motor_Direction_e _direction; // 当前方向

    // 公有变量
    Motor_Id_e id;                // 电机ID
    TIM_TypeDef *tim;             // 定时器指针
    uint16_t channel;             // PWM通道


    // 构造/析构函数（无参数，直接操作实例）
    // void (*Init)(void);
    // void (*DeInit)(void);

    // // 成员方法（无参数，直接操作实例）
    // void (*SetSpeed)(int16_t speed);      // 设置速度（带符号）
    // void (*SetDirection)(Motor_Direction_e dir); // 设置方向
    // void (*SetDuty)(uint16_t duty);       // 设置占空比
    // void (*Stop)(void);                         // 停止电机
    // void (*Brake)(void);                        // 刹车
} Motor_Class_t;


//不许直接修改，得通过函数来获得
/* ==================== 全局电机实例声明 ==================== */
// #ifdef QUAD_MOTOR_DRIVE
// // 四驱模式 - 四个独立电机实例
// extern Motor_Class_t Motor_FR;   // 前右电机实例
// extern Motor_Class_t Motor_FL;   // 前左电机实例
// extern Motor_Class_t Motor_BR;   // 后右电机实例
// extern Motor_Class_t Motor_BL;   // 后左电机实例
// #else
// // 双驱模式 - 两个独立电机实例
// extern Motor_Class_t Motor_Right; // 右电机实例（对应前右硬件）
// extern Motor_Class_t Motor_Left;  // 左电机实例（对应前左硬件）
// #endif

/* ==================== 全局函数声明 ==================== */



/**
 * @brief 设置电机速度（仅更新软件状态）
 * @param motor 电机 ID
 * @param speed 速度值 (-MOTOR_PWM_MAX_DUTY ~ +MOTOR_PWM_MAX_DUTY)
 */
void Motor_SetSpeed(Motor_Id_e motor, int16_t speed);

/**
 * @brief 设置电机方向（仅更新软件状态）
 * @param motor 电机 ID
 * @param dir 方向 (FORWARD/BACKWARD/STOP/BRAKE)
 */
void Motor_SetDirection(Motor_Id_e motor, Motor_Direction_e dir);

/**
 * @brief 更新电机硬件状态
 * @param motor 电机 ID
 * @note 根据 _speed 和 _direction 更新硬件，需配合上述函数使用
 */
void Motor_Update(Motor_Id_e motor);


/**
 * @brief 电机驱动初始化
 * @note  初始化所有电机的PWM和方向控制GPIO
 */
void Motor_Driver_Init(void);

/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void);

/**
 * @brief 刹车所有电机
 */
void Motor_BrakeAll(void);

/**
 * @brief 启动所有电机定时器
 */
void Motor_StartTimers(void);

/**
 * @brief 停止所有电机定时器
 */
void Motor_StopTimers(void);

#endif /* __MOTOR_H */
