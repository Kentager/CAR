/**
 * @file HardwareConfig.h
 * @brief 智能车硬件配置文件 - 四驱/双驱切换
 * @version 1.0
 * @date 2026-03-04
 *
 * @details 本文件用于配置智能车硬件架构
 *          - 通过定义/注释宏来切换四驱和双驱模式
 *          - 四驱模式：使用4路电机 + 4路编码器（TIM1/TIM8 +
 * TIM2/TIM3/TIM4/TIM5）
 *          - 双驱模式：使用2路电机 + 2路编码器（仅使用前轮：TIM1_CH1/CH2 +
 * TIM2/TIM3）
 *
 * @note   切换方法：
 *         - 四驱模式：定义 QUAD_MOTOR_DRIVE 宏（取消下面宏定义的注释）
 *         - 双驱模式：注释掉 QUAD_MOTOR_DRIVE 宏
 *
 * @example 四驱模式：
 *          #define QUAD_MOTOR_DRIVE
 *
 *          双驱模式：
 *          // #define QUAD_MOTOR_DRIVE
 */

#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

/* ==================== 驱动模式选择 ==================== */
/**
 * @brief 四驱模式开关
 * @note  取消注释以启用四驱模式，注释掉则为双驱模式
 */
// #define QUAD_MOTOR_DRIVE

/* ==================== 根据驱动模式自动配置 ==================== */

#ifdef QUAD_MOTOR_DRIVE
// 四驱模式配置
#define MOTOR_COUNT 4    // 电机数量
#define ENCODER_COUNT 4  // 编码器数量
#define MOTOR_MAX_ID 3   // 电机最大ID (0-3)
#define ENCODER_MAX_ID 3 // 编码器最大ID (0-3)

#define ENABLE_TIM8 1 // 使能TIM8（后轮电机PWM）
#define ENABLE_TIM4 1 // 使能TIM4（后右编码器）
#define ENABLE_TIM5 1 // 使能TIM5（后左编码器）

// 驱动芯片说明
#define DRIVE_CHIP_DESC "TB6612 x2 (四路H桥)"

#else
// 双驱模式配置（仅前轮）
#define MOTOR_COUNT 2    // 电机数量
#define ENCODER_COUNT 2  // 编码器数量
#define MOTOR_MAX_ID 1   // 电机最大ID (0-1)
#define ENCODER_MAX_ID 1 // 编码器最大ID (0-1)

#define ENABLE_TIM8 0 // 禁用TIM8（后轮电机PWM）
#define ENABLE_TIM4 0 // 禁用TIM4（后右编码器）
#define ENABLE_TIM5 0 // 禁用TIM5（后左编码器）

// 驱动芯片说明
#define DRIVE_CHIP_DESC "TB6612 x1 (双路H桥)"

#endif

/* ==================== 电机编号枚举（根据模式调整） ==================== */
// #ifdef QUAD_MOTOR_DRIVE
//     // 四驱模式
//     #define MOTOR_FR              0        // 前右电机
//     #define MOTOR_FL              1        // 前左电机
//     #define MOTOR_BR              2        // 后右电机
//     #define MOTOR_BL              3        // 后左电机
// #else
//     // 双驱模式
//     #define MOTOR_RIGHT            0        // 右电机
//     #define MOTOR_LEFT             1        // 左电机
// #endif

/* ==================== 编码器编号枚举（根据模式调整） ==================== */
// #ifdef QUAD_MOTOR_DRIVE
// // 四驱模式
// #define ENCODER_FR 0 // 前右编码器
// #define ENCODER_FL 1 // 前左编码器
// #define ENCODER_BR 2 // 后右编码器
// #define ENCODER_BL 3 // 后左编码器
// #else
// // 双驱模式
// #define ENCODER_RIGHT 0 // 右编码器
// #define ENCODER_LEFT 1  // 左编码器
// #endif

/* ==================== 调试信息宏 ==================== */
#ifdef QUAD_MOTOR_DRIVE
#define HARDWARE_MODE_STR "四轮驱动 (4WD)"
#else
#define HARDWARE_MODE_STR "双轮驱动 (2WD)"
#endif

#endif /* __HARDWARE_CONFIG_H */
