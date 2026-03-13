/**
 * @file kalman_filter.h
 * @brief 卡尔曼滤波姿态解算头文件 - 智能车专用
 * @version 1.0
 * @date 2026-03-13
 *
 * @details 本文件实现一维卡尔曼滤波器，用于JY61P姿态角数据滤波
 *          - 应用场景：偏航角（Yaw）、俯仰角（Pitch）、横滚角（Roll）的实时滤波
 *          - 算法：一维卡尔曼滤波器（单状态变量）
 *          - 更新频率：100Hz（与JY61P采样频率一致）
 *          - 优势：融合陀螺仪角速度和加速度计/磁力计测量，提高姿态解算精度
 *
 * @note   卡尔曼滤波步骤：
 *         1. 预测步骤（Predict）：
 *            - 状态预测：x̂(k|k-1) = x̂(k-1|k-1) + ω·dt
 *            - 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
 *         2. 更新步骤（Update）：
 *            - 卡尔曼增益：K(k) = P(k|k-1) / [P(k|k-1) + R]
 *            - 状态更新：x̂(k|k) = x̂(k|k-1) + K(k)·[z(k) - x̂(k|k-1)]
 *            - 协方差更新：P(k|k) = [1 - K(k)]·P(k|k-1)
 *         其中：
 *         - x̂(k|k)：k时刻的状态估计值
 *         - ω：陀螺仪角速度
 *         - z(k)：测量值（加速度计/磁力计）
 *         - Q：过程噪声协方差（陀螺仪噪声）
 *         - R：测量噪声协方差（加速度计+磁力计融合噪声）
 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "jy61p.h"
#include "stm32f4xx.h"

/* ==================== 卡尔曼滤波参数配置 ==================== */
// 过程噪声协方差（陀螺仪噪声）
#define KALMAN_Q_DEFAULT 0.001f  // 默认值，可根据实际调试调整

// 测量噪声协方差（加速度计+磁力计融合噪声）
#define KALMAN_R_DEFAULT 0.01f   // 默认值，可根据实际调试调整

// 初始协方差值
#define KALMAN_P_DEFAULT 1.0f   // 初始值，表示对初始状态的不确定度

// 采样周期（秒），与JY61P采样率100Hz对应
#define KALMAN_DT_DEFAULT 0.01f  // 10ms = 0.01s

/* ==================== 一维卡尔曼滤波器结构体 ==================== */
/**
 * @brief 一维卡尔曼滤波器数据结构体
 */
typedef struct {
  // 状态估计
  float x; // 当前状态估计值 x̂(k|k)

  // 协方差
  float P; // 当前协方差 P(k|k)

  // 卡尔曼滤波参数
  float Q; // 过程噪声协方差（陀螺仪噪声）
  float R; // 测量噪声协方差（加速度计+磁力计融合噪声）
  float dt;// 采样周期（秒）

  // 初始化标志
  uint8_t initialized;
} KalmanFilter_t;

/* ==================== 姿态解算结构体 ==================== */
/**
 * @brief 姿态解算数据结构体（包含原始数据和滤波后的数据）
 */
typedef struct {
  // 原始JY61P数据
  Acc_Param acc_raw;        // 原始加速度数据
  Gyro_Param gyro_raw;      // 原始角速度数据
  EulerAngle_Param euler_raw; // 原始欧拉角数据

  // 滤波后的欧拉角数据
  EulerAngle_Param euler_filtered; // 滤波后的欧拉角数据

  // 卡尔曼滤波器实例
  KalmanFilter_t kalman_yaw;   // 偏航角滤波器
  KalmanFilter_t kalman_pitch; // 俯仰角滤波器
  KalmanFilter_t kalman_roll;  // 横滚角滤波器

  // 时间戳
  uint32_t last_update_time; // 上次更新时间
} AttitudeSolver_t;

// 在头文件中声明全局姿态解算实例供外部使用
extern AttitudeSolver_t AttitudeSolver;

/* ==================== 全局函数声明 ==================== */

/**
 * @brief 初始化一维卡尔曼滤波器
 * @param kf 卡尔曼滤波器实例指针
 * @param Q 过程噪声协方差（陀螺仪噪声）
 * @param R 测量噪声协方差（加速度计+磁力计融合噪声）
 * @param dt 采样周期（秒）
 * @param initial_value 初始状态估计值
 */
void KalmanFilter_Init(KalmanFilter_t *kf, float Q, float R, float dt,
                       float initial_value);

/**
 * @brief 卡尔曼滤波器更新步骤（单次迭代）
 * @param kf 卡尔曼滤波器实例指针
 * @param measurement 测量值 z(k)
 * @param control_input 控制输入（可选，通常为陀螺仪角速度，用于预测）
 * @return 滤波后的状态估计值 x̂(k|k)
 */
float KalmanFilter_Update(KalmanFilter_t *kf, float measurement,
                           float control_input);

/**
 * @brief 重置卡尔曼滤波器
 * @param kf 卡尔曼滤波器实例指针
 * @param initial_value 初始状态估计值
 * @note 在系统复位或姿态重新校准时调用
 */
void KalmanFilter_Reset(KalmanFilter_t *kf, float initial_value);

/**
 * @brief 初始化姿态解算系统
 * @note 初始化JY61P传感器和三个卡尔曼滤波器
 */
void AttitudeSolver_Init(void);

/**
 * @brief 姿态解算更新
 * @note 从JY61P读取原始数据，应用卡尔曼滤波，输出滤波后的姿态角
 *       应在10ms定时器中断中调用（100Hz）
 */
void AttitudeSolver_Update(void);

/**
 * @brief 获取滤波后的姿态角
 * @return 滤波后的欧拉角数据结构体指针
 */
EulerAngle_Param *AttitudeSolver_GetFilteredAngle(void);

/**
 * @brief 获取原始姿态角
 * @return 原始的欧拉角数据结构体指针
 */
EulerAngle_Param *AttitudeSolver_GetRawAngle(void);

/**
 * @brief 获取加速度数据
 * @return 加速度数据结构体指针
 */
Acc_Param *AttitudeSolver_GetAcc(void);

/**
 * @brief 获取角速度数据
 * @return 角速度数据结构体指针
 */
Gyro_Param *AttitudeSolver_GetGyro(void);

#endif /* __KALMAN_FILTER_H */
