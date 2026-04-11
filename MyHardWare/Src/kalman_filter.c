// /**
//  * @file kalman_filter.c
//  * @brief 卡尔曼滤波姿态解算实现 - 智能车专用
//  * @version 1.0
//  * @date 2026-03-13
//  */

// #include "kalman_filter.h"
// #include "delay.h"
// #include "hmc5883l.h"
// #include "inv_mpu.h"
// #include "jy61p.h"
// #include "mpu6050.h"
// #include "task.h"
// #include <string.h>

// /* ==================== 全局变量定义 ==================== */
// AttitudeSolver_t AttitudeSolver; // 全局姿态解算实例

// /* ==================== 公有函数实现 ==================== */

// /**
//  * @brief 初始化一维卡尔曼滤波器
//  * @param kf 卡尔曼滤波器实例指针
//  * @param Q 过程噪声协方差（陀螺仪噪声）
//  * @param R 测量噪声协方差（加速度计+磁力计融合噪声）
//  * @param dt 采样周期（秒）
//  * @param initial_value 初始状态估计值
//  */
// void KalmanFilter_Init(KalmanFilter_t *kf, float Q, float R, float dt,
//                        float initial_value) {
//   // 参数检查
//   if (kf == NULL) {
//     return;
//   }

//   // 初始化状态估计值
//   kf->x = initial_value;

//   // 初始化协方差
//   kf->P = KALMAN_P_DEFAULT;

//   // 初始化卡尔曼滤波参数
//   kf->Q = Q;
//   kf->R = R;
//   kf->dt = dt;

//   // 标记为已初始化
//   kf->initialized = 1;
// }

// /**
//  * @brief 卡尔曼滤波器更新步骤（单次迭代）
//  * @param kf 卡尔曼滤波器实例指针
//  * @param measurement 测量值 z(k)
//  * @param control_input 控制输入（可选，通常为陀螺仪角速度，用于预测）
//  * @return 滤波后的状态估计值 x̂(k|k)
//  *
//  * @note 卡尔曼滤波步骤：
//  *       1. 预测步骤（Predict）：
//  *          - 状态预测：x̂(k|k-1) = x̂(k-1|k-1) + ω·dt
//  *          - 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
//  *       2. 更新步骤（Update）：
//  *          - 卡尔曼增益：K(k) = P(k|k-1) / [P(k|k-1) + R]
//  *          - 状态更新：x̂(k|k) = x̂(k|k-1) + K(k)·[z(k) - x̂(k|k-1)]
//  *          - 协方差更新：P(k|k) = [1 - K(k)]·P(k|k-1)
//  */
// float KalmanFilter_Update(KalmanFilter_t *kf, float measurement,
//                           float control_input) {
//   // 参数检查
//   if (kf == NULL || !kf->initialized) {
//     return 0.0f;
//   }

//   // ============ 预测步骤（Predict） ============
//   // 状态预测：x̂(k|k-1) = x̂(k-1|k-1) + ω·dt
//   float x_pred = kf->x + control_input * kf->dt;

//   // 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
//   float P_pred = kf->P + kf->Q;

//   // ============ 更新步骤（Update） ============
//   // 卡尔曼增益：K(k) = P(k|k-1) / [P(k|k-1) + R]
//   float K = P_pred / (P_pred + kf->R);

//   // 状态更新：x̂(k|k) = x̂(k|k-1) + K(k)·[z(k) - x̂(k|k-1)]
//   kf->x = x_pred + K * (measurement - x_pred);

//   // 协方差更新：P(k|k) = [1 - K(k)]·P(k|k-1)
//   kf->P = (1.0f - K) * P_pred;

//   return kf->x;
// }

// /**
//  * @brief 重置卡尔曼滤波器
//  * @param kf 卡尔曼滤波器实例指针
//  * @param initial_value 初始状态估计值
//  * @note 在系统复位或姿态重新校准时调用
//  */
// void KalmanFilter_Reset(KalmanFilter_t *kf, float initial_value) {
//   // 参数检查
//   if (kf == NULL) {
//     return;
//   }

//   // 重置状态估计值
//   kf->x = initial_value;

//   // 重置协方差
//   kf->P = KALMAN_P_DEFAULT;

//   // 不重置Q、R、dt参数，保持滤波器特性
// }

// /**
//  * @brief 初始化姿态解算系统
//  * @note 初始化JY61P传感器和三个卡尔曼滤波器
//  */
// void AttitudeSolver_Init(void) {
//   // 初始化JY61P传感器
//   // JY61p_Init();
//   MPU_Init();
//   hmc5883l_init();

//   // 初始化三个卡尔曼滤波器（偏航角、俯仰角、横滚角）
//   KalmanFilter_Init(&AttitudeSolver.kalman_yaw, KALMAN_Q_DEFAULT,
//                     KALMAN_R_DEFAULT, KALMAN_DT_DEFAULT, 0.0f); // 偏航角
//   KalmanFilter_Init(&AttitudeSolver.kalman_pitch, KALMAN_Q_DEFAULT,
//                     KALMAN_R_DEFAULT, KALMAN_DT_DEFAULT, 0.0f); // 俯仰角
//   KalmanFilter_Init(&AttitudeSolver.kalman_roll, KALMAN_Q_DEFAULT,
//                     KALMAN_R_DEFAULT, KALMAN_DT_DEFAULT, 0.0f); // 横滚角

//   // 清零原始数据
//   memset(&AttitudeSolver.acc_raw, 0, sizeof(Acc_Param));
//   memset(&AttitudeSolver.gyro_raw, 0, sizeof(Gyro_Param));
//   memset(&AttitudeSolver.euler_raw, 0, sizeof(EulerAngle_Param));
//   memset(&AttitudeSolver.euler_filtered, 0, sizeof(EulerAngle_Param));

//   // 初始化时间戳
//   AttitudeSolver.last_update_time = GetSysTick();

//   // 读取一次初始数据，初始化滤波器
//   AttitudeSolver_Update();
// }
// /**
//  * @brief 计算航向角
//  * @param roll 当前横滚角(弧度)
//  * @param pitch 当前俯仰角(弧度)
//  * @return 航向角(度)
//  */
// float AttitudeSolver_ComputeHeading(float roll, float pitch) {
//   // 获取磁力计数据
//   HMC5883L_Data_t *mag_data = hmc5883l_get_data();

//   // 将磁力计数据从机体坐标系转换到地理坐标系
//   float sin_roll = sinf(roll);
//   float cos_roll = cosf(roll);
//   float sin_pitch = sinf(pitch);
//   float cos_pitch = cosf(pitch);

//   // 旋转后的磁场分量
//   float hx = mag_data->x * cos_pitch + mag_data->y * sin_pitch * sin_roll +
//              mag_data->z * sin_pitch * cos_roll;
//   float hy = mag_data->y * cos_roll - mag_data->z * sin_roll;

//   // 计算航向角(弧度)
//   float heading_rad = atan2f(hy, hx);

//   // 转换为角度
//   float heading_deg = heading_rad * 180.0f / M_PI;

//   // 确保航向角在0-360度范围内
//   if (heading_deg < 0) {
//     heading_deg += 360.0f;
//   }

//   return heading_deg;
// }

// /**
//  * @brief 姿态解算更新
//  * @note 从JY61P读取原始数据，应用卡尔曼滤波，输出滤波后的姿态角
//  *       应在10ms定时器中断中调用（100Hz）
//  */
// void AttitudeSolver_Update(void) {
//   // 从JY61P读取原始数据
//   JY61p_Get(&AttitudeSolver.acc_raw, &AttitudeSolver.euler_raw,
//             &AttitudeSolver.gyro_raw);
//   // 计算横滚角和俯仰角(使用加速度计)
//   MPU_Updata();
//   AttitudeSolver.acc_raw.Ax = raw_data[0];
//   AttitudeSolver.acc_raw.Ay = raw_data[1];
//   AttitudeSolver.acc_raw.Az = raw_data[2];
//   AttitudeSolver.gyro_raw.Gx = raw_data[3];
//   AttitudeSolver.gyro_raw.Gy = raw_data[4];
//   AttitudeSolver.gyro_raw.Gz = raw_data[5];
//   // 这里没有获取欧拉角数据
//   float roll_rad = atan2f(AttitudeSolver.acc_raw.Ay,
//   AttitudeSolver.acc_raw.Az); float pitch_rad =
//       atan2f(-AttitudeSolver.acc_raw.Ax,
//              sqrtf(AttitudeSolver.acc_raw.Ay * AttitudeSolver.acc_raw.Ay +
//                    AttitudeSolver.acc_raw.Az * AttitudeSolver.acc_raw.Az));

//   //
//   俯仰角：使用陀螺仪角速度（Gy）作为控制输入进行预测，使用计算出的俯仰角作为测量值
//   AttitudeSolver.euler_filtered.Pitch = KalmanFilter_Update(
//       &AttitudeSolver.kalman_pitch, pitch_rad * 180.0f / M_PI,
//       AttitudeSolver.gyro_raw.Gy);

//   //
//   横滚角：使用陀螺仪角速度（Gx）作为控制输入进行预测，使用计算出的横滚角作为测量值
//   AttitudeSolver.euler_filtered.Roll =
//       KalmanFilter_Update(&AttitudeSolver.kalman_roll, roll_rad * 180.0f /
//       M_PI,
//                           AttitudeSolver.gyro_raw.Gx);
//   // 读取磁力计数据
//   hmc5883l_read_data(&AttitudeSolver.mag_data);

//   // 计算航向角(使用磁力计)
//   float heading_deg = AttitudeSolver_ComputeHeading(
//       AttitudeSolver.euler_filtered.Roll * M_PI / 180.0f,
//       AttitudeSolver.euler_filtered.Pitch * M_PI / 180.0f);
//   //
//   偏航角：使用陀螺仪角速度（Gz）作为控制输入进行预测，使用计算出的航向角作为测量值
//   AttitudeSolver.euler_filtered.Yaw = KalmanFilter_Update(
//       &AttitudeSolver.kalman_yaw, heading_deg, AttitudeSolver.gyro_raw.Gz);

//   // 应用卡尔曼滤波到各个角度
//   // 更新时间戳
//   AttitudeSolver.last_update_time = GetSysTick();
// }

// /**
//  * @brief 获取滤波后的姿态角
//  * @return 滤波后的欧拉角数据结构体指针
//  */
// EulerAngle_Param *AttitudeSolver_GetFilteredAngle(void) {
//   return &AttitudeSolver.euler_filtered;
// }

// /**
//  * @brief 获取原始姿态角
//  * @return 原始的欧拉角数据结构体指针
//  */
// EulerAngle_Param *AttitudeSolver_GetRawAngle(void) {
//   return &AttitudeSolver.euler_raw;
// }

// /**
//  * @brief 获取加速度数据
//  * @return 加速度数据结构体指针
//  */
// Acc_Param *AttitudeSolver_GetAcc(void) { return &AttitudeSolver.acc_raw; }

// /**
//  * @brief 获取角速度数据
//  * @return 角速度数据结构体指针
//  */
