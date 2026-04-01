#include <math.h>
#include <stdint.h>

// 定义数学常数PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 姿态解算结构体，包含滤波器状态和校准参数
 */
typedef struct {
  // --- 卡尔曼滤波器状态变量 ---
  float Xk[3];  // 后验估计状态向量 [roll, pitch, yaw] (弧度)
  float Xk_[3]; // 先验估计状态向量
  float Pk[3];  // 后验估计协方差
  float Pk_[3]; // 先验估计协方差
  float K[3];   // 卡尔曼增益
  float Uk[3];  // 系统控制量 (由陀螺仪计算的角速度)
  float Zk[3];  // 观测向量 (由加速度计和磁力计计算的角度)

  // --- 滤波器参数 ---
  float Q[3]; // 过程噪声协方差 (预测噪声，代表对陀螺仪的信任度，越小越信任)
  float R[3]; // 测量噪声协方差
              // (观测噪声，代表对加速度计/磁力计的信任度，越小越信任)
  float dt;   // 采样时间间隔 (秒)

  // --- 磁力计硬铁校准参数 ---
  // 这些参数需要根据实际环境进行8字形校准后填入
  float mag_offset[3]; // 磁场偏移量 [x, y, z]
  float mag_scale[3];  // 磁场缩放因子 [x, y, z]
} AttitudeEstimator;

// 初始化姿态解算器实例
AttitudeEstimator estimator = {
    // 初始状态设为0
    .Xk = {0, 0, 0},
    // 初始协方差设为1，表示初始不确定性较大
    .Pk = {1, 1, 1},
    // 过程噪声Q：根据陀螺仪性能调整。值越小，越信任陀螺仪的积分（短期稳定性好，但漂移慢）
    .Q = {0.001f, 0.001f, 0.001f},
    // 测量噪声R：根据传感器性能调整。
    // R[0],
    // R[1]对应加速度计，R[2]对应磁力计。值越小，越信任传感器测量值（收敛快，但抖动大）
    .R = {0.1f, 0.1f, 0.5f},
    // 采样时间，例如 500Hz 采样 -> 0.002s
    .dt = 0.002f,
    // 磁力计校准参数示例 (需替换为实际校准值)
    .mag_offset = {2.55f, -2.47f, 5.05f}, // (max + min) / 2
    .mag_scale = {1.0f, 1.0f, 1.0f}       // 2 / (max - min)
};

/**
 * @brief 姿态解算更新函数
 *
 * @param gx, gy, gz 陀螺仪原始数据 (rad/s 或 dps，需统一单位)
 * @param ax, ay, az 加速度计原始数据
 * @param mx, my, mz 磁力计原始数据
 * @param roll_out  输出的横滚角 (度)
 * @param pitch_out 输出的俯仰角 (度)
 * @param yaw_out   输出的偏航角 (度)
 */
void IMU_Attitude_Update(float gx, float gy, float gz, float ax, float ay,
                         float az, float mx, float my, float mz,
                         float *roll_out, float *pitch_out, float *yaw_out) {

  // --- 1. 磁力计数据校准 (硬铁校准) ---
  // 将原始数据减去偏移量并除以缩放因子，使数据分布在[-1, 1]附近
  float mx_cal = (mx - estimator.mag_offset[0]) * estimator.mag_scale[0];
  float my_cal = (my - estimator.mag_offset[1]) * estimator.mag_scale[1];
  float mz_cal = (mz - estimator.mag_offset[2]) * estimator.mag_scale[2];

  // --- 2. 系统输入计算 (陀螺仪角速度转欧拉角变化率) ---
  // 使用当前的姿态角 Xk 将机体坐标系的角速度转换为欧拉角的变化率
  // 注意：这里使用的是上一时刻的后验估计 Xk
  float sin_roll = sinf(estimator.Xk[0]);
  float cos_roll = cosf(estimator.Xk[0]);
  float sin_pitch = sinf(estimator.Xk[1]);
  float cos_pitch = cosf(estimator.Xk[1]);
  // 防止除以0
  if (fabsf(cos_pitch) < 0.0001f)
    cos_pitch = 0.0001f;

  estimator.Uk[0] = gx + sin_roll * tanf(estimator.Xk[1]) * gy +
                    cos_roll * tanf(estimator.Xk[1]) * gz;
  estimator.Uk[1] = cos_roll * gy - sin_roll * gz;
  estimator.Uk[2] = (sin_roll * gy + cos_roll * gz) / cos_pitch;

  // --- 3. 预测步骤 ---
  // 状态预测： Xk = Xk_prev + dt * Uk
  for (int i = 0; i < 3; i++) {
    estimator.Xk_[i] = estimator.Xk[i] + estimator.dt * estimator.Uk[i];
  }

  // 处理Yaw角在 +/- PI 处的跳变，保持连续性
  if (estimator.Xk_[2] > M_PI) {
    estimator.Xk_[2] -= 2 * M_PI;
  } else if (estimator.Xk_[2] < -M_PI) {
    estimator.Xk_[2] += 2 * M_PI;
  }

  // 协方差预测： Pk_ = Pk + Q
  for (int i = 0; i < 3; i++) {
    estimator.Pk_[i] = estimator.Pk[i] + estimator.Q[i];
  }

  // --- 4. 计算观测值 ---
  // 4.1 利用加速度计计算 Roll 和 Pitch
  // 假设机体只受重力影响，重力矢量的方向即为姿态
  estimator.Zk[0] = atan2f(ay, az);                        // Roll
  estimator.Zk[1] = atan2f(-ax, sqrtf(ay * ay + az * az)); // Pitch

  // 4.2 利用磁力计计算 Yaw
  // 需要将磁力计数据从机体坐标系转换到地理坐标系（水平坐标系）
  // 这里的转换矩阵使用了预测出的 Roll 和 Pitch 角 (Xk_)
  // 这一步是为了消除机体倾斜对航向角计算的影响
  float sin_pitch_pred = sinf(estimator.Xk_[1]);
  float cos_pitch_pred = cosf(estimator.Xk_[1]);
  float sin_roll_pred = sinf(estimator.Xk_[0]);
  float cos_roll_pred = cosf(estimator.Xk_[0]);

  // 旋转后的磁场分量
  float hx = mx_cal * cos_pitch_pred + my_cal * sin_pitch_pred * sin_roll_pred +
             mz_cal * sin_pitch_pred * cos_roll_pred;
  float hy = my_cal * cos_roll_pred - mz_cal * sin_roll_pred;
  // float hz = -mx_cal * sin_pitch_pred + my_cal * cos_pitch_pred *
  // sin_roll_pred + mz_cal * cos_pitch_pred * cos_roll_pred; //
  // 这一行对于计算航向角不是必须的

  // 计算航向角
  estimator.Zk[2] = atan2f(hy, hx);

  // --- 5. 更新步骤 (卡尔曼滤波融合) ---
  for (int i = 0; i < 3; i++) {
    // 计算卡尔曼增益： K = Pk_ / (Pk_ + R)
    estimator.K[i] = estimator.Pk_[i] / (estimator.Pk_[i] + estimator.R[i]);

    // 更新状态估计： Xk = Xk_ + K * (Zk - Xk_)
    // 注意：这里简化了观测模型，假设观测值直接对应状态
    float angle_diff = estimator.Zk[i] - estimator.Xk_[i];

    // 处理角度差在 +/- PI 处的跳变
    if (i == 2) { // 仅对Yaw角处理
      if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
      else if (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;
    }

    estimator.Xk[i] = estimator.Xk_[i] + estimator.K[i] * angle_diff;

    // 更新协方差： Pk = (1 - K) * Pk_
    estimator.Pk[i] = (1.0f - estimator.K[i]) * estimator.Pk_[i];
  }

  // --- 6. 输出结果 (弧度转角度) ---
  *roll_out = estimator.Xk[0] * 180.0f / M_PI;
  *pitch_out = estimator.Xk[1] * 180.0f / M_PI;
  *yaw_out = estimator.Xk[2] * 180.0f / M_PI;

  // 确保输出角度在 0-360 范围内 (可选)
  if (*yaw_out < 0)
    *yaw_out += 360.0f;
}
