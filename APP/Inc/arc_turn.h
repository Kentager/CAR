/**
 * @file arc_turn.h
 * @brief 圆弧转弯控制模块 - 智能车专用
 * @version 1.0
 * @date 2026-04-05
 *
 * @details 本模块实现圆弧转弯功能
 *          - 根据设定半径和目标速度计算左右轮速度差
 *          - 支持顺时针和逆时针转弯
 *          - 自动适配四驱/双驱模式
 *          - 基于运动学模型计算轮速分配
 *
 * @note   运动学模型：
 *         - 设定转弯半径R，目标速度V_center
 *         - 外侧轮速度: V_outer = V_center × (R + W/2) / R
 *         - 内侧轮速度: V_inner = V_center × (R - W/2) / R
 *         - 其中W为轮距（左右轮中心距离）
 */

#ifndef __ARC_TURN_H
#define __ARC_TURN_H

#include "stm32f4xx.h"
#include "pid_speed.h"

/* ==================== 圆弧转弯参数配置 ==================== */

// 机械参数（根据实际硬件测量调整）
#define WHEEL_BASE 0.15f        // 轮距（左右轮中心距离，单位：米）
#define WHEEL_RADIUS 0.0325f    // 轮子半径（单位：米）
#define MIN_TURN_RADIUS 0.2f    // 最小转弯半径（单位：米）
#define MAX_TURN_RADIUS 10.0f   // 最大转弯半径（单位：米）

// 控制参数
#define ARC_TURN_UPDATE_PERIOD_MS 10  // 圆弧转弯更新周期（毫秒）

/* ==================== 圆弧转弯方向枚举 ==================== */
/**
 * @brief 圆弧转弯方向枚举
 */
typedef enum {
  ARC_TURN_DIR_CW = 0,    // 顺时针转弯（左轮为内侧轮）
  ARC_TURN_DIR_CCW = 1,   // 逆时针转弯（右轮为内侧轮）
  ARC_TURN_DIR_STRAIGHT = 2  // 直线行驶
} ArcTurn_Direction_e;

/* ==================== 圆弧转弯数据结构体 ==================== */
/**
 * @brief 圆弧转弯控制数据结构体
 */
typedef struct {
  // 控制参数
  float turn_radius;           // 转弯半径（单位：米）
  float center_speed_m_s;      // 中心目标速度（单位：米/秒）
  ArcTurn_Direction_e direction;  // 转弯方向

  // 计算结果
  float left_speed_m_s;        // 左轮目标速度（单位：米/秒）
  float right_speed_m_s;       // 右轮目标速度（单位：米/秒）

  // 状态信息
  uint8_t enabled;             // 是否启用圆弧转弯控制
  uint32_t last_update_time;   // 上次更新时间

  // 关联的PID控制器
  Speed_PID_Controller_t *left_pid;   // 左轮PID控制器指针
  Speed_PID_Controller_t *right_pid;  // 右轮PID控制器指针

} ArcTurn_Controller_t;

/* ==================== 全局函数声明 ==================== */

/**
 * @brief 初始化圆弧转弯控制器
 * @param controller 控制器实例指针
 * @param left_pid 左轮PID控制器指针
 * @param right_pid 右轮PID控制器指针
 */
void ArcTurn_Init(ArcTurn_Controller_t *controller,
                  Speed_PID_Controller_t *left_pid,
                  Speed_PID_Controller_t *right_pid);

/**
 * @brief 设置圆弧转弯参数
 * @param controller 控制器实例指针
 * @param radius 转弯半径（单位：米，正数为转弯半径）
 * @param center_speed 中心目标速度（单位：米/秒）
 * @param direction 转弯方向（顺时针/逆时针）
 * @note 半径为负值时表示左转，正值时表示右转
 */
void ArcTurn_SetParameters(ArcTurn_Controller_t *controller,
                           float radius, float center_speed,
                           ArcTurn_Direction_e direction);

/**
 * @brief 启用圆弧转弯控制
 * @param controller 控制器实例指针
 */
void ArcTurn_Enable(ArcTurn_Controller_t *controller);

/**
 * @brief 禁用圆弧转弯控制
 * @param controller 控制器实例指针
 */
void ArcTurn_Disable(ArcTurn_Controller_t *controller);

/**
 * @brief 更新圆弧转弯控制
 * @param controller 控制器实例指针
 * @note 应在控制循环中调用，计算并设置左右轮目标速度
 */
void ArcTurn_Update(ArcTurn_Controller_t *controller);

/**
 * @brief 获取当前计算的左右轮速度
 * @param controller 控制器实例指针
 * @param left_speed 输出左轮速度（单位：米/秒）
 * @param right_speed 输出右轮速度（单位：米/秒）
 */
void ArcTurn_GetWheelSpeeds(ArcTurn_Controller_t *controller,
                           float *left_speed, float *right_speed);

#endif /* __ARC_TURN_H */