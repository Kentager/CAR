/**
 * @file encoder.c
 * @brief 编码器驱动实现文件 - 智能车专用（面向对象风格）
 * @version 2.0
 * @date 2026-03-04
 */

#include "encoder.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

/* ==================== 全局编码器实例定义 ==================== */
// 四驱模式编码器实例
#ifdef QUAD_MOTOR_DRIVE
Encoder_Class_t Encoder_FR = {0};
Encoder_Class_t Encoder_FL = {0};
Encoder_Class_t Encoder_BR = {0};
Encoder_Class_t Encoder_BL = {0};
#else
// 双驱模式编码器实例
Encoder_Class_t Encoder_Right = {0};
Encoder_Class_t Encoder_Left = {0};
#endif

/* ==================== 私有函数声明 ==================== */
static void Encoder_GPIO_Init(void);
static void Encoder_TIM_Init(void);
static void Encoder_Update_Data(Encoder_Class_t *inst);

/* ==================== 私有函数：单个实例操作 ==================== */

/**
 * @brief 前右编码器实例操作
 */
static void Encoder_FR_Init(void) {
  Encoder_GPIO_Init(ENCODER_FR);
  Encoder_FR._data.last_update_time = 0;
  Encoder_FR._initialized = 1;
}
static void Encoder_FR_DeInit(void) { Encoder_FR._initialized = 0; }
static int16_t Encoder_FR_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_FR_TIM);
}
static int32_t Encoder_FR_GetTotalCount(void) {
  return Encoder_FR._data.total_count;
}
static float Encoder_FR_GetSpeedRPM(void) { return Encoder_FR._data.speed_rpm; }
static float Encoder_FR_GetSpeedRPS(void) { return Encoder_FR._data.speed_rps; }
static float Encoder_FR_GetSpeedRadS(void) {
  return Encoder_FR._data.speed_rad_s;
}
static float Encoder_FR_GetSpeedMS(void) { return Encoder_FR._data.speed_m_s; }
static float Encoder_FR_GetDistance(void) {
  return Encoder_FR._data.total_distance;
}
static Encoder_Direction_e Encoder_FR_GetDirection(void) {
  return Encoder_FR._data.direction;
}
static void Encoder_FR_Update(void) { Encoder_Update_Data(&Encoder_FR); }
static void Encoder_FR_Reset(void) {
  Encoder_FR._data.count = 0;
  Encoder_FR._data.last_count = 0;
  Encoder_FR._data.delta_count = 0;
  Encoder_FR._data.speed_rpm = 0;
  Encoder_FR._data.speed_rps = 0;
  Encoder_FR._data.speed_rad_s = 0;
  Encoder_FR._data.speed_m_s = 0;
  Encoder_FR._data.total_count = 0;
  Encoder_FR._data.total_distance = 0;
  Encoder_FR._data.direction = ENCODER_DIR_STOPPED;
}

/**
 * @brief 前左编码器实例操作
 */
static void Encoder_FL_Init(void) {
  Encoder_GPIO_Init(ENCODER_FL);
  Encoder_FL._data.last_update_time = 0;
  Encoder_FL._initialized = 1;
}
static void Encoder_FL_DeInit(void) { Encoder_FL._initialized = 0; }
static int16_t Encoder_FL_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_FL_TIM);
}
static int32_t Encoder_FL_GetTotalCount(void) {
  return Encoder_FL._data.total_count;
}
static float Encoder_FL_GetSpeedRPM(void) { return Encoder_FL._data.speed_rpm; }
static float Encoder_FL_GetSpeedRPS(void) { return Encoder_FL._data.speed_rps; }
static float Encoder_FL_GetSpeedRadS(void) {
  return Encoder_FL._data.speed_rad_s;
}
static float Encoder_FL_GetSpeedMS(void) { return Encoder_FL._data.speed_m_s; }
static float Encoder_FL_GetDistance(void) {
  return Encoder_FL._data.total_distance;
}
static Encoder_Direction_e Encoder_FL_GetDirection(void) {
  return Encoder_FL._data.direction;
}
static void Encoder_FL_Update(void) { Encoder_Update_Data(&Encoder_FL); }
static void Encoder_FL_Reset(void) {
  Encoder_FL._data.count = 0;
  Encoder_FL._data.last_count = 0;
  Encoder_FL._data.delta_count = 0;
  Encoder_FL._data.speed_rpm = 0;
  Encoder_FL._data.speed_rps = 0;
  Encoder_FL._data.speed_rad_s = 0;
  Encoder_FL._data.speed_m_s = 0;
  Encoder_FL._data.total_count = 0;
  Encoder_FL._data.total_distance = 0;
  Encoder_FL._data.direction = ENCODER_DIR_STOPPED;
}

#ifdef QUAD_MOTOR_DRIVE

/**
 * @brief 后右编码器实例操作
 */
static void Encoder_BR_Init(void) {
  Encoder_GPIO_Init(ENCODER_BR);
  Encoder_BR._data.last_update_time = 0;
  Encoder_BR._initialized = 1;
}
static void Encoder_BR_DeInit(void) { Encoder_BR._initialized = 0; }
static int16_t Encoder_BR_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_BR_TIM);
}
static int32_t Encoder_BR_GetTotalCount(void) {
  return Encoder_BR._data.total_count;
}
static float Encoder_BR_GetSpeedRPM(void) { return Encoder_BR._data.speed_rpm; }
static float Encoder_BR_GetSpeedRPS(void) { return Encoder_BR._data.speed_rps; }
static float Encoder_BR_GetSpeedRadS(void) {
  return Encoder_BR._data.speed_rad_s;
}
static float Encoder_BR_GetSpeedMS(void) { return Encoder_BR._data.speed_m_s; }
static float Encoder_BR_GetDistance(void) {
  return Encoder_BR._data.total_distance;
}
static Encoder_Direction_e Encoder_BR_GetDirection(void) {
  return Encoder_BR._data.direction;
}
static void Encoder_BR_Update(void) { Encoder_Update_Data(&Encoder_BR); }
static void Encoder_BR_Reset(void) {
  Encoder_BR._data.count = 0;
  Encoder_BR._data.last_count = 0;
  Encoder_BR._data.delta_count = 0;
  Encoder_BR._data.speed_rpm = 0;
  Encoder_BR._data.speed_rps = 0;
  Encoder_BR._data.speed_rad_s = 0;
  Encoder_BR._data.speed_m_s = 0;
  Encoder_BR._data.total_count = 0;
  Encoder_BR._data.total_distance = 0;
  Encoder_BR._data.direction = ENCODER_DIR_STOPPED;
}

/**
 * @brief 后左编码器实例操作
 */
static void Encoder_BL_Init(void) {
  Encoder_GPIO_Init(ENCODER_BL);
  Encoder_BL._data.last_update_time = 0;
  Encoder_BL._initialized = 1;
}
static void Encoder_BL_DeInit(void) { Encoder_BL._initialized = 0; }
static int16_t Encoder_BL_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_BL_TIM);
}
static int32_t Encoder_BL_GetTotalCount(void) {
  return Encoder_BL._data.total_count;
}
static float Encoder_BL_GetSpeedRPM(void) { return Encoder_BL._data.speed_rpm; }
static float Encoder_BL_GetSpeedRPS(void) { return Encoder_BL._data.speed_rps; }
static float Encoder_BL_GetSpeedRadS(void) {
  return Encoder_BL._data.speed_rad_s;
}
static float Encoder_BL_GetSpeedMS(void) { return Encoder_BL._data.speed_m_s; }
static float Encoder_BL_GetDistance(void) {
  return Encoder_BL._data.total_distance;
}
static Encoder_Direction_e Encoder_BL_GetDirection(void) {
  return Encoder_BL._data.direction;
}
static void Encoder_BL_Update(void) { Encoder_Update_Data(&Encoder_BL); }
static void Encoder_BL_Reset(void) {
  Encoder_BL._data.count = 0;
  Encoder_BL._data.last_count = 0;
  Encoder_BL._data.delta_count = 0;
  Encoder_BL._data.speed_rpm = 0;
  Encoder_BL._data.speed_rps = 0;
  Encoder_BL._data.speed_rad_s = 0;
  Encoder_BL._data.speed_m_s = 0;
  Encoder_BL._data.total_count = 0;
  Encoder_BL._data.total_distance = 0;
  Encoder_BL._data.direction = ENCODER_DIR_STOPPED;
}

#else

/**
 * @brief 双驱模式：右编码器实例操作（使用前右硬件）
 */
static void Encoder_Right_Init(void) {
  Encoder_GPIO_Init(ENCODER_RIGHT);
  Encoder_Right._data.last_update_time = 0;
  Encoder_Right._initialized = 1;
}
static void Encoder_Right_DeInit(void) { Encoder_Right._initialized = 0; }
static int16_t Encoder_Right_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_FR_TIM);
}
static int32_t Encoder_Right_GetTotalCount(void) {
  return Encoder_Right._data.total_count;
}
static float Encoder_Right_GetSpeedRPM(void) {
  return Encoder_Right._data.speed_rpm;
}
static float Encoder_Right_GetSpeedRPS(void) {
  return Encoder_Right._data.speed_rps;
}
static float Encoder_Right_GetSpeedRadS(void) {
  return Encoder_Right._data.speed_rad_s;
}
static float Encoder_Right_GetSpeedMS(void) {
  return Encoder_Right._data.speed_m_s;
}
static float Encoder_Right_GetDistance(void) {
  return Encoder_Right._data.total_distance;
}
static Encoder_Direction_e Encoder_Right_GetDirection(void) {
  return Encoder_Right._data.direction;
}
static void Encoder_Right_Update(void) { Encoder_Update_Data(&Encoder_Right); }
static void Encoder_Right_Reset(void) {
  Encoder_Right._data.count = 0;
  Encoder_Right._data.last_count = 0;
  Encoder_Right._data.delta_count = 0;
  Encoder_Right._data.speed_rpm = 0;
  Encoder_Right._data.speed_rps = 0;
  Encoder_Right._data.speed_rad_s = 0;
  Encoder_Right._data.speed_m_s = 0;
  Encoder_Right._data.total_count = 0;
  Encoder_Right._data.total_distance = 0;
  Encoder_Right._data.direction = ENCODER_DIR_STOPPED;
}

/**
 * @brief 双驱模式：左编码器实例操作（使用前左硬件）
 */
static void Encoder_Left_Init(void) {
  Encoder_GPIO_Init(ENCODER_LEFT);
  Encoder_Left._data.last_update_time = 0;
  Encoder_Left._initialized = 1;
}
static void Encoder_Left_DeInit(void) { Encoder_Left._initialized = 0; }
static int16_t Encoder_Left_GetCount(void) {
  return (int16_t)TIM_GetCounter(ENCODER_FL_TIM);
}
static int32_t Encoder_Left_GetTotalCount(void) {
  return Encoder_Left._data.total_count;
}
static float Encoder_Left_GetSpeedRPM(void) {
  return Encoder_Left._data.speed_rpm;
}
static float Encoder_Left_GetSpeedRPS(void) {
  return Encoder_Left._data.speed_rps;
}
static float Encoder_Left_GetSpeedRadS(void) {
  return Encoder_Left._data.speed_rad_s;
}
static float Encoder_Left_GetSpeedMS(void) {
  return Encoder_Left._data.speed_m_s;
}
static float Encoder_Left_GetDistance(void) {
  return Encoder_Left._data.total_distance;
}
static Encoder_Direction_e Encoder_Left_GetDirection(void) {
  return Encoder_Left._data.direction;
}
static void Encoder_Left_Update(void) { Encoder_Update_Data(&Encoder_Left); }
static void Encoder_Left_Reset(void) {
  Encoder_Left._data.count = 0;
  Encoder_Left._data.last_count = 0;
  Encoder_Left._data.delta_count = 0;
  Encoder_Left._data.speed_rpm = 0;
  Encoder_Left._data.speed_rps = 0;
  Encoder_Left._data.speed_rad_s = 0;
  Encoder_Left._data.speed_m_s = 0;
  Encoder_Left._data.total_count = 0;
  Encoder_Left._data.total_distance = 0;
  Encoder_Left._data.direction = ENCODER_DIR_STOPPED;
}

#endif

/* ==================== 私有函数：硬件初始化 ==================== */

/**
 * @brief 初始化单个编码器的GPIO引脚
 * @param encoder 编码器ID
 */
static void Encoder_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：直接初始化所有4个编码器的GPIO
  // 前右编码器
  RCC_AHB1PeriphClockCmd(ENCODER_FR_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_FR_CH1_PIN | ENCODER_FR_CH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ENCODER_FR_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_FR_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_FR_CH1_PORT, GPIO_PinSource0, ENCODER_FR_AF);
  GPIO_PinAFConfig(ENCODER_FR_CH2_PORT, GPIO_PinSource1, ENCODER_FR_AF);

  // 前左编码器
  RCC_AHB1PeriphClockCmd(ENCODER_FL_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_FL_CH1_PIN | ENCODER_FL_CH2_PIN;
  GPIO_Init(ENCODER_FL_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_FL_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_FL_CH1_PORT, GPIO_PinSource6, ENCODER_FL_AF);
  GPIO_PinAFConfig(ENCODER_FL_CH2_PORT, GPIO_PinSource7, ENCODER_FL_AF);

  // 后右编码器
  RCC_AHB1PeriphClockCmd(ENCODER_BR_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_BR_CH1_PIN | ENCODER_BR_CH2_PIN;
  GPIO_Init(ENCODER_BR_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_BR_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_BR_CH1_PORT, GPIO_PinSource12, ENCODER_BR_AF);
  GPIO_PinAFConfig(ENCODER_BR_CH2_PORT, GPIO_PinSource13, ENCODER_BR_AF);

  // 后左编码器
  RCC_AHB1PeriphClockCmd(ENCODER_BL_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_BL_CH1_PIN | ENCODER_BL_CH2_PIN;
  GPIO_Init(ENCODER_BL_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_BL_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_BL_CH1_PORT, GPIO_PinSource0, ENCODER_BL_AF);
  GPIO_PinAFConfig(ENCODER_BL_CH2_PORT, GPIO_PinSource1, ENCODER_BL_AF);

#else
  // 双驱模式：只初始化前轮2个编码器的GPIO
  // 前右编码器
  RCC_AHB1PeriphClockCmd(ENCODER_FR_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_FR_CH1_PIN | ENCODER_FR_CH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ENCODER_FR_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_FR_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_FR_CH1_PORT, GPIO_PinSource0, ENCODER_FR_AF);
  GPIO_PinAFConfig(ENCODER_FR_CH2_PORT, GPIO_PinSource1, ENCODER_FR_AF);

  // 前左编码器
  RCC_AHB1PeriphClockCmd(ENCODER_FL_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_FL_CH1_PIN | ENCODER_FL_CH2_PIN;
  GPIO_Init(ENCODER_FL_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_FL_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_FL_CH1_PORT, GPIO_PinSource6, ENCODER_FL_AF);
  GPIO_PinAFConfig(ENCODER_FL_CH2_PORT, GPIO_PinSource7, ENCODER_FL_AF);

#endif

}

/**
 * @brief 初始化编码器定时器为正交解码模式
 * @param encoder 编码器ID
 */
static void Encoder_TIM_Init(Encoder_Id_e encoder) {
  TIM_TypeDef *tim = Encoder_GetTIM(encoder);
  uint32_t tim_clk;

  // 获取时钟
  switch (encoder) {
#ifdef QUAD_MOTOR_DRIVE
  case ENCODER_FR:
    tim_clk = ENCODER_FR_TIM_CLK;
    break;
  case ENCODER_FL:
    tim_clk = ENCODER_FL_TIM_CLK;
    break;

  case ENCODER_BR:
    tim_clk = ENCODER_BR_TIM_CLK;
    break;
  case ENCODER_BL:
    tim_clk = ENCODER_BL_TIM_CLK;
    break;
#else
  case ENCODER_RIGHT:
    tim_clk = ENCODER_FR_TIM_CLK;
    break;
  case ENCODER_LEFT:
    tim_clk = ENCODER_FL_TIM_CLK;
    break;
#endif
  default:
    return;
  }

  // 使能定时器时钟
  RCC_APB1PeriphClockCmd(tim_clk, ENABLE);

  // 定时器时基配置
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 65535; // 16位最大值
  TIM_TimeBaseStructure.TIM_Prescaler = 0;  // 不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

  // 配置为正交编码器模式 (TI1和TI2都计数，4倍频)
  TIM_EncoderInterfaceConfig(tim,
                             TIM_EncoderMode_TI12,   // TI1和TI2都计数
                             TIM_ICPolarity_Rising,  // TI1上升沿
                             TIM_ICPolarity_Rising); // TI2上升沿

  // 配置输入滤波器
  TIM_SetIC1Filter(tim, TIM_ICFilter_PSC_5);
  TIM_SetIC2Filter(tim, TIM_ICFilter_PSC_5);

  // 使能定时器
  TIM_Cmd(tim, ENABLE);
}

/**
 * @brief 更新编码器数据（测速计算）
 * @param inst 编码器实例指针
 */
static void Encoder_Update_Data(Encoder_Class_t *inst) {
  // 读取当前计数值
  int16_t current_count = (int16_t)TIM_GetCounter(inst->tim);

  // 计算增量（处理溢出）
  int16_t delta = current_count - inst->_data.last_count;

  // 处理溢出情况
  if (delta > 32767) {
    delta -= 65536; // 向后溢出
  } else if (delta < -32768) {
    delta += 65536; // 向前溢出
  }

  inst->_data.delta_count = delta;
  inst->_data.last_count = current_count;

  // 更新总计数值
  inst->_data.total_count += delta;

  // 判断方向
  if (delta > 0) {
    inst->_data.direction = ENCODER_DIR_FORWARD;
  } else if (delta < 0) {
    inst->_data.direction = ENCODER_DIR_BACKWARD;
  } else {
    inst->_data.direction = ENCODER_DIR_STOPPED;
  }

  // 计算转速 (RPM) = (delta / PPR) * (60000 / sample_period_ms)
  float revolutions = (float)delta / (float)ENCODER_PPR;
  inst->_data.speed_rps = revolutions * (1000.0f / ENCODER_SAMPLE_PERIOD_MS);
  inst->_data.speed_rpm = inst->_data.speed_rps * 60.0f;

  // 计算角速度 (rad/s) = speed_rps * 2 * PI
  inst->_data.speed_rad_s = inst->_data.speed_rps * 6.28318f;

  // 计算线速度 (m/s) = speed_rad_s * radius
  float radius = inst->_data.wheel_diameter / 2.0f;
  inst->_data.speed_m_s = (float)delta * PULSE_TO_RAD * radius;
}

/* ==================== 编码器实例初始化 ==================== */

/**
 * @brief 初始化所有编码器实例的函数指针
 */
void Encoder_Driver_Init(void) {
#ifdef QUAD_MOTOR_DRIVE
  // 前右编码器
  Encoder_FR.id = ENCODER_FR;
  Encoder_FR.tim = ENCODER_FR_TIM;
  Encoder_FR._data.wheel_diameter = 0.065f; // 默认轮径65mm
  Encoder_FR._data.gear_ratio = 30.0f;
  Encoder_FR._data.direction = ENCODER_DIR_STOPPED;
  Encoder_FR.Init = Encoder_FR_Init;
  Encoder_FR.DeInit = Encoder_FR_DeInit;
  Encoder_FR.Reset = Encoder_FR_Reset;
  Encoder_FR.GetCount = Encoder_FR_GetCount;
  Encoder_FR.GetTotalCount = Encoder_FR_GetTotalCount;
  Encoder_FR.GetSpeedRPM = Encoder_FR_GetSpeedRPM;
  Encoder_FR.GetSpeedRPS = Encoder_FR_GetSpeedRPS;
  Encoder_FR.GetSpeedRadS = Encoder_FR_GetSpeedRadS;
  Encoder_FR.GetSpeedMS = Encoder_FR_GetSpeedMS;
  Encoder_FR.GetDistance = Encoder_FR_GetDistance;
  Encoder_FR.GetDirection = Encoder_FR_GetDirection;
  Encoder_FR.Update = Encoder_FR_Update;

  // 前左编码器
  Encoder_FL.id = ENCODER_FL;
  Encoder_FL.tim = ENCODER_FL_TIM;
  Encoder_FL._data.wheel_diameter = 0.065f;
  Encoder_FL._data.gear_ratio = 30.0f;
  Encoder_FL._data.direction = ENCODER_DIR_STOPPED;
  Encoder_FL.Init = Encoder_FL_Init;
  Encoder_FL.DeInit = Encoder_FL_DeInit;
  Encoder_FL.Reset = Encoder_FL_Reset;
  Encoder_FL.GetCount = Encoder_FL_GetCount;
  Encoder_FL.GetTotalCount = Encoder_FL_GetTotalCount;
  Encoder_FL.GetSpeedRPM = Encoder_FL_GetSpeedRPM;
  Encoder_FL.GetSpeedRPS = Encoder_FL_GetSpeedRPS;
  Encoder_FL.GetSpeedRadS = Encoder_FL_GetSpeedRadS;
  Encoder_FL.GetSpeedMS = Encoder_FL_GetSpeedMS;
  Encoder_FL.GetDistance = Encoder_FL_GetDistance;
  Encoder_FL.GetDirection = Encoder_FL_GetDirection;
  Encoder_FL.Update = Encoder_FL_Update;

  // 后右编码器
  Encoder_BR.id = ENCODER_BR;
  Encoder_BR.tim = ENCODER_BR_TIM;
  Encoder_BR._data.wheel_diameter = 0.065f;
  Encoder_BR._data.gear_ratio = 30.0f;
  Encoder_BR._data.direction = ENCODER_DIR_STOPPED;
  Encoder_BR.Init = Encoder_BR_Init;
  Encoder_BR.DeInit = Encoder_BR_DeInit;
  Encoder_BR.Reset = Encoder_BR_Reset;
  Encoder_BR.GetCount = Encoder_BR_GetCount;
  Encoder_BR.GetTotalCount = Encoder_BR_GetTotalCount;
  Encoder_BR.GetSpeedRPM = Encoder_BR_GetSpeedRPM;
  Encoder_BR.GetSpeedRPS = Encoder_BR_GetSpeedRPS;
  Encoder_BR.GetSpeedRadS = Encoder_BR_GetSpeedRadS;
  Encoder_BR.GetSpeedMS = Encoder_BR_GetSpeedMS;
  Encoder_BR.GetDistance = Encoder_BR_GetDistance;
  Encoder_BR.GetDirection = Encoder_BR_GetDirection;
  Encoder_BR.Update = Encoder_BR_Update;

  // 后左编码器
  Encoder_BL.id = ENCODER_BL;
  Encoder_BL.tim = ENCODER_BL_TIM;
  Encoder_BL._data.wheel_diameter = 0.065f;
  Encoder_BL._data.gear_ratio = 30.0f;
  Encoder_BL._data.direction = ENCODER_DIR_STOPPED;
  Encoder_BL.Init = Encoder_BL_Init;
  Encoder_BL.DeInit = Encoder_BL_DeInit;
  Encoder_BL.Reset = Encoder_BL_Reset;
  Encoder_BL.GetCount = Encoder_BL_GetCount;
  Encoder_BL.GetTotalCount = Encoder_BL_GetTotalCount;
  Encoder_BL.GetSpeedRPM = Encoder_BL_GetSpeedRPM;
  Encoder_BL.GetSpeedRPS = Encoder_BL_GetSpeedRPS;
  Encoder_BL.GetSpeedRadS = Encoder_BL_GetSpeedRadS;
  Encoder_BL.GetSpeedMS = Encoder_BL_GetSpeedMS;
  Encoder_BL.GetDistance = Encoder_BL_GetDistance;
  Encoder_BL.GetDirection = Encoder_BL_GetDirection;
  Encoder_BL.Update = Encoder_BL_Update;

  Encoder_FL.Init();
  Encoder_FR.Init();
  Encoder_BL.Init();
  Encoder_BR.Init();

  Encoder_TIM_Init(ENCODER_FR);
  Encoder_TIM_Init(ENCODER_FL);
  Encoder_TIM_Init(ENCODER_BR);
  Encoder_TIM_Init(ENCODER_BL);

#else
  // 右编码器 (使用前右硬件)
  Encoder_Right.id = ENCODER_RIGHT;
  Encoder_Right.tim = ENCODER_FR_TIM;
  Encoder_Right._data.wheel_diameter = 0.065f;
  Encoder_Right._data.gear_ratio = 30.0f;
  Encoder_Right._data.direction = ENCODER_DIR_STOPPED;
  Encoder_Right.Init = Encoder_Right_Init;
  Encoder_Right.DeInit = Encoder_Right_DeInit;
  Encoder_Right.Reset = Encoder_Right_Reset;
  Encoder_Right.GetCount = Encoder_Right_GetCount;
  Encoder_Right.GetTotalCount = Encoder_Right_GetTotalCount;
  Encoder_Right.GetSpeedRPM = Encoder_Right_GetSpeedRPM;
  Encoder_Right.GetSpeedRPS = Encoder_Right_GetSpeedRPS;
  Encoder_Right.GetSpeedRadS = Encoder_Right_GetSpeedRadS;
  Encoder_Right.GetSpeedMS = Encoder_Right_GetSpeedMS;
  Encoder_Right.GetDistance = Encoder_Right_GetDistance;
  Encoder_Right.GetDirection = Encoder_Right_GetDirection;
  Encoder_Right.Update = Encoder_Right_Update;

  // 左编码器 (使用前左硬件)
  Encoder_Left.id = ENCODER_LEFT;
  Encoder_Left.tim = ENCODER_FL_TIM;
  Encoder_Left._data.wheel_diameter = 0.065f;
  Encoder_Left._data.gear_ratio = 30.0f;
  Encoder_Left._data.direction = ENCODER_DIR_STOPPED;
  Encoder_Left.Init = Encoder_Left_Init;
  Encoder_Left.DeInit = Encoder_Left_DeInit;
  Encoder_Left.Reset = Encoder_Left_Reset;
  Encoder_Left.GetCount = Encoder_Left_GetCount;
  Encoder_Left.GetTotalCount = Encoder_Left_GetTotalCount;
  Encoder_Left.GetSpeedRPM = Encoder_Left_GetSpeedRPM;
  Encoder_Left.GetSpeedRPS = Encoder_Left_GetSpeedRPS;
  Encoder_Left.GetSpeedRadS = Encoder_Left_GetSpeedRadS;
  Encoder_Left.GetSpeedMS = Encoder_Left_GetSpeedMS;
  Encoder_Left.GetDistance = Encoder_Left_GetDistance;
  Encoder_Left.GetDirection = Encoder_Left_GetDirection;
  Encoder_Left.Update = Encoder_Left_Update;
  Encoder_Right.Init();
  Encoder_Left.Init();
  Encoder_TIM_Init(ENCODER_LEFT);
  Encoder_TIM_Init(ENCODER_RIGHT);

#endif
}
