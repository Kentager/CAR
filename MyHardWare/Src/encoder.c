
//=============================================================================
// NOTE:
// 注意，encoder会自动解算speed,需要在头文件中确定采样周期ENCODER_SAMPLE_PERIOD_MS和采样频率ENCODER_SAMPLE_FREQ_HZ
//=============================================================================

/**
 *
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
static Encoder_Class_t Encoder_Right = {
    ._data = {.last_count = 0,
              .delta_count = 0,
              .speed_rpm = 0.0f,
              .speed_rps = 0.0f,
              .speed_rad_s = 0.0f,
              .speed_m_s = 0.0f,
              .total_count = 0,
              .total_distance = 0.0f,
              .direction = ENCODER_DIR_STOPPED,
              .wheel_diameter = WHEEL_DIAMETER,
              .gear_ratio = GEAR_RATIO,
              .last_update_time = 0},
    ._initialized = 0,
    .id = ENCODER_RIGHT,
    .tim = TIM5};

static Encoder_Class_t Encoder_Left = {
    ._data = {.last_count = 0,
              .delta_count = 0,
              .speed_rpm = 0.0f,
              .speed_rps = 0.0f,
              .speed_rad_s = 0.0f,
              .speed_m_s = 0.0f,
              .total_count = 0,
              .total_distance = 0.0f,
              .direction = ENCODER_DIR_STOPPED,
              .wheel_diameter = WHEEL_DIAMETER,
              .gear_ratio = GEAR_RATIO,
              .last_update_time = 0},
    ._initialized = 0,
    .id = ENCODER_LEFT,
    .tim = TIM2}; // 修改为 TIM2
#endif

/* ==================== 私有函数：硬件初始化 ==================== */

static void Encoder_GPIO_Init(void);
static void Encoder_TIM_Init(void);
/**
 * @brief 初始化单个编码器的GPIO引脚
 * @param encoder 编码器ID
 */
static void Encoder_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：直接初始化所有 4 个编码器的 GPIO
  // 前右编码器 (TIM2)
  RCC_AHB1PeriphClockCmd(ENCODER_FR_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_FR_CH1_PIN | ENCODER_FR_CH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ENCODER_FR_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_FR_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_FR_CH1_PORT, GPIO_PinSource15, ENCODER_FR_AF);
  GPIO_PinAFConfig(ENCODER_FR_CH2_PORT, GPIO_PinSource3, ENCODER_FR_AF);

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

  // 后左编码器 (TIM5)
  RCC_AHB1PeriphClockCmd(ENCODER_BL_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_BL_CH1_PIN | ENCODER_BL_CH2_PIN;
  GPIO_Init(ENCODER_BL_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_BL_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_BL_CH1_PORT, GPIO_PinSource10, ENCODER_BL_AF);
  GPIO_PinAFConfig(ENCODER_BL_CH2_PORT, GPIO_PinSource11, ENCODER_BL_AF);

#else
  // 双驱模式：只初始化 2 个编码器的 GPIO
  // 右编码器 (TIM5)
  RCC_AHB1PeriphClockCmd(ENCODER_RIGHT_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_RIGHT_CH1_PIN | ENCODER_RIGHT_CH2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ENCODER_RIGHT_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_RIGHT_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_RIGHT_CH1_PORT, GPIO_PinSource0, ENCODER_RIGHT_AF);
  GPIO_PinAFConfig(ENCODER_RIGHT_CH2_PORT, GPIO_PinSource1, ENCODER_RIGHT_AF);

  // 左编码器 (TIM2)
  RCC_AHB1PeriphClockCmd(ENCODER_LEFT_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ENCODER_LEFT_CH1_PIN | ENCODER_LEFT_CH2_PIN;
  GPIO_Init(ENCODER_LEFT_CH1_PORT, &GPIO_InitStructure);
  GPIO_Init(ENCODER_LEFT_CH2_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENCODER_LEFT_CH1_PORT, GPIO_PinSource5, ENCODER_LEFT_AF);
  GPIO_PinAFConfig(ENCODER_LEFT_CH2_PORT, GPIO_PinSource3, ENCODER_LEFT_AF);

#endif
}

/**
 * @brief 编码器定时器初始化（根据硬件配置直接初始化）
 * @note  通过条件编译直接初始化对应的定时器，无需运行时判断
 */
static void Encoder_TIM_Init(void) {
#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：初始化 TIM2, TIM3, TIM4, TIM5

  // 使能定时器时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                             RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5,
                         ENABLE);

  // TIM2 配置 (前右编码器)
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // 输入捕获初始化结构体
  TIM_ICInitTypeDef TIM_IC_InitStructure;
  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_IC_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_IC_InitStructure.TIM_ICSelection = TIM_ICSC_DIRECT;
  TIM_IC_InitStructure.TIM_ICPrescaler = TIM_IC_PSC_DIV1;
  TIM_IC_InitStructure.TIM_ICFilter = 0x05; // 滤波器设置
  TIM_IC_Init(TIM2, &TIM_IC_InitStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_IC_Init(TIM2, &TIM_IC_InitStructure);

  // 配置编码器接口
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_Cmd(TIM2, ENABLE);

  // TIM3 配置 (前左编码器)
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_IC_Init(TIM3, &TIM_IC_InitStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_IC_Init(TIM3, &TIM_IC_InitStructure);

  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_Cmd(TIM3, ENABLE);

  // TIM4 配置 (后右编码器)
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_IC_Init(TIM4, &TIM_IC_InitStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_IC_Init(TIM4, &TIM_IC_InitStructure);

  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_Cmd(TIM4, ENABLE);

  // TIM5 配置 (后左编码器)
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_IC_Init(TIM5, &TIM_IC_InitStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_IC_Init(TIM5, &TIM_IC_InitStructure);

  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_Cmd(TIM5, ENABLE);

#else
  // 双驱模式：只初始化 TIM2, TIM5

  // 使能定时器时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);

  // TIM2 配置 (左编码器)
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 200000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // 输入捕获初始化结构体
  TIM_ICInitTypeDef TIM_IC_InitStructure;
  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_IC_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_IC_InitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_IC_InitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_IC_InitStructure.TIM_ICFilter = 0x05; // 滤波器设置
  TIM_ICInit(TIM2, &TIM_IC_InitStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM2, &TIM_IC_InitStructure);

  // 配置编码器接口
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);
  TIM_Cmd(TIM2, ENABLE);

  // TIM5 配置 (右编码器)
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM5, &TIM_IC_InitStructure);
  TIM_IC_InitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM5, &TIM_IC_InitStructure);

  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Falling);
  TIM_Cmd(TIM5, ENABLE);
#endif
  Encoder_Update();
}

void Encoder_Driver_Init(void) {
  Encoder_GPIO_Init();
  Encoder_TIM_Init();
}

/**
 * @brief 更新所有编码器的数据
 * @note  根据QUAD_MOTOR_DRIVE宏定义自动适配四驱/双驱模式
 *        在10ms控制循环中调用，实现所有编码器数据同步更新
 */
void Encoder_Update(void) {
  // 获取当前时间戳（用于计算时间间隔）
  float dt_seconds = ENCODER_SAMPLE_PERIOD_MS / 1000.0f; // 转换为秒

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：处理 4 个编码器
  Encoder_Class_t *encoders[] = {&Encoder_FR, &Encoder_FL, &Encoder_BR,
                                 &Encoder_BL};
  for (int i = 0; i < 4; i++) {
    Encoder_Class_t *instance = encoders[i];

    // 读取当前计数值
    int32_t current_count = (int32_t)TIM_GetCounter(instance->tim);

    // 计算增量并处理计数器溢出/下溢
    int32_t delta = current_count - instance->_data.last_count;

    // 检测是否发生溢出或下溢（ARR=65535）
    if (delta > 32767) {
      // 计数器从 65535 回绕到 0，需要减去 65536
      delta -= 65536;
    } else if (delta < -32767) {
      // 计数器从 0 回绕到 65535，需要加上 65536
      delta += 65536;
    }

    // 更新数据结构
    instance->_data.count = current_count;
    instance->_data.delta_count = delta;
    instance->_data.last_count = current_count;
    instance->_data.total_count += delta;

    // 计算速度
    instance->_data.speed_rps = (float)delta / (float)ENCODER_PPR / dt_seconds;
    instance->_data.speed_rpm = instance->_data.speed_rps / RPM_TO_RPS;
    instance->_data.speed_rad_s = instance->_data.speed_rps * 6.28318f;
    instance->_data.speed_m_s =
        instance->_data.speed_rad_s * (instance->_data.wheel_diameter / 2.0f);

    // 更新总里程
    instance->_data.total_distance += instance->_data.speed_m_s * dt_seconds;

    // 判断方向
    if (delta > 0) {
      instance->_data.direction = ENCODER_DIR_FORWARD;
    } else if (delta < 0) {
      instance->_data.direction = ENCODER_DIR_BACKWARD;
    } else {
      instance->_data.direction = ENCODER_DIR_STOPPED;
    }

    // 更新时间戳
    instance->_data.last_update_time = current_time;
  }
#else
  // 双驱模式：处理 2 个编码器
  Encoder_Class_t *encoders[] = {&Encoder_Right, &Encoder_Left};
  uint32_t current_count_array[2] = {
      (uint32_t)TIM_GetCounter(Encoder_Right.tim),
      (uint32_t)TIM_GetCounter(Encoder_Left.tim)};
  for (int i = 0; i < 2; i++) {
    uint32_t current_time = GetSysTick();
    Encoder_Class_t *instance = encoders[i];

    // 读取当前计数值 (支持 32 位定时器)
    int32_t current_count = current_count_array[i];

    // 计算增量并处理计数器溢出/下溢
    int32_t delta = current_count - instance->_data.last_count;

    // 检测是否发生溢出或下溢（ARR=200000）
    if (delta > 100000) {
      // 计数器从 200000 回绕到 0，需要减去 200001
      delta -= 200001;
    } else if (delta < -100000) {
      // 计数器从 0 回绕到 200000，需要加上 200001
      delta += 200001;
    }

    // 更新数据结构
    instance->_data.last_count = current_count;
    instance->_data.delta_count = delta;
    instance->_data.total_count += delta;

    // // 计算速度
    // instance->_data.speed_rps = (float)delta / (float)ENCODER_PPR /
    // dt_seconds; instance->_data.speed_rpm = instance->_data.speed_rps /
    // RPM_TO_RPS; instance->_data.speed_rad_s = instance->_data.speed_rps
    // * 6.28318f; instance->_data.speed_m_s =
    //     instance->_data.speed_rad_s * (instance->_data.wheel_diameter
    //     / 2.0f);

    // // 更新总里程
    // instance->_data.total_distance += instance->_data.speed_m_s * dt_seconds;

    instance->_data.speed_rps = (float)delta / 28000.0f / dt_seconds;

    instance->_data.speed_rpm = instance->_data.speed_rps / RPM_TO_RPS;

    instance->_data.speed_rad_s = instance->_data.speed_rps * 6.28318f;

    instance->_data.speed_m_s = instance->_data.speed_rad_s * (0.065f / 2.0f);

    // 更新总里程
    // instance->_data.total_distance += instance->_data.speed_m_s * dt_seconds;
    instance->_data.total_distance +=
        (float)delta / 28000.0f * 3.14159f * 0.065f;

    // 判断方向
    if (delta > 0) {
      instance->_data.direction = ENCODER_DIR_FORWARD;
    } else if (delta < 0) {
      instance->_data.direction = ENCODER_DIR_BACKWARD;
    } else {
      instance->_data.direction = ENCODER_DIR_STOPPED;
    }

    // 更新时间戳
    instance->_data.last_update_time = current_time;
  }
#endif
  // printf("encoder task tim:%d\r\n", GetSysTick());
}
Encoder_Data_t Encoder_GetData(Encoder_Id_e encoder) {
  // Encoder_Update(); // 确保数据是最新的
  Encoder_Data_t empty_data = {0};

  // 参数检查
  if (encoder >= ENCODER_MAX) {
    return empty_data;
  }

#ifdef QUAD_MOTOR_DRIVE
  // 四驱模式：返回 4 个编码器的数据
  Encoder_Class_t *encoders[] = {&Encoder_FR, &Encoder_FL, &Encoder_BR,
                                 &Encoder_BL};
#else
  // 双驱模式：返回 2 个编码器的数据
  Encoder_Class_t *encoders[] = {&Encoder_Right, &Encoder_Left};
#endif

  // 返回对应编码器的数据副本
  return encoders[encoder]->_data;
}
