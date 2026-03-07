/**
 * @file motor.c
 * @brief 电机PWM驱动实现文件 - 智能车专用（面向对象风格）
 * @version 2.0
 * @date 2026-03-04
 */

#include "motor.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stddef.h"

/* ==================== 全局电机实例定义 ==================== */
// 四驱模式电机实例
#ifdef QUAD_MOTOR_DRIVE
Motor_Class_t Motor_FR = {0};
Motor_Class_t Motor_FL = {0};
Motor_Class_t Motor_BR = {0};
Motor_Class_t Motor_BL = {0};
#else
// 双驱模式电机实例
static Motor_Class_t Motor_Left = {0,MOTOR_DIR_STOP,MOTOR_LEFT,MOTOR_FL_PWM_TIM,MOTOR_FL_PWM_CHANNEL};
static Motor_Class_t Motor_Right = {0,MOTOR_DIR_STOP,MOTOR_RIGHT,MOTOR_FR_PWM_TIM,MOTOR_FR_PWM_CHANNEL};
#endif

/* ==================== 私有函数声明 ==================== */
static void Motor_GPIO_Init(void);
static void Motor_TIM_Init(TIM_TypeDef* tim, uint32_t tim_clk);
static void Motor_TIM_Channel_Init(TIM_TypeDef* tim, uint16_t channel);
static void Motor_Instance_SetSpeed(Motor_Class_t* inst, int16_t speed);



//* ==================== 通用函数：Motor操控 ==================== */
/**
 * @brief 通用电机速度设置函数
 * @param motor 电机 ID (MOTOR_FR/MOTOR_FL/MOTOR_BR/MOTOR_BL 或 MOTOR_RIGHT/MOTOR_LEFT)
 * @param speed 速度值 (-MOTOR_PWM_MAX_DUTY ~ +MOTOR_PWM_MAX_DUTY)
 *              正值：正转，负值：反转，0：停止
 * 
 * @note 此函数仅修改电机实例的 _speed 成员变量，不直接操作硬件,  speed最好不要传入负值，传入负值理论上就要去改motor的direction变量，暂且还没有写
 */
void Motor_SetSpeed(Motor_Id_e motor, int16_t speed)
{
    Motor_Class_t* inst = NULL;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR: inst = &Motor_FR; break;
        case MOTOR_FL: inst = &Motor_FL; break;
        case MOTOR_BR: inst = &Motor_BR; break;
        case MOTOR_BL: inst = &Motor_BL; break;
        default: return;
    }
#else
    switch (motor) {
        case MOTOR_RIGHT: inst = &Motor_Right; break;
        case MOTOR_LEFT:  inst = &Motor_Left; break;
        default: return;
    }
#endif
    
    // 限制速度范围
    if (speed > MOTOR_PWM_MAX_DUTY) {
        speed = MOTOR_PWM_MAX_DUTY;
    } else if (speed < -MOTOR_PWM_MAX_DUTY) {
        speed = -MOTOR_PWM_MAX_DUTY;
    }
    
    // 更新速度变量
    inst->_speed = speed;
}

/**
 * @brief 通用电机方向设置函数
 * @param motor 电机 ID
 * @param dir 方向 (MOTOR_DIR_FORWARD/MOTOR_DIR_BACKWARD/MOTOR_DIR_STOP/MOTOR_DIR_BRAKE)
 * 
 * @note 此函数仅修改电机实例的 _direction 成员变量，不直接操作硬件
 */
 void Motor_SetDirection(Motor_Id_e motor, Motor_Direction_e dir)
{
    Motor_Class_t* inst = NULL;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR: inst = &Motor_FR; break;
        case MOTOR_FL: inst = &Motor_FL; break;
        case MOTOR_BR: inst = &Motor_BR; break;
        case MOTOR_BL: inst = &Motor_BL; break;
        default: return;
    }
#else
    switch (motor) {
        case MOTOR_RIGHT: inst = &Motor_Right; break;
        case MOTOR_LEFT:  inst = &Motor_Left; break;
        default: return;
    }
#endif
    
    // 更新方向变量
    inst->_direction = dir;
}

/**
 * @brief 通用电机硬件更新函数
 * @param motor 电机 ID
 * 
 * @details 根据电机实例的 _speed 和 _direction 成员变量更新硬件状态：
 *          1. 根据 _direction 设置 GPIO 方向控制引脚
 *          2. 根据 |_speed| 设置 PWM 占空比
 * 
 * @note 调用此函数前应先调用 Motor_SetSpeed 和/或 Motor_SetDirection
 */
void Motor_Update(Motor_Id_e motor)
{
    Motor_Class_t* inst = NULL;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR: inst = &Motor_FR; break;
        case MOTOR_FL: inst = &Motor_FL; break;
        case MOTOR_BR: inst = &Motor_BR; break;
        case MOTOR_BL: inst = &Motor_BL; break;
        default: return;
    }
#else
    switch (motor) {
        case MOTOR_RIGHT: inst = &Motor_Right; break;
        case MOTOR_LEFT:  inst = &Motor_Left; break;
        default: return;
    }
#endif
    
    // 根据 direction 设置 GPIO
    GPIO_TypeDef* dir_port;
    uint16_t dir1_pin, dir2_pin;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR:
            dir_port = MOTOR_FR_DIR_PORT;
            dir1_pin = MOTOR_FR_DIR1_PIN;
            dir2_pin = MOTOR_FR_DIR2_PIN;
            break;
        case MOTOR_FL:
            dir_port = MOTOR_FL_DIR_PORT;
            dir1_pin = MOTOR_FL_DIR1_PIN;
            dir2_pin = MOTOR_FL_DIR2_PIN;
            break;
        case MOTOR_BR:
            dir_port = MOTOR_BR_DIR_PORT;
            dir1_pin = MOTOR_BR_DIR1_PIN;
            dir2_pin = MOTOR_BR_DIR2_PIN;
            break;
        case MOTOR_BL:
            dir_port = MOTOR_BL_DIR_PORT;
            dir1_pin = MOTOR_BL_DIR1_PIN;
            dir2_pin = MOTOR_BL_DIR2_PIN;
            break;
        default:
            return;
    }
#else
    if (motor == MOTOR_RIGHT) {
        dir_port = MOTOR_FR_DIR_PORT;
        dir1_pin = MOTOR_FR_DIR1_PIN;
        dir2_pin = MOTOR_FR_DIR2_PIN;
    } else if (motor == MOTOR_LEFT) {
        dir_port = MOTOR_FL_DIR_PORT;
        dir1_pin = MOTOR_FL_DIR1_PIN;
        dir2_pin = MOTOR_FL_DIR2_PIN;
    } else {
        return;
    }
#endif
    
    // 执行方向控制
    switch (inst->_direction) {
        case MOTOR_DIR_FORWARD:
            GPIO_SetBits(dir_port, dir1_pin);
            GPIO_ResetBits(dir_port, dir2_pin);
            break;
        case MOTOR_DIR_BACKWARD:
            GPIO_ResetBits(dir_port, dir1_pin);
            GPIO_SetBits(dir_port, dir2_pin);
            break;
        case MOTOR_DIR_STOP:
            GPIO_ResetBits(dir_port, dir1_pin | dir2_pin);
            break;
        case MOTOR_DIR_BRAKE:
            GPIO_SetBits(dir_port, dir1_pin | dir2_pin);
            break;
    }
    
    // 根据 speed 绝对值设置 PWM 占空比
    uint16_t duty = (inst->_speed >= 0) ? (uint16_t)inst->_speed : (uint16_t)(-inst->_speed);
    if (duty > MOTOR_PWM_MAX_DUTY) duty = MOTOR_PWM_MAX_DUTY;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR:
        case MOTOR_BR:
            TIM_SetCompare1(inst->tim, duty);
            break;
        case MOTOR_FL:
        case MOTOR_BL:
            TIM_SetCompare2(inst->tim, duty);
            break;
    }
#else
    if (motor == MOTOR_RIGHT) {
        TIM_SetCompare1(inst->tim, duty);
    } else if (motor == MOTOR_LEFT) {
        TIM_SetCompare2(inst->tim, duty);
    }
#endif
}


/**
 * @brief 通用单个电机停止函数
 * @param motor 电机 ID (MOTOR_FR/MOTOR_FL/MOTOR_BR/MOTOR_BL 或 MOTOR_RIGHT/MOTOR_LEFT)
 * 
 * @details 将电机的速度设置为 0，方向设置为 STOP
 *           直接操作硬件，确保最快响应
 */
void Motor_Stop(Motor_Id_e motor)
{
    Motor_Class_t* inst = NULL;
    GPIO_TypeDef* dir_port;
    uint16_t dir1_pin, dir2_pin;
    TIM_TypeDef* pwm_tim;
    uint16_t pwm_channel;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR:
            inst = &Motor_FR;
            dir_port = MOTOR_FR_DIR_PORT;
            dir1_pin = MOTOR_FR_DIR1_PIN;
            dir2_pin = MOTOR_FR_DIR2_PIN;
            pwm_tim = MOTOR_FR_PWM_TIM;
            pwm_channel = MOTOR_FR_PWM_CHANNEL;
            break;
        case MOTOR_FL:
            inst = &Motor_FL;
            dir_port = MOTOR_FL_DIR_PORT;
            dir1_pin = MOTOR_FL_DIR1_PIN;
            dir2_pin = MOTOR_FL_DIR2_PIN;
            pwm_tim = MOTOR_FL_PWM_TIM;
            pwm_channel = MOTOR_FL_PWM_CHANNEL;
            break;
        case MOTOR_BR:
            inst = &Motor_BR;
            dir_port = MOTOR_BR_DIR_PORT;
            dir1_pin = MOTOR_BR_DIR1_PIN;
            dir2_pin = MOTOR_BR_DIR2_PIN;
            pwm_tim = MOTOR_BR_PWM_TIM;
            pwm_channel = MOTOR_BR_PWM_CHANNEL;
            break;
        case MOTOR_BL:
            inst = &Motor_BL;
            dir_port = MOTOR_BL_DIR_PORT;
            dir1_pin = MOTOR_BL_DIR1_PIN;
            dir2_pin = MOTOR_BL_DIR2_PIN;
            pwm_tim = MOTOR_BL_PWM_TIM;
            pwm_channel = MOTOR_BL_PWM_CHANNEL;
            break;
        default: return;
    }
#else
    switch (motor) {
        case MOTOR_RIGHT:
            inst = &Motor_Right;
            dir_port = MOTOR_FR_DIR_PORT;
            dir1_pin = MOTOR_FR_DIR1_PIN;
            dir2_pin = MOTOR_FR_DIR2_PIN;
            pwm_tim = MOTOR_FR_PWM_TIM;
            pwm_channel = MOTOR_FR_PWM_CHANNEL;
            break;
        case MOTOR_LEFT:
            inst = &Motor_Left;
            dir_port = MOTOR_FL_DIR_PORT;
            dir1_pin = MOTOR_FL_DIR1_PIN;
            dir2_pin = MOTOR_FL_DIR2_PIN;
            pwm_tim = MOTOR_FL_PWM_TIM;
            pwm_channel = MOTOR_FL_PWM_CHANNEL;
            break;
        default: return;
    }
#endif
    
    // 更新状态变量
    inst->_speed = 0;
    inst->_direction = MOTOR_DIR_STOP;
    
    // 直接关闭 PWM 输出（占空比设为 0）
    if (pwm_channel == TIM_Channel_1) {
        TIM_SetCompare1(pwm_tim, 0);
    } else if (pwm_channel == TIM_Channel_2) {
        TIM_SetCompare2(pwm_tim, 0);
    }
    
    // 直接设置 GPIO 为停止状态（两个方向引脚都为低电平）
    GPIO_ResetBits(dir_port, dir1_pin | dir2_pin);
}

/**
 * @brief 通用单个电机刹车函数
 * @param motor 电机 ID (MOTOR_FR/MOTOR_FL/MOTOR_BR/MOTOR_BL 或 MOTOR_RIGHT/MOTOR_LEFT)
 * 
 * @details 将电机的速度设置为 0，方向设置为 BRAKE（能耗制动）
 *           直接操作硬件，确保最快响应
 */
void Motor_Brake(Motor_Id_e motor)
{
    Motor_Class_t* inst = NULL;
    GPIO_TypeDef* dir_port;
    uint16_t dir1_pin, dir2_pin;
    TIM_TypeDef* pwm_tim;
    uint16_t pwm_channel;
    
#ifdef QUAD_MOTOR_DRIVE
    switch (motor) {
        case MOTOR_FR:
            inst = &Motor_FR;
            dir_port = MOTOR_FR_DIR_PORT;
            dir1_pin = MOTOR_FR_DIR1_PIN;
            dir2_pin = MOTOR_FR_DIR2_PIN;
            pwm_tim = MOTOR_FR_PWM_TIM;
            pwm_channel = MOTOR_FR_PWM_CHANNEL;
            break;
        case MOTOR_FL:
            inst = &Motor_FL;
            dir_port = MOTOR_FL_DIR_PORT;
            dir1_pin = MOTOR_FL_DIR1_PIN;
            dir2_pin = MOTOR_FL_DIR2_PIN;
            pwm_tim = MOTOR_FL_PWM_TIM;
            pwm_channel = MOTOR_FL_PWM_CHANNEL;
            break;
        case MOTOR_BR:
            inst = &Motor_BR;
            dir_port = MOTOR_BR_DIR_PORT;
            dir1_pin = MOTOR_BR_DIR1_PIN;
            dir2_pin = MOTOR_BR_DIR2_PIN;
            pwm_tim = MOTOR_BR_PWM_TIM;
            pwm_channel = MOTOR_BR_PWM_CHANNEL;
            break;
        case MOTOR_BL:
            inst = &Motor_BL;
            dir_port = MOTOR_BL_DIR_PORT;
            dir1_pin = MOTOR_BL_DIR1_PIN;
            dir2_pin = MOTOR_BL_DIR2_PIN;
            pwm_tim = MOTOR_BL_PWM_TIM;
            pwm_channel = MOTOR_BL_PWM_CHANNEL;
            break;
        default: return;
    }
#else
    switch (motor) {
        case MOTOR_RIGHT:
            inst = &Motor_Right;
            dir_port = MOTOR_FR_DIR_PORT;
            dir1_pin = MOTOR_FR_DIR1_PIN;
            dir2_pin = MOTOR_FR_DIR2_PIN;
            pwm_tim = MOTOR_FR_PWM_TIM;
            pwm_channel = MOTOR_FR_PWM_CHANNEL;
            break;
        case MOTOR_LEFT:
            inst = &Motor_Left;
            dir_port = MOTOR_FL_DIR_PORT;
            dir1_pin = MOTOR_FL_DIR1_PIN;
            dir2_pin = MOTOR_FL_DIR2_PIN;
            pwm_tim = MOTOR_FL_PWM_TIM;
            pwm_channel = MOTOR_FL_PWM_CHANNEL;
            break;
        default: return;
    }
#endif
    
    // 更新状态变量
    inst->_speed = 0;
    inst->_direction = MOTOR_DIR_BRAKE;
    
    // 直接关闭 PWM 输出（占空比设为 0）
    if (pwm_channel == TIM_Channel_1) {
        TIM_SetCompare1(pwm_tim, 0);
    } else if (pwm_channel == TIM_Channel_2) {
        TIM_SetCompare2(pwm_tim, 0);
    }
    
    // 直接设置 GPIO 为刹车状态（两个方向引脚都为高电平）
    GPIO_SetBits(dir_port, dir1_pin | dir2_pin);
}




//* ==================== 私有函数：硬件初始化 ==================== */

/**
 * @brief 初始化所有电机的 GPIO 引脚
 * @note  根据编译配置 (QUAD_MOTOR_DRIVE) 一次性初始化所有需要的 GPIO
 */
static void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef QUAD_MOTOR_DRIVE
    // ==================== 四驱模式：初始化所有 4 个电机 GPIO ====================
    
    // --- 前右电机 (MOTOR_FR) - TIM1_CH1 ---
    RCC_AHB1PeriphClockCmd(MOTOR_FR_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_FR_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FR_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_FR_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_FR_PWM_PORT, GPIO_PinSource9, MOTOR_FR_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FR_DIR1_PIN | MOTOR_FR_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_FR_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_FR_DIR_PORT, MOTOR_FR_DIR1_PIN | MOTOR_FR_DIR2_PIN);

    // --- 前左电机 (MOTOR_FL) - TIM1_CH2 ---
    RCC_AHB1PeriphClockCmd(MOTOR_FL_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_FL_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FL_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_FL_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_FL_PWM_PORT, GPIO_PinSource11, MOTOR_FL_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FL_DIR1_PIN | MOTOR_FL_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_FL_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_FL_DIR_PORT, MOTOR_FL_DIR1_PIN | MOTOR_FL_DIR2_PIN);

    // --- 后右电机 (MOTOR_BR) - TIM8_CH1 ---
    RCC_AHB1PeriphClockCmd(MOTOR_BR_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_BR_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_BR_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_BR_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_BR_PWM_PORT, GPIO_PinSource6, MOTOR_BR_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_BR_DIR1_PIN | MOTOR_BR_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_BR_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_BR_DIR_PORT, MOTOR_BR_DIR1_PIN | MOTOR_BR_DIR2_PIN);

    // --- 后左电机 (MOTOR_BL) - TIM8_CH2 ---
    RCC_AHB1PeriphClockCmd(MOTOR_BL_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_BL_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_BL_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_BL_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_BL_PWM_PORT, GPIO_PinSource7, MOTOR_BL_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_BL_DIR1_PIN | MOTOR_BL_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_BL_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_BL_DIR_PORT, MOTOR_BL_DIR1_PIN | MOTOR_BL_DIR2_PIN);

#else
    // ==================== 双驱模式：仅初始化前轮 2 个电机 GPIO ====================
    
    // --- 右电机 (MOTOR_RIGHT) - 使用前右硬件 TIM1_CH1 ---
    RCC_AHB1PeriphClockCmd(MOTOR_FR_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_FR_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FR_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_FR_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_FR_PWM_PORT, GPIO_PinSource9, MOTOR_FR_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FR_DIR1_PIN | MOTOR_FR_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_FR_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_FR_DIR_PORT, MOTOR_FR_DIR1_PIN | MOTOR_FR_DIR2_PIN);

    // --- 左电机 (MOTOR_LEFT) - 使用前左硬件 TIM1_CH2 ---
    RCC_AHB1PeriphClockCmd(MOTOR_FL_PWM_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(MOTOR_FL_DIR_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FL_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MOTOR_FL_PWM_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_FL_PWM_PORT, GPIO_PinSource11, MOTOR_FL_PWM_AF);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_FL_DIR1_PIN | MOTOR_FL_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_FL_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_FL_DIR_PORT, MOTOR_FL_DIR1_PIN | MOTOR_FL_DIR2_PIN);
#endif
}

/**
 * @brief 初始化定时器时基单元
 * @param tim 定时器指针
 * @param tim_clk 定时器时钟RCC
 */
static void Motor_TIM_Init(TIM_TypeDef* tim, uint32_t tim_clk)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 使能定时器时钟
    RCC_APB2PeriphClockCmd(tim_clk, ENABLE);

    // 定时器时基配置 - 20kHz PWM
    // 168MHz / (0+1) / (8399+1) = 20kHz
    TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_PWM_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(tim, ENABLE);
}

/**
 * @brief 初始化定时器PWM通道
 * @param tim 定时器指针
 * @param channel PWM通道
 */
static void Motor_TIM_Channel_Init(TIM_TypeDef* tim, uint16_t channel)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

/* ==================== 公有函数实现 ==================== */

/**
 * @brief 电机驱动初始化
 */
void Motor_Driver_Init(void)
{
    // 初始化TIM1 (前右、前左电机 / 双驱模式下的右、左电机)
    Motor_TIM_Init(MOTOR_FR_PWM_TIM, MOTOR_FR_PWM_TIM_CLK);
    Motor_TIM_Channel_Init(MOTOR_FR_PWM_TIM, MOTOR_FR_PWM_CHANNEL);
    Motor_TIM_Channel_Init(MOTOR_FR_PWM_TIM, MOTOR_FL_PWM_CHANNEL);

#ifdef QUAD_MOTOR_DRIVE
    // 初始化TIM8 (后右、后左电机)
    Motor_TIM_Init(MOTOR_BR_PWM_TIM, MOTOR_BR_PWM_TIM_CLK);
    Motor_TIM_Channel_Init(MOTOR_BR_PWM_TIM, MOTOR_BR_PWM_CHANNEL);
    Motor_TIM_Channel_Init(MOTOR_BR_PWM_TIM, MOTOR_BL_PWM_CHANNEL);
#endif

    // 使能PWM输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
#ifdef QUAD_MOTOR_DRIVE
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
#endif
    Motor_GPIO_Init();
}


//似乎没必要，可以去掉
// /**
//  * @brief 电机驱动去初始化
//  */
// void Motor_Driver_DeInit(void)
// {
//     Motor_StopAll();
//     TIM_Cmd(TIM1, DISABLE);
//     TIM_CtrlPWMOutputs(TIM1, DISABLE);
// #ifdef QUAD_MOTOR_DRIVE
//     TIM_Cmd(TIM8, DISABLE);
//     TIM_CtrlPWMOutputs(TIM8, DISABLE);
// #endif
// }



/**
 * @brief 停止所有电机
 */
void Motor_StopAll(void)
{
#ifdef QUAD_MOTOR_DRIVE
    Motor_Stop(MOTOR_FR);
    Motor_Stop(MOTOR_FL);
    Motor_Stop(MOTOR_BR);
    Motor_Stop(MOTOR_BL);
#else
    Motor_Stop(MOTOR_RIGHT);
    Motor_Stop(MOTOR_LEFT);
#endif
}

/**
 * @brief 刹车所有电机
 */
void Motor_BrakeAll(void)
{
#ifdef QUAD_MOTOR_DRIVE
    Motor_Brake(MOTOR_FR);
    Motor_Brake(MOTOR_FL);
    Motor_Brake(MOTOR_BR);
    Motor_Brake(MOTOR_BL);
#else
    Motor_Brake(MOTOR_RIGHT);
    Motor_Brake(MOTOR_LEFT);
#endif
}

// ... existing code ...


/**
 * @brief 启动所有电机定时器
 */
void Motor_StartTimers(void)
{
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
#ifdef QUAD_MOTOR_DRIVE
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
#endif
}

/**
 * @brief 停止所有电机定时器
 */
void Motor_StopTimers(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_Cmd(TIM1, DISABLE);
#ifdef QUAD_MOTOR_DRIVE
    TIM_CtrlPWMOutputs(TIM8, DISABLE);
    TIM_Cmd(TIM8, DISABLE);
#endif
}
