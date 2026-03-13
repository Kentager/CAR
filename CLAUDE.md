# 智能车项目核心指南 (CLAUDE.md)
## 1. 项目核心信息
### 1.1 硬件架构
- 主控芯片：STM32F407ZGT6 (ARM Cortex-M4, 168MHz, 1MB Flash, 192KB RAM)
- 电机驱动：tb6612 (扩展为两路tb6612，共四路H桥，支持四路直流电机驱动，**支持通过宏定义切换双驱/四驱架构**)
- 寻路模块：8路TCRT5000红外循迹传感器 (模拟/数字输出)
- 姿态/里程检测：
  - JY61P (三轴加速度+三轴陀螺仪)：采用**卡尔曼滤波**实现姿态解算（偏航角/俯仰角/横滚角）
  - 四路电机编码器 (每路电机独立编码器，测速/里程计算)
- 通信：USART (波特率115200，8N1，用于调试/数据输出)

### 1.2 核心功能目标
- 基础功能：通过8路TCRT传感器实现黑线/白线循迹
- 运动控制：
  - 四路编码器独立反馈转速，闭环控制保证四轮转速同步，提升直线行驶精度
  - JY61P卡尔曼滤波解算偏航角，实时修正行驶偏航，结合编码器做二次校准
  - 结合编码器+卡尔曼滤波后的姿态角实现精准转弯（差速转弯/原地转弯，四轮独立调速）
  - **双环PID控制架构**：
    - **速度环**：采用**增量式PID算法**，基于编码器反馈的线速度进行控制，具有抗积分饱和、误动作影响小的优点
    - **角度环**：采用**位置式PID算法**，基于JY61P卡尔曼滤波后的姿态角进行控制，提供高精度的角度定位

### 1.3 关键系统约束
- **电源与驱动**：
  - **独立供电**：两路TB6612需独立供电，避免单电源过载；每路电机需做过流保护。
  - **PWM频率**：严格锁定 **20kHz**，平衡静音效果（>20kHz人耳不可听）与开关损耗（<100kHz）。
- **时序与算法**：
  - **同步机制**：JY61P采样频率(100Hz)必须与卡尔曼滤波迭代频率严格匹配，防止积分漂移。
  - **中断抢占**：必须严格执行「编码器 > 姿态解算 > 循迹采样」的优先级顺序，防止高速行驶时丢步。
  - **调试限制**：串口日志输出频率限制在10Hz（100ms）以内，防止阻塞式printf影响控制回路实时性。
  - **编译限制**：不许编辑makefil文件
- **四驱/双驱切换**：
  - 配置文件：[MyHardWare/Inc/HardwareConfig.h](file:///home/sword/STM32/CAR/MyHardWare/Inc/HardwareConfig.h)
  - 切换方法：取消/注释 `#define QUAD_MOTOR_DRIVE` 宏
  - **注意**：切换后需要重新编译项目，无需修改 Makefile

## 2. 开发/调试核心规则
### 2.1 代码规范
- 开发环境：VSCode + ARM-GCC (arm-none-eabi-gcc) + STM32标准库 (STM32F4xx_StdPeriph_Driver)+裸机开发（不使用freerots等实时系统）
- 编译工具链：ARM-GCC 10.3或以上版本，链接脚本基于STM32F407ZGT6_FLASH.ld
- 面向对象代码风格（C语言实现）：
  - 「类」命名：前缀+功能，如 Motor_Class、JY61P_Class、KalmanFilter_Class
  - 「成员变量」：结构体封装，私有变量加下划线前缀（如 _speed），公有变量无前缀
  - 「成员方法」：函数指针封装，命名格式：类名_方法名（如 Motor_Class_SetSpeed）
  - 「构造/析构」：统一命名为 Class_Init() / Class_DeInit()，负责资源初始化/释放
- 代码风格：
  - 函数名使用下划线命名 (motor_ctrl_set_speed)，全局常量大写 (PWM_FREQ_20KHZ)
  - 标准库函数调用遵循官方规范 (如 TIM_SetCompare1()、USART_SendData())
  - 中断服务函数命名：USART1_IRQHandler、TIM2_IRQHandler（符合标准库中断向量表）
  - 四路电机命名：MOTOR_FR(前右)、MOTOR_FL(前左)、MOTOR_BR(后右)、MOTOR_BL(后左)
  - 卡尔曼滤波函数命名：kalman_filter_init()、kalman_filter_update()
- 中断优先级：
  - 分组：NVIC_PriorityGroup_2 (2位抢占优先级，2位响应优先级)
  - 优先级排序：编码器中断 (抢占1) > JY61P数据读取中断 (抢占2) > 卡尔曼滤波计算 (抢占2) > TCRT采样中断 (抢占3) > 串口中断 (抢占3)
- 定时器分配（四路电机专属）：
  - PWM输出（20kHz，1000级分辨率）：
    - TIM1 (高级定时器)：CH1=MOTOR_FR_PWM, CH2=MOTOR_FL_PWM
    - TIM8 (高级定时器)：CH1=MOTOR_BR_PWM, CH2=MOTOR_BL_PWM
    - 方向控制：GPIO口独立控制每路电机正反转（TIM1/8仅输出PWM，方向由GPIO控制）
  - 编码器计数（正交解码模式）：
    - TIM2：MOTOR_FR编码器
    - TIM3：MOTOR_FL编码器
    - TIM4：MOTOR_BR编码器
    - TIM5：MOTOR_BL编码器
  - 辅助定时器：
    - TIM6：TCRT传感器采样 (10ms周期，TIM_IT_Update中断)
    - TIM7：JY61P数据读取 (10ms周期，100Hz，TIM_IT_Update中断)，触发卡尔曼滤波计算

### 2.2 关键技术参数
- 串口配置：USART1 (PA9/PA10)，波特率115200，无奇偶校验，1停止位
  - 标准库配置：USART_InitStructure.USART_BaudRate = 115200;
  - 时钟源：APB2时钟 (168MHz)，USART1时钟分频系数1
- PWM参数：20kHz频率，1000级分辨率
  - 配置：ARR=8399，PSC=0 (TIM_PrescalerConfig)，计数模式向上计数
  - 标准库实现：TIM_OC1Init() 配置PWM模式1，TIM_Cmd() 使能定时器
- 编码器参数：每路电机编码器参数一致（示例：500线编码器，减速比1:30）
  - 标准库配置：TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising)
  - 计数范围：0-65535（16位定时器，溢出后清零重新计数）
- JY61P + 卡尔曼滤波参数：
  - JY61P：I2C1 (PB6/PB7)，采样率100Hz，低通滤波5Hz
    - 标准库I2C配置：I2C_StandardMode (100kHz)，I2C_AcknowledgeConfig(I2C1, ENABLE)
  - 卡尔曼滤波（一维，针对偏航角）：
    - 过程噪声协方差 Q = 0.001（陀螺仪噪声）
    - 测量噪声协方差 R = 0.01（加速度计+磁力计融合噪声）
    - 状态估计协方差 P 初始值 = 1.0
    - 迭代频率：100Hz（与JY61P采样频率一致）
    - 解算维度：优先解算偏航角（yaw），用于智能车方向控制；俯仰/横滚角可选解算

### 2.3 编译与调试规则
- 编译脚本：使用Makefile（核心编译参数如下）
  - 核心CFLAGS：-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -Wall
  - 链接参数：-T STM32F407ZGT6_FLASH.ld -Wl,-Map=output.map,--cref
- 烧录工具：openocd + cmsis-dap，VSCode配置launch.json调试
- 调试输出：串口打印使用自定义printf重定向（重映射fputc到USART_SendData）
  - 调试内容：每100ms输出一次卡尔曼滤波后的偏航角、四路电机转速
- 标准库宏定义：需在编译时添加 -DSTM32F407_ZX -DUSE_STDPERIPH_DRIVER -DKALMAN_FILTER_JY61P
  - **驱动模式**：通过修改 [MyHardWare/Inc/HardwareConfig.h](file:///home/sword/STM32/CAR/MyHardWare/Inc/HardwareConfig.h) 文件切换四驱/双驱模式：
    - **四驱模式**：取消注释 `#define QUAD_MOTOR_DRIVE` 宏
    - **双驱模式**：注释掉 `#define QUAD_MOTOR_DRIVE` 宏
    - 无需修改 Makefile，宏定义在头文件中配置即可自动生效

### 2.4 核心算法逻辑
- **双环PID控制架构**：
  - **速度环（增量式PID）**：
    - 控制目标：电机线速度（m/s）
    - 算法特点：输出控制量增量 Δu(k)，具有抗积分饱和特性，适合速度控制
    - 更新频率：10ms（100Hz）
    - 反馈源：四路编码器独立测速
    - 公式：Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k)·dt + Kd·[e(k)-2e(k-1)+e(k-2)]/dt
  - **角度环（位置式PID）**：
    - 控制目标：车体姿态角度（度）
    - 算法特点：直接输出绝对控制量 u(k)，精度高，适合角度/位置控制
    - 更新频率：5ms（200Hz）
    - 反馈源：JY61P卡尔曼滤波后的姿态角
    - 公式：u(k) = Kp·e(k) + Ki·∫e(t)dt + Kd·de(t)/dt
- 循迹算法：8路TCRT数据归一化后，采用加权平均法计算偏差值，PID调节四轮转速（外侧电机增速/内侧电机减速）
- 直线行驶：
  - 基础层：四路编码器反馈转速，计算每路与基准转速的差值，PID独立补偿
  - 补偿层：JY61P卡尔曼滤波解算偏航角，实时调整左右侧电机整体转速差，修正行驶方向（偏航角偏差>1°时启动补偿）
- 转弯控制：
  - 小角度转弯（<30°）：基于卡尔曼滤波后的偏航角闭环控制，差速调节左右侧电机转速（前/后电机转速同步）
  - 大角度/原地转弯（≥30°）：编码器计数控制每路电机转动距离，卡尔曼滤波后的偏航角校准最终角度，实现四轮独立调速的原地转向

## 3. 功能细分
以下文档包含细分功能细节，仅在处理对应任务时阅读：
- `hardware_config`：四驱/双驱硬件架构切换配置、宏定义使用方法
- `motor_control`：四路/双路电机PWM驱动、正反转、刹车逻辑及tb6612保护机制（标准库实现）
- `tcrt_sampling`：8路TCRT传感器校准、ADC采样（标准库）、数据滤波、偏差计算
- `encoder_JY61P`：四路/双路编码器计数（标准库正交解码）、JY61P I2C读写、卡尔曼滤波实现（标准库C代码）、轮转速同步算法
- `kalman_filter`：卡尔曼滤波核心公式、参数调优方法、偏航角解算实战（适配智能车场景）
- `debug_log`：四路/双路电机状态+卡尔曼滤波姿态角输出规范
- `pid_control`：**新增** - 速度环增量式PID与角度环位置式PID详细实现说明

## 4. 四驱/双驱切换详细说明
### 4.1 硬件配置差异
| 配置项 | 四驱模式 (QUAD_MOTOR_DRIVE) | 双驱模式 |
|--------|----------------------------|-----------|
| 电机数量 | 4路电机 | 2路电机 |
| 编码器数量 | 4路编码器 | 2路编码器 |
| 驱动芯片 | TB6612 x 2 | TB6612 x 1 |
| PWM定时器 | TIM1 + TIM8 | TIM1 |
| 编码器定时器 | TIM2 + TIM3 + TIM4 + TIM5 | TIM2 + TIM3 |

### 4.2 切换步骤
1. 打开配置文件：[MyHardWare/Inc/HardwareConfig.h](file:///home/sword/STM32/CAR/MyHardWare/Inc/HardwareConfig.h)
2. 找到 `QUAD_MOTOR_DRIVE` 宏定义
3. **切换到四驱模式**：取消注释
   ```c
   #define QUAD_MOTOR_DRIVE
4. **切换到双电机模式**：注释掉
   ```c
   #define QUAD_MOTOR_DRIVE
5. **保存并编译**

### 4.3 代码接口差异
**四驱模式：**
- 电机ID：MOTOR_FR(前右)、MOTOR_FL(前左)、MOTOR_BR(后右)、MOTOR_BL(后左)
- 编码器ID：ENCODER_FR、ENCODER_FL、ENCODER_BR、ENCODER_BL
- PID控制器：
  - 速度环：Speed_PID_FR、Speed_PID_FL、Speed_PID_BR、Speed_PID_BL（四个独立的增量式PID控制器）
  - 角度环：Angle_PID（单个位置式PID控制器，控制整体车体姿态）
- 快捷函数：Motor_FR_SetSpeed()、Motor_FL_SetSpeed()、Motor_BR_SetSpeed()、Motor_BL_SetSpeed()

**双驱模式：**
- 电机ID：MOTOR_RIGHT(右轮)、MOTOR_LEFT(左轮)
- 编码器ID：ENCODER_RIGHT、ENCODER_LEFT  
- PID控制器：
  - 速度环：Speed_PID_Right、Speed_PID_Left（两个独立的增量式PID控制器）
  - 角度环：Angle_PID（单个位置式PID控制器，控制整体车体姿态）
- 快捷函数：Motor_RIGHT_SetSpeed()、Motor_LEFT_SetSpeed()

### 4.4 注意事项
- **PID控制器适配**：切换模式后，PID控制器的数量会自动适配。四驱模式有4个速度环PID控制器，双驱模式有2个速度环PID控制器，角度环PID控制器始终为1个。
- **编译要求**：切换模式后必须重新编译整个项目，以确保所有条件编译宏正确生效。
- **硬件连接**：双驱模式下，硬件仅使用前轮电机和编码器（MOTOR_FR/MOTOR_FL 对应 MOTOR_RIGHT/MOTOR_LEFT，ENCODER_FR/ENCODER_FL 对应 ENCODER_RIGHT/ENCODER_LEFT）。
- **API兼容性**：所有电机和编码器的API接口保持一致，只是枚举值和控制器实例数量不同。上层应用代码无需修改，底层会根据宏定义自动适配。
- **性能考虑**：四驱模式下计算负载更高，需要确保主循环执行时间不超过控制周期要求（速度环10ms，角度环5ms）。
- **调试建议**：首次切换模式后，建议重新调试PID参数，因为机械特性和负载分布可能发生变化。