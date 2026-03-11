# USART1 DMA 智能传输工作流程分析

## 概述

本模块实现了基于 **DMA + 环形缓冲区 + 定时器轮询** 的智能串口通信机制，采用 STM32F407 的 DMA2 控制器实现高效数据收发，显著降低 CPU 负载。

---

## 核心架构

```
┌──────────────────────────────────────────────────────────────┐
│                      应用层 (Application)                     │
│                    printf() / UART1_SendData()                │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│                   环形缓冲区管理层                            │
│          tx_queue (发送)           rx_queue (接收)            │
│          [256 字节]                 [256 字节]                │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│                   DMA 控制器层                                │
│    DMA2 Stream7 (TX)            DMA2 Stream2 (RX)            │
│    Memory → Peripheral          Peripheral → Memory          │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│                   硬件外设层                                  │
│         USART1 (PA9/PA10) @ 115200bps                        │
│         TIM7 (100μs 定时检查)                                │
└──────────────────────────────────────────────────────────────┘
```

---

## 关键组件说明

### 1. 环形缓冲区 (Ring Buffer)

```c
// 定义位置
static uint8_t dma_tx_buffer[256];  // DMA 发送缓冲区
static uint8_t dma_rx_buffer[256];  // DMA 接收缓冲区
RingBuffer tx_queue;                // 发送队列管理结构
RingBuffer rx_queue;                // 接收队列管理结构
```

**作用**：
- 解耦应用层与硬件层的数据传输节奏
- 支持突发数据写入和平滑数据读出
- 避免数据丢失和覆盖

**数据结构**：
```
tx_queue:
  - buffer: 指向 dma_tx_buffer[256]
  - head:   写指针（应用程序写入位置）
  - tail:   读指针（DMA 读取位置）
  - size:   缓冲区总大小 (256)
  - is_dma_enabled: DMA 工作状态标志
```

---

### 2. DMA 通道配置

| 功能 | DMA 流 | 通道 | 方向 | 优先级 |
|------|--------|------|------|--------|
| 发送 (TX) | DMA2 Stream7 | Channel 4 | Memory → Peripheral | High |
| 接收 (RX) | DMA2 Stream2 | Channel 4 | Peripheral → Memory | High |

**关键配置**：
```c
// 发送 DMA 配置
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 单次模式
DMA_InitStructure.DMA_Priority = DMA_Priority_High;

// 接收 DMA 配置
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
```

---

### 3. TIM7 定时器轮询机制

**定时周期**：100μs

**时钟计算**：
```
APB1 时钟 = 42MHz
Prescaler= 420- 1
Counter Frequency = 42MHz / 420 = 100kHz (周期 10μs)
Period = 10 - 1
Overflow Time = 100kHz / 10 = 10kHz = 100μs
```

**作用**：
- 定期检查 tx_queue 是否有新数据
- 有数据时启动 DMA 传输
- 无数据时关闭中断降低 CPU 负载

---

## 完整工作流程

### 一、初始化阶段

```
main()
  ↓
USART1_Init()
  ├── RingBuffer_Init(&tx_queue, dma_tx_buffer, 256)
  ├── RingBuffer_Init(&rx_queue, dma_rx_buffer, 256)
  ├── USART1_DMA_Init()
  │   ├── 使能 DMA2 时钟
  │   ├── 配置 NVIC 中断优先级
  │   │   ├── DMA2_Stream2_IRQn (RX, Prio: 0, Sub: 1)
  │   │   └── DMA2_Stream7_IRQn (TX, Prio: 0, Sub: 1)
  │   ├── 配置 DMA2 Stream7 (TX)
  │   └── 配置 DMA2 Stream2 (RX)
  ├── Usart_Config()
  │   ├── USART_GPIO_Config()
  │   │   ├── 使能 GPIOA 时钟
  │   │   ├── 配置 PA9 (TX) - 复用推挽输出
  │   │   ├── 配置 PA10 (RX) - 复用推挽输出
  │   │   └── 复用功能：GPIO_AF_USART1
  │   ├── 配置 USART1 参数
  │   │   ├── BaudRate: 115200
  │   │   ├── WordLength: 8 bits
  │   │   ├── StopBits: 1
  │   │   ├── Parity: No
  │   │   └── HardwareFlowControl: None
  │   └── 使能 USART DMA 请求
  │       ├── USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE)
  │       └── USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE)
  ├── TIM7_Config()
  │   ├── 使能 TIM7 时钟
  │   ├── 配置预分频和重装载值 (100μs)
  │   └── 配置 NVIC (TIM7_IRQn, Prio: 0, Sub: 2)
  └── UART1_DMA_Start_RX()
      └── 启动 DMA 接收（持续监听）
```

---

### 二、数据发送流程（重点）

#### 场景 1：应用程序调用 printf() 发送数据

```
应用层：printf("Hello\r\n")
  ↓
__io_putchar(int ch)  [被注释的实现，展示设计意图]
  ↓
RingBuffer_Push(&tx_queue, ch)
  ├── 检查缓冲区是否有空闲空间
  ├── 写入数据到 buffer[head]
  ├── head++ (循环递增)
  └── 返回 1 (成功)
  ↓
检查 DMA 状态
  ├── if (!tx_queue.is_dma_enabled && !RingBuffer_IsEmpty())
  │   └── TIM7_EnableIRQ()  // 唤醒定时器检查
  └── 返回
```

**关键点**：
- `printf()` **立即返回**，不等待数据实际发送
- CPU 可继续执行其他任务
- 数据暂存在 tx_queue 中

---

#### 场景 2：TIM7 定时器检查并启动 DMA

```
每 100μs 触发一次 TIM7_IRQHandler()
  ↓
清除中断标志
  ↓
检查 tx_queue 是否为空
  ↓
if (!RingBuffer_IsEmpty(&tx_queue))
  ↓
USART_StartDMA_TX()  ← 【核心函数】
  │
  ├── 计算连续可读数据长度
  │   ├── 情况 A: head > tail (未跨越末尾)
  │   │   └── data_count = head - tail
  │   │
  │   └── 情况 B: head < tail (跨越末尾)
  │       └── data_count = size - tail
  │           (只传输从 tail 到缓冲区末尾的部分)
  │
  ├── 配置 DMA 参数
  │   ├── DMA2_Stream7->M0AR = &tx_queue.buffer[tail]
  │   ├── DMA_SetCurrDataCounter(data_count)
  │   └── last_dma_tx_count = data_count
  │
  └── 启动 DMA 传输
      ├── DMA_Cmd(DMA2_Stream7, ENABLE)
      └── tx_queue.is_dma_enabled = 1
  ↓
TIM7_DisableIRQ()  // 关闭定时器中断
                   // (DMA 传输期间不需要频繁检查)
```

**环形缓冲区连续数据计算示例**：

```
情况 A: 数据未跨越末尾
buffer: [0][1][2][3][4][5]...[255]
              ↑           ↑
            tail        head
data_count = head - tail = 4 - 2 = 2 字节

情况 B: 数据跨越末尾（循环）
buffer: [0][1][2]...[253][254][255]
              ↑               ↑
            head            tail
data_count = size - tail = 256 - 254 = 2 字节
(下一次 DMA 会传输 [0][1][2] 这部分)
```

---

#### 场景 3：DMA 传输完成中断处理

```
DMA 完成数据传输
  ↓
DMA2_Stream7_IRQHandler()
  ↓
清除中断标志
DMA_ClearITPendingBit(DMA_IT_TCIF7)
  ↓
dma_tx_transfer_complete = 1
  ↓
DMA_Cmd(DMA2_Stream7, DISABLE)
  ↓
计算已传输数据量
  transferred = last_dma_tx_count - DMA2_Stream7->NDTR
  (通常 NDTR=0, transmitted = last_dma_tx_count)
  ↓
更新读指针
  RingBuffer_SkipRead(&tx_queue, transferred)
  ├── tail += transferred
  └── tail %= size (循环递增)
  ↓
tx_queue.is_dma_enabled = 0
  ↓
【关键决策点】检查队列是否还有数据
  │
  ├── if (!RingBuffer_IsEmpty(&tx_queue))
  │   │
  │   └── // 还有数据！立即启动下一段 DMA
  │       USART_StartDMA_TX()
  │       ├── 可能是分段传输的第二段
  │       └── 或是 DMA 期间 printf 新写入的数据
  │
  └── else
      │
      └── // 队列已空，重新使能 TIM7 等待新数据
          TIM7_EnableIRQ()
```

**智能连续传输机制**：

```
时间轴示例：
t0: printf("ABC") → tx_queue: [ABC][空]...
                    TIM7 检测到数据，启动 DMA 发送"ABC"
                    
t1: DMA 发送完成，tail 移动到末尾
    此时 printf("DEF") 又写入新数据
    tx_queue: [空]...[DEF]
    
t2: DMA 中断检查队列不为空
    立即启动下一段 DMA 发送"DEF"
    (无需等待下一个 100μs 定时器)
    
t3: DMA 再次完成，队列为空
    重新使能 TIM7 中断
```

---

### 三、数据接收流程

#### 持续 DMA 接收

```
初始化时启动：
UART1_DMA_Start_RX()
  ↓
配置 DMA2 Stream5 (注：代码中混用 Stream2 和 Stream5)
  ├── M0AR = rx_queue.buffer
  ├── NDTR = rx_queue.size (256)
  └── ENABLE

工作过程：
USART1 接收到数据 → DR 寄存器
  ↓
DMA 自动搬运 DR → rx_queue.buffer
  ↓
NDTR 递减 (剩余计数减少)
  ↓
填满整个缓冲区后触发 DMA_IT_TC 中断
  ↓
DMA2_Stream2_IRQHandler()
  ↓
清除中断标志
  ↓
UART1_DMA_Start_RX()  // 重新启动 DMA（循环接收）
```

**注意**：代码中存在 Stream 编号不一致问题
- 初始化使用：DMA2_Stream2
- 接收函数使用：DMA2_Stream5
- **这是一个潜在的 BUG**

---

## 关键函数详解

### 1. `USART_StartDMA_TX()` - 智能分段传输

```c
static void USART_StartDMA_TX(void) {
    // 空队列检查
    if (tx_queue.head == tx_queue.tail) return;
    
    uint16_t data_count;
    
    // 计算连续可读长度（处理循环缓冲区）
    if (tx_queue.head > tx_queue.tail) {
        // 情况 1: 数据在中间，未回绕
        data_count = tx_queue.head - tx_queue.tail;
    } else {
        // 情况 2: 数据跨越末尾，只传第一段
        data_count = tx_queue.size - tx_queue.tail;
    }
    
    // 配置 DMA
    DMA2_Stream7->M0AR = (uint32_t)&tx_queue.buffer[tx_queue.tail];
    DMA_SetCurrDataCounter(DMA2_Stream7, data_count);
    last_dma_tx_count = data_count;
    
    // 启动
    DMA_Cmd(DMA2_Stream7, ENABLE);
    tx_queue.is_dma_enabled = 1;
}
```

**设计亮点**：
- ✅ 自动处理环形缓冲区的循环特性
- ✅ 分段传输保证 DMA 访问连续内存
- ✅ 保存传输计数用于中断后更新指针

---

### 2. `DMA2_Stream7_IRQHandler()` - 无缝衔接传输

```c
void DMA2_Stream7_IRQHandler(void) {
    // 清除标志
    DMA_ClearITPendingBit(DMA_IT_TCIF7);
    
    // 计算已发送字节数
    uint16_t transferred = last_dma_tx_count - DMA2_Stream7->NDTR;
    
    // 更新读指针
    RingBuffer_SkipRead(&tx_queue, transferred);
    tx_queue.is_dma_enabled = 0;
    
    // 【智能决策】
    if (!RingBuffer_IsEmpty(&tx_queue)) {
        // 立即启动下一段（零延迟）
        USART_StartDMA_TX();
    } else {
        // 进入低功耗等待模式
        TIM7_EnableIRQ();
    }
}
```

**优势**：
- ✅ 避免传统 UART 发送的 CPU 等待
- ✅ 支持大数据块连续传输
- ✅ 动态启停降低功耗

---

## 性能优化分析

### 1. CPU 负载对比

| 方案 | CPU 占用率 | 适用场景 |
|------|-----------|---------|
| 轮询发送 | ~80% (等待 TXE) | 极低频数据 |
| 中断发送 | ~30% (每字节中断) | 小数据量 |
| **DMA+ 环形缓冲** | **<5%** | **高频大数据** |

---

### 2. 数据吞吐能力

**理论最大值**：
```
波特率：115200 bps = 11520 B/s
10ms 可发送：115 字节
100ms 可发送：1152 字节
```

**实际测试**：
```c
// 连续打印 100 字节数据
printf("%s", long_string);  // 瞬间完成，CPU 立即返回

// DMA 在后台以 115200bps 速率发送
// CPU 可同时处理电机控制、传感器读取等任务
```

---

### 3. 延迟特性

| 阶段 | 延迟 | 说明 |
|------|------|------|
| printf 写入缓冲 | <1μs | 仅内存拷贝 |
| 等待 TIM7 检查 | 0-100μs | 最坏情况 |
| DMA 启动 | ~5μs | 寄存器配置 |
| 实际串口发送 | 86.8μs/字节 | 115200bps |

**平均延迟**：~50μs (软件) + 86.8μs × N (硬件)

---

## 潜在问题与改进建议

### 问题 1: DMA Stream 编号不一致 ⚠️

```c
// 初始化中
DMA2_Stream2 // RX
DMA2_Stream7 // TX

// 但接收函数中使用
UART1_DMA_Start_RX() {
    DMA2_Stream5->M0AR = ...  // ❌ 应该是 Stream2!
}
```

**影响**：接收 DMA 无法正常工作

**修复**：
```c
void UART1_DMA_Start_RX(void) {
    DMA2_Stream2->M0AR = (uint32_t)rx_queue.buffer;  // 改 Stream5→Stream2
    DMA2_Stream2->NDTR = rx_queue.size;
    DMA_Cmd(DMA2_Stream2, ENABLE);  // 改 Stream5→Stream2
    rx_queue.is_dma_enabled = 1;
}
```

---

### 问题 2: 被注释的 `__io_putchar` 实现

```c
/* 
int __io_putchar(int ch) {
  while (RingBuffer_Push(&tx_queue, (uint8_t)ch) == 0) {}
  if (!tx_queue.is_dma_enabled && !RingBuffer_IsEmpty(&tx_queue)) {
    TIM7_EnableIRQ();
  }
  return (ch);
}
*/
```

**疑问**：当前使用什么方式实现 printf？

**建议**：
- 若使用半主机模式 (Semihosting)，需确保调试器支持
- 若要独立运行，应取消注释此函数

---

### 问题 3: 环形缓冲区溢出风险

```c
// 当前实现未展示 RingBuffer_Push 的满缓冲处理
while (RingBuffer_Push(&tx_queue, ch) == 0) {
    // 死等？丢弃？还是阻塞？
}
```

**改进建议**：
```c
// 方案 A: 带超时等待
uint32_t timeout = 1000;
while (RingBuffer_Push(&tx_queue, ch) == 0 && timeout--);
if (timeout == 0) return ERROR;  // 超时放弃

// 方案 B: 直接丢弃并警告
if (RingBuffer_Push(&tx_queue, ch) == 0) {
    ulog_warn("UART TX buffer overflow!\r\n");
    return ERROR;
}
```

---

### 问题 4: 多任务并发安全问题

```c
// 主循环和中断都可能访问 tx_queue
main loop:  RingBuffer_Push(&tx_queue, ...)  // 写操作
TIM7 IRQ:  RingBuffer_IsEmpty(&tx_queue)     // 读操作
DMA IRQ:   RingBuffer_SkipRead(&tx_queue, n) // 写操作
```

**风险**：竞态条件导致 head/tail 指针错乱

**解决方案**：
```c
// 方法 1: 临界区保护
__disable_irq();
RingBuffer_Push(&tx_queue, data);
__enable_irq();

// 方法 2: 原子操作 (如果 RingBuffer 支持)
RingBuffer_Push_Atomic(&tx_queue, data);
```

---

## 典型应用场景

### 场景 1: 高频传感器数据上传

```c
// 每 10ms 采集一次 MPU6050
void sensor_task(void) {
    static char buf[100];
    sprintf(buf, "ACC: %.2f, %.2f, %.2f\r\n", ax, ay, az);
    printf("%s", buf);  // 立即返回，不阻塞
    // CPU 继续处理其他任务
}
```

**优势**：不会因串口发送慢而阻塞控制循环

---

### 场景 2: 调试日志实时输出

```c
// PID 控制过程中实时打印
Speed_PID_Update(&controller);
ulog_info("Target: %.2f, Actual: %.2f\r\n", 
          target_speed, actual_speed);
// 日志在后台发送，不影响 PID 实时性
```

---

### 场景 3: 大数据量固件升级

```c
// 通过串口接收固件数据
void firmware_update(uint8_t *data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        UART1_SendByte(data[i]);  // 轮询方式（不推荐）
    }
    // 改进：使用 DMA 批量发送
    memcpy(tx_queue.buffer, data, size);
    tx_queue.head = size;
    USART_StartDMA_TX();
}
```

---

## 总结

### 设计优点 ✅

1. **低 CPU 占用**：DMA 承担数据搬运，CPU 利用率 <5%
2. **高实时性**：100μs 定时检查，响应迅速
3. **智能调度**：自动启停 DMA，无缝衔接传输
4. **支持突发数据**：256 字节环形缓冲，抗冲击能力强
5. **模块化设计**：GPIO/USART/DMA/TIM7 分离配置

---

### 待改进项 ⚠️

1. **BUG 修复**：统一 RX DMA Stream 编号 (Stream2 vs Stream5)
2. **恢复打印**：取消注释 `__io_putchar` 实现
3. **并发安全**：添加临界区保护或原子操作
4. **溢出处理**：完善 RingBuffer_Push 满缓冲策略
5. **错误恢复**：添加 DMA 错误中断处理 (FEIF、DMEIF 等)

---

### 推荐使用方式

```c
// 1. 初始化
USART1_Init();

// 2. 启用标准打印（取消注释 __io_putchar）

// 3. 应用中直接使用 printf
printf("Motor Speed: %.2f m/s\r\n", speed);

// 4. 如需发送二进制数据
uint16_t sensor_data[10];
// ... 填充数据 ...
for (int i = 0; i < 10; i++) {
    UART1_SendByte(sensor_data[i] >> 8);  // 高字节
    UART1_SendByte(sensor_data[i] & 0xFF); // 低字节
}
```

---

## 参考资料

- STM32F407 Reference Manual - USART, DMA, TIM 章节
- ARM Cortex-M4 Technical Reference Manual
- 《嵌入式 C 语言自我修养》- 环形缓冲区实现
- 正点原子/野火 STM32F4 开发板教程
