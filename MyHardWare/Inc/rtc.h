#ifndef __RTC_H
#define __RTC_H
#include "stm32f4xx.h"

#include "stdio.h"
#define RTC_CLOCK_SOURCE_LSE

// 异步分频因子
#define ASYNCHPREDIV 0X7E
// 同步分频因子
#define SYNCHPREDIV 0XFE

// 时间宏定义
#define RTC_H12_AMorPM RTC_H12_AM
#define HOURS 0   // 0~23
#define MINUTES 0 // 0~59
#define SECONDS 0 // 0~59

// 日期宏定义
#define WEEKDAY 0 // 1~7
#define DATE 0    // 1~31
#define MONTH 0   // 1~12
#define YEAR 0    // 0~99

// 时间格式宏定义
#define RTC_Format_BINorBCD RTC_Format_BIN

// 备份域寄存器宏定义
#define RTC_BKP_DRX RTC_BKP_DR0
// 写入到备份寄存器的数据宏定义
#define RTC_BKP_DATA 0X32F1

void RTC_TimeAndDate_Set(void);
void RTC_CLK_Config(void);
char *RTC_GetNowTime(void);
#endif // !__RTC_H