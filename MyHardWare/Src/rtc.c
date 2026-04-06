
#include "rtc.h"

// 定义一个静态字符数组来存储时间字符串
static char timeBuffer[100];
/**
 * @brief  RTC配置：选择RTC时钟源，设置RTC_CLK的分频系数
 * @param  无
 * @retval 无
 */
void RTC_CLK_Config(void) {
  RTC_InitTypeDef RTC_InitStructure;

  /*使能 PWR 时钟*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* PWR_CR:DBF置1，使能RTC、RTC备份寄存器和备份SRAM的访问 */
  PWR_BackupAccessCmd(ENABLE);

#if defined(RTC_CLOCK_SOURCE_LSI)
  /* 使用LSI作为RTC时钟源会有误差,仅仅是为了实验方便 */

  /* 使能LSI */
  RCC_LSICmd(ENABLE);
  /* 等待LSI稳定 */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
  }
  /* 选择LSI做为RTC的时钟源 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

#elif defined(RTC_CLOCK_SOURCE_LSE)

  /* 使能LSE */
  RCC_LSEConfig(RCC_LSE_ON);
  /* 等待LSE稳定 */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
  }
  /* 选择LSE做为RTC的时钟源 */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

#endif /* RTC_CLOCK_SOURCE_LSI */

  /* 使能RTC时钟 */
  RCC_RTCCLKCmd(ENABLE);

  /* 等待 RTC APB 寄存器同步 */
  RTC_WaitForSynchro();

  /*=====================初始化同步/异步预分频器的值======================*/
  /* 驱动日历的时钟ck_spare = LSE/[(255+1)*(127+1)] = 1HZ */

  /* 设置异步预分频器的值*/
  RTC_InitStructure.RTC_AsynchPrediv = ASYNCHPREDIV;
  /* 设置同步预分频器的值 */
  RTC_InitStructure.RTC_SynchPrediv = SYNCHPREDIV;
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  /* 用RTC_InitStructure的内容初始化RTC寄存器 */
  if (RTC_Init(&RTC_InitStructure) == ERROR) {
    printf("\n\r RTC 时钟初始化失败 \r\n");
  }
}

/**
 * @brief  设置时间和日期
 * @param  无
 * @retval 无
 */
void RTC_TimeAndDate_Set(void) {
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;

  // 初始化时间
  RTC_TimeStructure.RTC_H12 = RTC_H12_AMorPM;
  RTC_TimeStructure.RTC_Hours = HOURS;
  RTC_TimeStructure.RTC_Minutes = MINUTES;
  RTC_TimeStructure.RTC_Seconds = SECONDS;
  RTC_SetTime(RTC_Format_BINorBCD, &RTC_TimeStructure);
  RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);

  // 初始化日期
  RTC_DateStructure.RTC_WeekDay = WEEKDAY;
  RTC_DateStructure.RTC_Date = DATE;
  RTC_DateStructure.RTC_Month = MONTH;
  RTC_DateStructure.RTC_Year = YEAR;
  RTC_SetDate(RTC_Format_BINorBCD, &RTC_DateStructure);
  RTC_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
}
/**
 * @brief  显示时间和日期
 * @param  无
 * @retval 无
 */
char *RTC_GetNowTime(void) {
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;
  // 获取日历
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

  // 打印日期
  sprintf(timeBuffer, "TheDate:  Y:20%0.2d - M:%0.2d - D:%0.2d - W:%0.2d",
          RTC_DateStructure.RTC_Year, RTC_DateStructure.RTC_Month,
          RTC_DateStructure.RTC_Date, RTC_DateStructure.RTC_WeekDay);
  return timeBuffer;
}
