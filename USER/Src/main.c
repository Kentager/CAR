#include "delay.h"
#include "led.h"
#include "motor.h"
#include "rtc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "ulog.h"
#include "usart.h"
#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>

void my_console_logger(ulog_level_t severity, const char *msg) {
  printf("%s [%s]: %s",
         RTC_GetNowTime(), // user defined function
         ulog_level_name(severity), msg);
}

void motor_test(void) {
  Motor_SetSpeed(MOTOR_LEFT, 4000);  // 左电机正转，速度 4000
  Motor_SetSpeed(MOTOR_RIGHT, 4000); // 右电机正转，速度 4000
  Motor_SetDirection(MOTOR_LEFT, MOTOR_DIR_FORWARD);
  Motor_SetDirection(MOTOR_RIGHT, MOTOR_DIR_FORWARD);

  // 更新电机状态，将配置应用到硬件
  Motor_Update(MOTOR_LEFT);
  Motor_Update(MOTOR_RIGHT);
}
int main() {
  int arg = 42;
  led_Init();
  LED_Off();
  Motor_Driver_Init();

  motor_test();
  USART1_Init();
  printf("hello world\r\n");
  printf("fdsfd\r\n");
  ULOG_INIT();
  delay_init();

  // log messages with a severity of WARNING or higher to the console.  The
  // user must supply a method for my_console_logger, e.g. along the lines
  // of what is shown above.

  ULOG_SUBSCRIBE(my_console_logger, ULOG_WARNING_LEVEL);

  ULOG_CRITICAL("Critical,arg=100\r\n");
  ULOG_WARNING("Critical, arg=%d\r\n", arg); // logs to file and console
  while (1) {

    GPIO_ToggleBits(GPIOC, 13);
    delay_ms(100);
  }
}
