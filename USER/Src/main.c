#include "delay.h"
#include "led.h"
#include "rtc.h"
#include "stm32f4xx_gpio.h"
#include "ulog.h"
#include "usart.h"
#include <stdio.h>
#include <stm32f4xx.h>

// To use uLog, you must define a function to process logging messages. It can
// write the messages to a console, to a file, to an in-memory buffer: the
// choice is yours.  And you get to choose the format of the message.
//
// The following example prints to the console.
//
// One caveat: msg is a static string and will be over-written at the next call
// to ULOG.  This means you may print it or copy it, but saving a pointer to it
// will lead to confusion and astonishment.
//
void my_console_logger(ulog_level_t severity, const char *msg) {
  printf("%s [%s]: %s",
         RTC_GetNowTime(), // user defined function
         ulog_level_name(severity), msg);
}

int main() {
  int arg = 42;
  led_Init();

  Usart_Config();
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
