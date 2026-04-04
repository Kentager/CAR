#include "delay.h"
#include "encoder.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <stdio.h>
int main(void) {
  USART1_Init();
  delay_init();
  Encoder_Driver_Init();

  Encoder_Data_t data;

  while (1) {
    Encoder_Update();
    data = Encoder_GetData(ENCODER_RIGHT);
    printf("RIGHT speed:%.3f     ", data.speed_m_s);
    data = Encoder_GetData(ENCODER_LEFT);
    printf("LEFT speed:%.3f\r\n", data.speed_m_s);
    delay_ms(10);
  }
}
