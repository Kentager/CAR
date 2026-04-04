#include "delay.h"
#include "encoder.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <stdio.h>

int main(void) {
  delay_init();
  USART1_Init();
  Encoder_Driver_Init();
  Encoder_Data_t data;

  while (1) {
    Encoder_Update();
    // printf("Encoder Value: %d\n", encoder_value);
    data = Encoder_GetData(ENCODER_LEFT);
    printf("rleft:%.3f  ", data.total_distance);
    data = Encoder_GetData(ENCODER_RIGHT);
    printf("right:%.3f\r\n", data.total_distance);
    delay_ms(5);
  }
}
