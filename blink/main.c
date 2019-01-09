#include "stm8s.h"

void    delay(void)
{
  for(unsigned long delay_count = 0; delay_count < 20000; delay_count++);
}

void main( void )
{
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
  
  while (1)
  {
    GPIO_WriteHigh(GPIOD, GPIO_PIN_0);
    delay();
    GPIO_WriteLow(GPIOD, GPIO_PIN_0);
    delay();
  }
}
