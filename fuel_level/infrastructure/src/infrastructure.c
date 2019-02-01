#include "infrastructure.h"

void    set_up_peripherals(void)
{
  CLK_Config();
  UART2_Config();
  TIM4_Config_Fuel();
}

void	print_UART(uint8_t *data) /* len is constantly 10 */
{
  int i = 0;
  while(i < 10)
    putchar_UART(data[i++]);
  putchar_UART('\n');
}

