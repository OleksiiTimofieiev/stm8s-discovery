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
  
  while(data[i] != 0x0)
    putchar_UART(data[i++]);
  
  putchar_UART('\n');
}

int     data_buf_length(uint8_t * data_buffer)
{
  int i = 0;
  
  while (data_buffer[i] != 0x0)
    i++;
      
  return (i);
}
