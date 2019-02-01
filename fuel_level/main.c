#include "stm8s.h"
#include "config.h"
#include "output.h"
#include "interrupts.h"

uint8_t data_buffer[30] = { 0x0 };
int	buffer_iterator = 0;
int     timer_stop_event = 0;
int     milliseconds = 0;
bool    byte_received = FALSE;

void    set_up_peripherals(void)
{
  CLK_Config();
  UART2_Config();
  TIM4_Config_Fuel();
}

void main( void )
{
  set_up_peripherals();
   
  while (1)
  {
	;
  }
}

void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}