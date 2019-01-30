#include "stm8s.h"
#include "config.h"
#include "output.h"
#include "interrupts.h"

#include "stdio.h"

uint8_t data[10] = { 0x0 };
int	i;

void main( void )
{
 // printf("%s\n", "fuel_stuff");
  i = 0;
  CLK_Config();
  UART2_Config();
  
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