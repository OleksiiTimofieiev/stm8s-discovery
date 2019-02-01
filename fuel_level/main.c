#include "stm8s.h"
#include "config.h"
#include "output.h"
#include "interrupts.h"
#include "infrastructure.h" /* supporting funcs */

uint8_t data_buffer[30] = { 0x0 };
int	buffer_iterator = 0;
int     timer_stop_event = 0;
int     milliseconds = 0;
bool    byte_received = FALSE;
bool    received_full_packet = FALSE;


/*
logic

int length = data_buf_length(data_buffer);
                
                if (length == REQUEST_6_REPLY)
                  memset(data_buffer, 0x0, sizeof(data_buffer));
*/

void main( void )
{
  set_up_peripherals();
   
  while (1)
  {
    if (received_full_packet)
    {
      //putchar_UART('a');
      received_full_packet = FALSE;
      
      
      print_UART(data_buffer);
      buffer_iterator = 0;
      memset(data_buffer, 0x0, sizeof(data_buffer));
    }
    else if (data_buf_length(data_buffer) == REQUEST_6_REPLY)
    {
      //putchar_UART('b');
      print_UART(data_buffer);
      buffer_iterator = 0;
      memset(data_buffer, 0x0, sizeof(data_buffer));
    }
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