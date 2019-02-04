#include "stm8s.h"
#include "config.h"
#include "output.h"
#include "interrupts.h"
#include "infrastructure.h" /* supporting funcs */

// TODO: send / response logic;

uint8_t data_buffer[DATA_BUFFER_SIZE] = { 0x0 };
int	buffer_iterator = 0;
int     timer_stop_event = 0;
int     milliseconds = 0;
bool    byte_received = FALSE;
bool    received_full_packet = FALSE;

void    sendRequest(void)
{
  putchar_UART('1');
  putchar_UART('2');
  putchar_UART('3');
}

typedef struct  s_REQUEST_6_response
{
  uint8_t       temperature;
  uint16_t      relative_level;
  uint16_t      frequency;
}               t_REQUEST_6_response;

t_REQUEST_6_response get_data(uint8_t *data_buffer)
{
  t_REQUEST_6_response  data;
  
  data.temperature = data_buffer[3];
  data.relative_level = (uint16_t) (data_buffer[4] << 8 | data_buffer[5]);
  data.frequency = (uint16_t) (data_buffer[6] << 8 | data_buffer[7]);
  
  return (data);
}

void    zero(t_REQUEST_6_response *data)
{
  data->temperature = 0;
  data->relative_level = 0;
  data->frequency = 0;
}

bool    packet_validation(uint8_t *data_buffer)
{
//  uint8_t x = crc8(data_buffer, 8);
//  putchar_UART(x);
  
  if 
    (
        data_buffer[0] == PACKET_PREFIX &&
        data_buffer[1] == PACKET_NET_ADDRESS &&
        data_buffer[2] == PACKET_CODE_OPERATION &&
        data_buffer[8] == crc8(data_buffer, REQUEST_6_REPLY_LENGTH - 1)
     )
    return (TRUE);
  return (FALSE);
}

void    logic(void)
{
  t_REQUEST_6_response data;
  
  if (received_full_packet) /* add data processing routines */
  {
//      putchar_UART('a');
      zero(&data);
    
      received_full_packet = FALSE;
      if (packet_validation(data_buffer))
        print_UART(data_buffer); /* handle information */
      buffer_iterator = 0;
      memset(data_buffer, 0x0, sizeof(data_buffer));
      byte_received = FALSE;
  }
  else if (buffer_iterator == REQUEST_6_REPLY_LENGTH)
  {
//       putchar_UART('b');
      if (packet_validation(data_buffer))
      {
//        print_UART(data_buffer); /* handle information */
        zero(&data);
        
        data = get_data(data_buffer);
        
        putchar_UART(data.temperature);
        putchar_UART(data.relative_level);
        putchar_UART(data.frequency);
      }
      buffer_iterator = 0;
      memset(data_buffer, 0x0, sizeof(data_buffer));
      byte_received = FALSE;
  } 
//   else if (byte_received == FALSE) /* clear these stuff */
//  {
////    putchar_UART('c');
//    buffer_iterator = 0;
//    memset(data_buffer, 0x0, sizeof(data_buffer));
//    
////    for (int i = 0; i < 100000; i++); 
//  }
}

void main( void )
{
  set_up_peripherals();
  
  sendRequest();
   
  while (1)
    logic(); /* pass struct here, struct has to be for all params with u16 or float ? */
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