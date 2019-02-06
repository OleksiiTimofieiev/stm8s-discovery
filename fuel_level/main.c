#include "stm8s.h"
#include "config.h"
#include "output.h"
#include "interrupts.h"
#include "infrastructure.h" /* supporting funcs */

uint8_t data_buffer[DATA_BUFFER_SIZE] = { 0x0 };
int	buffer_iterator = 0;
int     timer_stop_event = 0;
int     milliseconds = 0;
bool    byte_received = FALSE;
bool    received_full_packet = FALSE;
uint8_t device_address[4] = { 0x0 };

void    (*p[4]) (uint8_t *request_line);

// TODO: external configuration through UART (do nothing until configured);
// TODO: array of structures with funcs and other details for the different models of the devices;
// TODO: array of function pointers / structs with functions;
// TODO: remaster files to the normal reading format;
// TODO: if we have 00 address reply on the line; (some type of the function) default is not 00 address;
// TODO: handle responses from the separate devices;
// TODO: manage several devices on the line;
//
//ДУТ 
//
//- Конфигурированиях параметры
//- Самодиагностика
//- Калман
//- Передача данных на модем
//- Средние значениz


struct:
  {
    type of the logger
      address of the logger
        request functions array
          handling functions array
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
  t_REQUEST_6_response data; // maximum byte to handle information, if 1-4 byte then bitwise operations;
  
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

void    init_array_of_funcs(void)
{
  p[0] = &send_request;
  p[1] = &send_request;
  p[2] = &send_request;
  p[3] = &send_request;
}

void main( void )
{
  set_up_peripherals();
   
  while (1)
    logic(); /* remaster according to the new architecture format */
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