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
  
  while(i < 9)
    putchar_UART(data[i++]);
  if (i != 0x0)
    putchar_UART('\n');
}

int     data_buf_length(uint8_t * data_buffer)
{
  int i = 0;
  
  while (data_buffer[i] != 0x0)
    i++;
      
  return (i);
}

uint8_t crc8(uint8_t *data, int len) /* len == 8, packet without CRC */
{
 int j;
 
 uint8_t crc = 0;
 uint8_t i;
 
 for(j = 0; j < len; j++)
 {
   i = crc ^ data[j];
   crc = 0;
   if(i & 0x01) crc ^= 0x5e;
   if(i & 0x02) crc ^= 0xbc;
   if(i & 0x04) crc ^= 0x61;
   if(i & 0x08) crc ^= 0xc2;
   if(i & 0x10) crc ^= 0x9d;
   if(i & 0x20) crc ^= 0x23;
   if(i & 0x40) crc ^= 0x46;
   if(i & 0x80) crc ^= 0x8c;
 }
 return (crc);
}

void    send_request(uint8_t * request_line)
{
  while (*request_line)
    putchar_UART(*request_line++);
}