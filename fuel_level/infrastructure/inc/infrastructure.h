#ifndef __INFRASTRUCTURE_H
#define __INFRASTRUCTURE_H

#include "config.h"
#include "output.h"

void    set_up_peripherals(void);
void	print_UART(uint8_t *data);
int     data_buf_length(uint8_t * data_buffer);
uint8_t crc8(uint8_t *data, int len);
void    send_request(uint8_t * request_line);


#endif