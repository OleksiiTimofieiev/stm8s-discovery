#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm8s_uart2.h"

#define DATA_BUFFER_SIZE         9
#define TIM4_PERIOD              124
#define TIMER_PERIOD             3000 /* 1000 == 1 second */
#define REQUEST_6_REPLY_LENGTH   9

/* packet check section */
#define PACKET_PREFIX            0x3E
#define PACKET_NET_ADDRESS       1
#define PACKET_CODE_OPERATION    0x06
#define PACKET_CONTROL_SUM       0x02


void CLK_Config(void);
void UART2_Config(void);
void TIM4_Config_Fuel(void);

#endif