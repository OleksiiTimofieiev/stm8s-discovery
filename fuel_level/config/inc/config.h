#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm8s_uart2.h"

#define TIM4_PERIOD       124
#define TIMER_PERIOD      3000 /* 1000 == 1 second */
#define REQUEST_6_REPLY   7

void CLK_Config(void);
void UART2_Config(void);
void TIM4_Config_Fuel(void);

#endif