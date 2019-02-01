#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H

#include "stm8s.h"
#include "output.h"
#include "string.h"
#include "infrastructure.h"
#include "config.h"
#include "stdlib.h"

INTERRUPT void UART2_RX_IRQHandler(void); /* UART2 RX */
// INTERRUPT void UART2_TX_IRQHandler(void); /* UART2 TX */

#endif
