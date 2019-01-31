#include "interrupts.h"

extern int i;
extern uint8_t data[10];
extern int timer_stop_event;
extern int milliseconds;

#define TIMER_PERIOD 1000 /* 1000 == 1 second */

void	print_UART(uint8_t *data) /* len is constantly 10 */
{
  int i = 0;
  while(i < 10)
  {
	putchar_UART(data[i++]);
  }
}

//#include "stdio.h"

//
//INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
//{
//    /* In order to detect unexpected events during development,
//       it is recommended to set a breakpoint on the following instruction.
//    */
//  	/* Write one byte to the transmit data register */
//  //UART2_SendData8('x');
//  //if (!\0)
//  putchar('2');
//
//  //if (TxCounter == TX_BUFFER_SIZE)
//  //{
//    /* Disable the USART Transmit Complete interrupt */
//    UART2_ITConfig(UART2_IT_TXE, DISABLE);
//  //}
//}

 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
	  if (i < 10)
		data[i++] = getchar();
	  else
		i = 0;
	  // ? stop when receive is complete;
      // UART2_ITConfig(UART2_IT_RXNE_OR, DISABLE);
    }
 
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  milliseconds++;
  
  if (milliseconds == (TIMER_PERIOD))
  {
	milliseconds = 0;
	print_UART(data);
	memset(data, 0x0, sizeof(data));
  }
  
  /* Cleat Interrupt Pending bit */
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  
  // ? stop of the timer;
}

