#include "interrupts.h"

extern int i;
extern uint8_t data[10];

#include "stdio.h"
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
	  else {
		i = 0;
		
		while (i < 10)
		{
		 // printf("%c", (char)data[i++]); /* not working */
		  putchar_UART(data[i++]);
		}
		i = 0;
	  }
	  
      //string[i] = c;
      //i++;
    	  
	  //UART2_ITConfig(UART2_IT_TXE, ENABLE);
	  //UART2_ITConfig(UART2_IT_TXE, ENABLE);
	  
	  // ? stop when receive is complete;
      //UART2_ITConfig(UART2_IT_RXNE_OR, DISABLE);
    }

