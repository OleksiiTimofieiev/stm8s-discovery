#include "interrupts.h"

INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  	/* Write one byte to the transmit data register */
  //UART2_SendData8('x');
  putchar('2');

  //if (TxCounter == TX_BUFFER_SIZE)
  //{
    /* Disable the USART Transmit Complete interrupt */
    UART2_ITConfig(UART2_IT_TXE, DISABLE);
  //}
}

 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
      int c = getchar();
      //string[i] = c;
      //i++;
    	  
	  //UART2_ITConfig(UART2_IT_TXE, ENABLE);
	  UART2_ITConfig(UART2_IT_TXE, ENABLE);
	  UART2_SendData8(c);
      //UART2_ITConfig(UART2_IT_RXNE_OR, DISABLE);
    }

