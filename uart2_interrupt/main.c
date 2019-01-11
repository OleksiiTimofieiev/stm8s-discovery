#include "stm8s.h"
#include "stdio.h"

INTERRUPT_HANDLER(UART2_RX_IRQHandler, 18)
{
  char c;
 
  c = UART2_ReceiveData8();
  
   /* Write a character to the UART2 */
  UART2_SendData8(c);
  /* Loop until the end of transmission */
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
  
    /* Read one byte from the receive data register */
    //RxBuffer1[IncrementVar_RxCounter1()] = UART1_ReceiveData8();

    //if (GetVar_RxCounter1() == GetVar_NbrOfDataToRead1())
    //{
        /* Disable the UART1 Receive interrupt */
     //   UART1_ITConfig(UART1_IT_RXNE_OR, DISABLE);
   // }
  
}


void main( void )
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  
  UART2_DeInit();
  
  UART2_Init((uint32_t)115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
              UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
  
  /* Enable UART2 Receive interrupt */
  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
    
  /* Enable general interrupts */
  enableInterrupts();  
  
  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
  
  while (1)
  {
    ;
  }
  
 
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
