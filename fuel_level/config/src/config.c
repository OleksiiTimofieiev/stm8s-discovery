#include "config.h"

void CLK_Config(void)
{
    /* Initialization of the clock */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

void UART2_Config(void)
{

  // https://en.wikipedia.org/wiki/Baud
  // stop bit and start bit -> 960 bytes per second;
  
  	 // https://en.wikipedia.org/wiki/Baud
  // stop bit and start bit -> 960 bytes per second;
  
  /* Deinitializes the UART1 and UART3 peripheral */
    UART2_DeInit();
    //UART3_DeInit();
    /* UART1 and UART3 configuration -------------------------------------------------*/
    /* UART1 and UART3 configured as follow:
          - BaudRate = 9600 baud  
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Receive and transmit enabled
          - UART1 Clock disabled
     */
    /* Configure the UART1 */
    //UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
               // UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
    
    /* Enable UART1 Transmit interrupt*/
    
    
    /* Configure the UART3 */
 	UART2_Init((uint32_t)115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
                UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);

    /* Enable UART3 Receive interrupt */
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
	UART2_ITConfig(UART2_IT_TXE, ENABLE);
    
	UART2_Cmd(ENABLE);
	
    /* Enable general interrupts */
    enableInterrupts();    
}