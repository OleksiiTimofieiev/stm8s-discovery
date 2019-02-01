#include "config.h"

void CLK_Config(void)
{
    /* Initialization of the clock */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

void UART2_Config(void)
{ 
    /* Deinitializes the UART1 and UART3 peripheral */
    UART2_DeInit();
    
    /* Configure the UART2 */
    UART2_Init((uint32_t)115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
               UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);

    /* Enable UART2 Receive interrupt */
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
    // UART2_ITConfig(UART2_IT_TXE, ENABLE); /* TX interrupt activation */
    
    /* enable UART2 peripheral */
    UART2_Cmd(ENABLE);
	
    /* Enable general interrupts */
    enableInterrupts(); /* all activated interrupts have to be defined in the vector table */
}

void TIM4_Config_Fuel(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  /* enable interrupts */
  enableInterrupts();
  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}
