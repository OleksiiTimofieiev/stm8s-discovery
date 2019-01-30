#include "stm8s.h"
#include "stdio.h"
#include "stdlib.h"
#include "i2c_acc.h"
#include "driver_funcs.h"

#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)

/*
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  switch (I2C_GetLastEvent())
  {
      // EV5 /
    case I2C_EVENT_MASTER_MODE_SELECT :
	  printf("test\r\n");
      break;
    default:
      break;
  }
}
*/

char string[6] = { 0x0 }; /* may be used like extern variable */
int   i = 0; /* may be used like extern variable */
#define TIM4_PERIOD       124

int timer_stop_event = 0;
int millisecond = 0;

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  millisecond++;
  
  if (millisecond == 1000)
  {
  	printf("We have a timer\n");
	millisecond = 0;
	timer_stop_event++;
  }
  
  /* Cleat Interrupt Pending bit */
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  
  if (timer_stop_event == 50)
  {
	printf("STOP\n");
	TIM4_Cmd(DISABLE);
  }

}

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)

 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  	/* Write one byte to the transmit data register */
  UART2_SendData8('x');

  //if (TxCounter == TX_BUFFER_SIZE)
  //{
    /* Disable the USART Transmit Complete interrupt */
    UART2_ITConfig(UART2_IT_TXE, DISABLE);
  //}
}


 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
    /* Read one byte from the receive data register */
    //RxBuffer2[IncrementVar_RxCounter2()] = UART3_ReceiveData8();

    //if (GetVar_RxCounter2() == GetVar_NbrOfDataToRead2())
    //{
    /* Disable the UART2 Receive interrupt */
        
    if (i <= 5)
    {
      char c = UART2_ReceiveData8();
      string[i] = c;
      i++;
    }
    else
    {
	  UART2_ITConfig(UART2_IT_TXE, ENABLE);
	  UART2_SendData8('x');
      printf("%s\n", "interrupt stopper");
      UART2_ITConfig(UART2_IT_RXNE_OR, DISABLE);
      printf("%s\n", string);
    }
}

#endif /* STM8S105*/

PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART2 */
  UART2_SendData8(c);
  /* Loop until the end of transmission */
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET);
    c = UART2_ReceiveData8();
  return (c);
}

static void CLK_Config(void)
{
    /* Initialization of the clock */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	//CLK_HSECmd(1);
}

static void UART_Config(void)
{

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

static void TIM4_Config(void)
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

void main( void )
{
  i = 0;
  CLK_Config();
  //I2C_ACC_Init();
  
   // 	/* while the bus is busy */
  	//while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
	
	///* send start condition */
  	//I2C_GenerateSTART(ENABLE);
  
  
  
  UART_Config();  
  
  putchar('z');
  // TIM4_Config();
  
  // if (availability())
  //{
  	//init_accelerometer();
	
	while (1)
  	{
   		 ;
 	}
  //}
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
