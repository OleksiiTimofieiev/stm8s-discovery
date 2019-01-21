#include "stm8s.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm8s_i2c.h"
#include "i2c_acc.h"

#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)

char string[6] = { 0x0 }; /* may be used like extern variable */
int   i = 0; /* may be used like extern variable */
u8		iTmp = 0;

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)

 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
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
}

static void UART_Config(void)
{
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
    //UART1_ITConfig(UART1_IT_TXE, ENABLE);
    
    /* Configure the UART3 */
 		UART2_Init((uint32_t)115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
                UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);

    /* Enable UART3 Receive interrupt */
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
    
    /* Enable general interrupts */
    enableInterrupts();    
}

void main( void )
{
  CLK_Config();
  
  I2C_ACC_Init();

  UART_Config();  
  
  printf("%s", "\n\ri2c test\n\r");
  
  I2C_ACC_ByteRead(MPU_6050_SLAVE_ADDRESS, MPU_6050_WHO_AM_I, &iTmp);
  
  if ( iTmp == 0x68 )
  {
	putchar('+');
	iTmp = 0;
	I2C_ACC_ByteRead(MPU_6050_SLAVE_ADDRESS, MPU_6050_WHO_AM_I, &iTmp);
  
	if ( iTmp == 0x68 )
	  putchar('+');
	else
	  putchar('-');
  }
  else
  {
	putchar('-');
  }
	
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
