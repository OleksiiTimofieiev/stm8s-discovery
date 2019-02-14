#include "interrupts.h"

extern int  logger_init_request_status;
extern int buffer_iterator;
extern uint8_t data_buffer[10];
extern int timer_stop_event;
extern int milliseconds;
extern bool byte_received;
extern bool received_full_packet;


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
  // switch case for the buffers according to the request timeout;
	  //if (i < 10)
	  //{
		data_buffer[buffer_iterator++] = getchar();
		//putchar_UART('x');
		byte_received = TRUE;
	  //}
	  //else
		//i = 0;
	  // ? stop when receive is complete;
      // UART2_ITConfig(UART2_IT_RXNE_OR, DISABLE);
    }
 
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) /* flag */
{
  milliseconds++;
  
  if (logger_init_request_status == 0)
    send_request("a");
  else if (logger_init_request_status == 1)
    send_request("b");
  
  if (milliseconds == TIMER_PERIOD)
  {
	milliseconds = 0;
	// print_UART(data);
	// memset(data, 0x0, sizeof(data));
	if (byte_received == TRUE)
	{
	  //putchar_UART('1');
	  byte_received = FALSE;
	  //buffer_iterator = 0;
	}
	else
          received_full_packet = TRUE;
	  //print_UART(data_buffer);
  }
  
  /* Cleat Interrupt Pending bit */
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  // ? stop of the timer;
}

