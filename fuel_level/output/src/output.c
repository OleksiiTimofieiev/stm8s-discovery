#include "output.h"

void putchar_UART (int c)
{
  /* Write a character to the UART2 */
  UART2_SendData8(c);
  
  /* Loop until the end of transmission */
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}

int getchar (void)
{
  int c = 0;

  /* Loop until the Read data register flag is SET */
  while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET);
  
  c = UART2_ReceiveData8();
  
  return (c);
}