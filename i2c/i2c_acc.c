#include "i2c_acc.h"
#include "stm8s_i2c.h"

void	I2C_ACC_Init(void)
{
	u8	Input_Clock = 0x0;
	
	Input_Clock = CLK_GetClockFreq() / 1000000;
	
	I2C_Cmd(ENABLE);
	
	I2C_Init(I2C_Speed, SLAVE_ADDRESS, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, Input_Clock);
	
	
}

void	I2C_ACC_ByteWrite(u8 I2C_Slave_Address, u8 iData)
{
	/* send start condition */
  	I2C_GenerateSTART(ENABLE);
	
	/* test in I2C event and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	/* send EEPROM address to write */
	I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_TX);
	
	/* test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	I2C_SendData(iData);
	
	/* test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
}