#include "i2c_acc.h"
#include "stm8s_i2c.h"
#include "stdio.h"

void	I2C_ACC_Init(void)
{
	/* u8	Input_Clock = 0x0;
	
	Input_Clock = CLK_GetClockFreq() / 1 000 000;
	
	printf("%d\n", Input_Clock); */
	
	I2C_Cmd(ENABLE);
	
	I2C_Init(I2C_Speed /* 100 000 */, MPU_6050_SLAVE_ADDRESS /* 0x68 */,
			 I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16 /* Input_Clock */);
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
	
	/* send stop condition */
  	I2C_GenerateSTOP(ENABLE);
}

void	I2C_ACC_ByteRead(u8 I2C_Slave_Address, u8 ReadAddr, u8 *pBuffer)
{
  	printf("slave -> %d\n", I2C_Slave_Address);
	printf("register address -> %d\n", ReadAddr);
	
  	/* while the bus is busy */
  	while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
	
	/* send start condition */
  	I2C_GenerateSTART(ENABLE);
	
	/* test in I2C event and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	/* send address to write */
	I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_TX);
	
	/* test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	/* send the address of the first byte to be read and wait event detection */
	I2C_SendData(ReadAddr); /* LSB */
	
	/* test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	/* send start condition for the second time */
  	I2C_GenerateSTART(ENABLE);
	
	/* test in I2C event and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	/* send EEPROM address to write */
	I2C_Send7bitAddress(I2C_Slave_Address, I2C_DIRECTION_RX);
	
	/* test on EV and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
	
	*pBuffer = I2C_ReceiveData();
	
	I2C_AcknowledgeConfig(I2C_ACK_NONE);
	
	/* send stop condition */
  	I2C_GenerateSTOP(ENABLE);
	
}