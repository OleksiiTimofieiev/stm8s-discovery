#include "i2c_acc.h"
#include "stm8s_i2c.h"
#include "stdio.h"

void	delay(double time)
{
 	while (time--);
}
  

void	I2C_ACC_Init(void)
{
	/* u8	Input_Clock = 0x0;
	
	Input_Clock = CLK_GetClockFreq() / 1 000 000;
	
	printf("%d\n", Input_Clock); */
	  I2C_Cmd(ENABLE);
	I2C_DeInit();
	
	I2C_Init(I2C_Speed, LSM6DS3_BUS_ADDRESS,
			 I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16 /* InputClockFrequencyMHz */);

	
}

void	I2C_ACC_ByteWrite(u8 I2C_Slave_Address, u8 iData)
{
	/* send start condition */
  	I2C_GenerateSTART(ENABLE);
	
	/* test in I2C event and clear it */
	while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
	
	/* send EEPROM address to write */
	I2C_Send7bitAddress(I2C_Slave_Address<<1, I2C_DIRECTION_TX);
	
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
  	printf("slave -> 0x%x\n", I2C_Slave_Address);
	printf("register address -> 0x%x\n", ReadAddr);
	
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
/*
uint8_t MPU6050ReadReg(uint8_t regaddr)
{
uint8_t tmp;

I2C_GenerateSTART(ENABLE);
while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(MPU6050_Write, I2C_DIRECTION_TX); // Device address and direction
while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

I2C_SendData(regaddr); // Mode register
while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

I2C_GenerateSTOP(ENABLE); // Generating Stop - Ре-Старт не прокатывает без Стоп-а
I2C_GenerateSTART(ENABLE);
while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(MPU6050_Write, I2C_DIRECTION_RX); // Device address and direction
while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
tmp = I2C_ReceiveData(); // Reading data from the buffer

I2C->CR2 &= ~I2C_CR2_ACK; // Sending NACK, so slave will release SDA - Без NACK-а слейв не освобождает линию
I2C_GenerateSTOP(ENABLE); // Send STOP Condition

return tmp;
}
*/