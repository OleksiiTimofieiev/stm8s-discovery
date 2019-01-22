#ifndef __I2C_ACC_H
#define __I2C_ACC_H

#include "stm8s.h"

#define I2C_Speed 			10000
#define LSM6DS3_BUS_ADDRESS			0xD4
#define WHO_AM_I					0x0F

void	I2C_ACC_Init(void);
void	I2C_ACC_ByteWrite(u8 I2C_Slave_Address, u8 iData);
void	I2C_ACC_ByteRead(u8 I2C_Slave_Address, u8 ReadAddr, u8 *pBuffer);

#endif