#ifndef __I2C_ACC_H
#define __I2C_ACC_H

#include "stm8s.h"

#define I2C_Speed 			200000

#define MPU_6050_SLAVE_ADDRESS		0x68
#define MPU_6050_WHO_AM_I			0x75

void	I2C_ACC_Init(void);
void	I2C_ACC_ByteWrite(u8 I2C_Slave_Address, u8 iData);
void	I2C_ACC_ByteRead(u8 I2C_Slave_Address, u8 ReadAddr, u8 *pBuffer);

#endif