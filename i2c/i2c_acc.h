#ifndef __I2C_ACC_H
#define __I2C_ACC_H

#include "stm8s.h"

#define I2C_Speed 			100000

#define SLAVE_ADDRESS		0x68 //AD0 = 0; ?: (do i need to connect the corresponding pin)
#define WHO_AM_I			0x75

void	I2C_ACC_Init(void);

#endif