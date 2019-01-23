#ifndef __DRIVER_FUNCS_H
#define __DRIVER_FUNCS_H

#include "lsm6ds3.h"
#include "stm8s_i2c.h"
#include "i2c_acc.h"
#include "stdio.h"

u8		availability(void);
void	init_accelerometer(void);

#endif