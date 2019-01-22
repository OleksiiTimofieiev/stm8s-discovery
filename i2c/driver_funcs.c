#include "driver_funcs.h"

u8	availability(void)
{
	u8		iTmp = 0;
	I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, WHO_AM_I, &iTmp);
  
	if ( iTmp == LSM6DS3_DEVICE_ID )
	 return (1);
	else
	  return (0);
}