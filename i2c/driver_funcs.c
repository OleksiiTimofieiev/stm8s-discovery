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

void	init_accelerometer(void)
{
  uint8_t value = 0;
  
  /* increment register address incrementation when reading from them */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL3_C, &value);
  value &=~LSM6DS3_ACC_GYRO_IF_INC_MASK;
  value |= LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL3_C, value);
  
  /*
  uint8_t control = 0;
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL3_C, &control);
  
  printf("reg value -> %d\n", control);
  */
  /* setting BDU update -> to avoid reading bytes of the different readings (bytes from the different measurements) */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL3_C, &value);
  value &=~LSM6DS3_ACC_GYRO_BDU_MASK;
  value |= LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL3_C, value);
  
  /* disable FIFO -> bypass mode */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_FIFO_CTRL5, &value);
  value &=~LSM6DS3_ACC_GYRO_FIFO_MODE_MASK;
  value |= LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_FIFO_CTRL5, value);
  
  /* turn off device == 0000 */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, &value);
  value &=~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, value);
  
  /* scale selection == 2g */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, &value);
  value &=~LSM6DS3_ACC_GYRO_FS_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, value);
  
  /* enabling x.y.z */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL9_XL, &value);
  value &=~ (	LSM6DS3_ACC_GYRO_XEN_XL_MASK |\
				LSM6DS3_ACC_GYRO_YEN_XL_MASK |\
			  	LSM6DS3_ACC_GYRO_ZEN_XL_MASK	);
  value |= (	LSM6DS3_ACC_GYRO_XEN_XL_ENABLED|\
				LSM6DS3_ACC_GYRO_YEN_XL_ENABLED|\
				LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED	);
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL9_XL, value);
  
   /* turn on device */
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, &value);
  value &=~LSM6DS3_ACC_GYRO_ODR_XL_MASK;
  value |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
  I2C_ACC_RegWrite(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_CTRL1_XL, value);
}
