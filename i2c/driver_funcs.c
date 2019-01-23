#include "driver_funcs.h"

void	delay(double time)
{
 	while (time--);
}

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

void	Accel_GetXYZ(int16_t *pData)
{
  uint8_t buffer[6];
  
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTX_L_XL, &buffer[0]);
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTX_H_XL, &buffer[1]);
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTY_L_XL, &buffer[2]);
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTY_H_XL, &buffer[3]);
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTZ_L_XL, &buffer[4]);
  I2C_ACC_ByteRead(LSM6DS3_BUS_ADDRESS, LSM6DS3_ACC_GYRO_OUTZ_H_XL, &buffer[5]);

  for (uint8_t i = 0; i < 3; i++)
  {
	pData[i] = ((int16_t)((uint16_t)buffer[2 * i + 1] << 8) + buffer[2 * i]);
  }
  
}

void	Accel_Read(void)
{
  int16_t buffer[3] = {0};
  int16_t xval, yval, zval;
  char	str[100] = {'\0'};
  
  Accel_GetXYZ(buffer);
  
  xval = buffer[0];
  yval = buffer[1];
  zval = buffer[2];
  
  sprintf(str, "X:%06d Y:%06d Z:%06d\r\n", xval, yval, zval);
  printf("%s\r\n", str);
}
