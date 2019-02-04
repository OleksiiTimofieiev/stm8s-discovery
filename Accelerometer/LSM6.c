/*
 * LSM6.c
 *
 *  Created on: 30 марта 2017
 *      Author: SnowBars
 */
#include <stdbool.h>
#include <math.h>
#include "bg_types.h"
#include "em_letimer.h"

#include "stdint.h"

#include "em_timer.h"

#include "em_usart.h"
#include "em_emu.h"
#include "em_cmu.h"

#include "em_system.h"

#include "em_gpio.h"
#include "em_usart.h"

#include "em_i2c.h"

#include "i2cspm.h"

#include "delay.h"

#include "LSM6.h"
#include "cardio_types.h"

LSM6_pin_t LSM6_pins[] = {{.LSM6_pin_name = LSM6_DRDY_PIN,    .LSM6_GPIO_Port = gpioPortA,  .LSM6_GPIO_Pin = 1},
                          {.LSM6_pin_name = LSM6_PWDN_PIN,    .LSM6_GPIO_Port = gpioPortB,  .LSM6_GPIO_Pin = 13}
						};


static const long b1_down[3] = {1.248212558582122,-0.24042093579640966, -0.029354866409124217};
static const double b2_down[5] = {3.0506279710853121,-0.92493932084504449,-0.29881031338260411,-2.6822848994878346, 0.24005450090990482};
static const double IW1_1_down[9] = {2.3610236781601022, -4.120889087562583, 0.68055861851764021,0.46564545105863037, 0.18407538531200585, -1.5902522787580524,0.0141957634476369, 0.96817747835785961, 0.47989259083344787};
static const double LW2_1_down[15] = {3.3813241694056173, 1.8986243284429234, 1.6014798899231502,-0.32610259499260302, -10.233525014234628, 6.8477889729124044,-1.9835797795096723, 6.504353561849392, -14.350856362376048,-1.8494331240738955, 14.930202928882226, 11.335430290262895, 0.33955376698740741, -12.591523799016493, -7.2036875940834735};



static const double b1_up[3] = {0.53030431405081546, 0.13361225870955609,1.343710485165803};
static const double b2_up[5] = {-3.5598094065130832, 1.6930224317254083, 2.7653410935809593, 1.2690292406503001, -2.0793357301133968};
static const double IW1_1_up[9] = {1.6893814908424791, 3.2690330061454578, -0.14324263797846884, 0.16561307526740368, 0.79799373530113094, 1.5634286140837816,-1.783333396118236, 3.4664575795542709, -3.0366664188707229};
static const double LW2_1_up[15] = {-7.2967778864440165, 6.8662662814393345, 4.858051926560738, -0.14361468919319531, 3.3704200310496333, -2.4621404618291245, 3.7468121144998108, -6.0836078553452895, -0.55437134719456849,-3.0685903862921879, -4.2358125008548937, -4.7002822445929882, 5.438664212427164, 2.2258753450070619, 2.0617062930604018};

void LSM6_Read(uint8_t reg, uint8_t *data, uint8_t rLen)
{
	uint8_t i, tmp;
	//BUS_RegBitWrite(&GPIO->P[gpioPortD].DOUT, 13, 1);
	//delay_ms(1);
	BUS_RegBitWrite(&GPIO->P[gpioPortC].DOUT, 10, 0);
	//delay_ms(3);
	reg|=0x80;
	USART_SpiTransfer(USART1, (reg));
	for(i=0; i<rLen; i++)
	{
		tmp=USART_SpiTransfer(USART1, NEAD_FOR_READ_LSM);
		data[i]=tmp;
	}
	//delay_ms(2);
	BUS_RegBitWrite(&GPIO->P[gpioPortC].DOUT, 10, 1);
}

void LSM6_Write(uint8_t reg, uint8_t rVal)
{
	BUS_RegBitWrite(&GPIO->P[gpioPortC].DOUT, 10, 0);

	USART_SpiTransfer(USART1, reg);
	USART_SpiTransfer(USART1, rVal);

	BUS_RegBitWrite(&GPIO->P[gpioPortC].DOUT, 10, 1);
}




uint16_t LSM6_GetPin(LSM6_pin_en LSM6_pin_name)
{
//	return BUS_RegBitRead(&GPIO->P[LSM6_pins[LSM6_pin_name].LSM6_GPIO_Port].DOUT, LSM6_pins[LSM6_pin_name].LSM6_GPIO_Pin);
}

void LSM6__SetPin(LSM6_pin_en LSM6_pin_name, eLSM6_Bit_State_t BitVal)
{
	//BUS_RegBitWrite(&GPIO->P[LSM6_pins[LSM6_pin_name].LSM6_GPIO_Port].DOUT, LSM6_pins[LSM6_pin_name].LSM6_GPIO_Pin, BitVal);
}


uint8_t LSM6DS3_ADDRESS = LSM6DS3_ADD_1;
uint8_t IMUBuffer[22] = {0};
GyroRawData gyroOffsets;
MotionData motionDataF;

ImuRawData RawData;

sPedArch_t PedArch;
sAngleAc_t g_AngleAc;
sBodyState_t BodyState;
sAngleGyro_t AngleGyro;

static const float LSM6DS3_FS_XL_SENSITIVITY[] = {0.000061f, 0.000488f, 0.000122f, 0.000244f};
static const float LSM6DS3_FS_G_SENSITIVITY[] = {0.00875f, 0.0175f, 0.035f, 0.07f};

LSM6DS3_FS_XL_t currentFSXL;
LSM6DS3_FS_G_t currentFSG;


status_t I2CReadByte(uint8_t addr, uint8_t reg, uint8_t* data)
{
	  I2C_TransferSeq_TypeDef    seq;
	  I2C_TransferReturn_TypeDef ret;
	  I2C_TypeDef *i2c=I2C0;

	  uint8_t                    i2c_read_data[2];
	  uint8_t                    i2c_write_data[1];

	  seq.addr  = addr;
	  seq.flags = I2C_FLAG_WRITE_READ;
	  /* Select command to issue */
	  i2c_write_data[0] = reg;
	  //i2c_write_data[1] = byte;
	  seq.buf[0].data = i2c_write_data;
	  seq.buf[0].len  = 1;	  /* Select location/length of data to be read */
	  seq.buf[1].data = i2c_read_data;
	  seq.buf[1].len  = 1;

	//  ret = I2CSPM_Transfer(i2c, &seq);

	  if (ret != i2cTransferDone)
	  {
	    *data = 0;
	    return((int) ret);
	  }

	  *data = i2c_read_data[0];
	return IMU_SUCCESS;
}

uint8_t Accel_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
	uint8_t byte;
	I2CReadByte(DeviceAddr, RegisterAddr, &byte);
	return byte;
}

void Accel_GetXYZ(int16_t* pData)
{
	uint8_t buffer[8];
	uint8_t i=0;
	buffer[0]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTX_L_XL);
	buffer[1]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTX_H_XL);
	buffer[2]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTY_L_XL);
	buffer[3]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTY_H_XL);
	buffer[4]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTZ_L_XL);
	buffer[5]=Accel_IO_Read(0xD4,LSM6DS3_ACC_GYRO_OUTZ_H_XL);
	buffer[6]=Accel_IO_Read(0xD4,LSM6DS3_OUT_TEMP_L);
	buffer[7]=Accel_IO_Read(0xD4,LSM6DS3_OUT_TEMP_H);

	for(i=0;i<4;i++)
	{
		pData[i] = (int16_t)(((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);
	}
}
status_t I2CWriteByte(uint8_t addr, uint8_t reg, uint8_t byte)
{
	  I2C_TransferSeq_TypeDef    seq;
	  I2C_TransferReturn_TypeDef ret;
	  uint8_t                    i2c_read_data[2];
	  uint8_t                    i2c_write_data[2];
	  I2C_TypeDef *i2c=I2C0;
	  seq.addr  = addr;
	  seq.flags = I2C_FLAG_WRITE;
	  /* Select command to issue */
	  i2c_write_data[0] = reg;
	  i2c_write_data[1] = byte;
	  seq.buf[0].data   = i2c_write_data;
	  seq.buf[0].len    = 2;
	  /* Select location/length of data to be read */
	  seq.buf[1].data = i2c_read_data;
	  seq.buf[1].len  = 0;

//	  ret = I2CSPM_Transfer(i2c, &seq);

	  if (ret != i2cTransferDone)
	  {
	    return((int) ret);
	  }
	return IMU_SUCCESS;
}

void Accel_IO_Write(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
	I2CWriteByte(DeviceAddr, RegisterAddr, Value);
}



status_t I2CReadBytes(uint8_t adr, uint8_t reg, uint8_t len, uint8_t* buf)
{
	I2C_TransferSeq_TypeDef    seq;
		  I2C_TransferReturn_TypeDef ret;
		  uint8_t                    i2c_read_data[32];
		  uint8_t                    i2c_write_data[2];
		  I2C_TypeDef *i2c=I2C0;
		  seq.addr  = adr;
		  seq.flags = I2C_FLAG_WRITE_READ;
		  /* Select command to issue */
		  i2c_write_data[0] = reg;
	//	  i2c_write_data[1] = 1;
		  seq.buf[0].data   = i2c_write_data;
		  seq.buf[0].len    = 1;
		  /* Select location/length of data to be read */
		  seq.buf[1].data = i2c_read_data;
		  seq.buf[1].len  = len;

		//  ret = I2CSPM_Transfer(i2c, &seq);

		  if (ret != i2cTransferDone)
		  {
		    return((int) ret);
		  }
		  memcpy(&buf[0], (uint8_t*)&i2c_read_data[0], len);

	return IMU_SUCCESS;
}


status_t initIMU()
{
    uint8_t id = 0;

    if (getIMUDeviceID(&id) == IMU_SUCCESS)
    {
        if (id == LSM6DS3_DEVICE_ID)
        {
        	resetIMU();
        	sleeptAccel();
        	initAccel();
        	initGyro();
        	//calibrateLSM6DS3();
        	initPed();
         	initInterrupts();
        	return IMU_SUCCESS;
        }
    }
    else
    {
      LSM6DS3_ADDRESS = LSM6DS3_ADD_2;

      if (getIMUDeviceID(&id) == IMU_SUCCESS)
      {
        if (id == LSM6DS3_DEVICE_ID)
        {
        	resetIMU();
        	sleeptAccel();
        	initAccel();
        	initGyro();
        	//calibrateLSM6DS3();
        	initPed();
        	initInterrupts();
        return IMU_SUCCESS;
        }
      }
    }

      return IMU_ERROR;
}

status_t getIMUDeviceID(uint8_t *id)
{
    if (id == 0)
        id = IMUBuffer;
    LSM6_Read(LSM6DS3_WHO_AM_I_REG, id, 1);
    return IMU_SUCCESS;
}

status_t resetIMU()
{
    LSM6_Write((uint8_t)LSM6DS3_CTRL3_C, (uint8_t)LSM6DS3_SW_RESET_RESET_DEVICE);

     return IMU_SUCCESS;
}



status_t calibrateLSM6DS3()
{
    uint16_t i = 0;
    GyroRawData rotation;

    gyroOffsets.gx = gyroOffsets.gy = gyroOffsets.gz = 0;



    for (i = 0; i < 1000; i += 10)
    {
        getRawRotation(&rotation.gx, &rotation.gy, &rotation.gz);

        if (i == 0)
        {
            gyroOffsets.gx = rotation.gx;
            gyroOffsets.gy = rotation.gy;
            gyroOffsets.gz = rotation.gz;
        }
        else
        {
            gyroOffsets.gx = (gyroOffsets.gx + rotation.gx) / 2;
            gyroOffsets.gy = (gyroOffsets.gy + rotation.gy) / 2;
            gyroOffsets.gz = (gyroOffsets.gz + rotation.gz) / 2;
        }
        delay_ms(10);
    }

}

status_t getMotionT(MotionData *motionData)
{
    ImuRawData imuData;
    status_t s;

    s=getRawMotionT(&RawData);
    motionData->T = calculateTemperature(RawData.T);
    motionData->ax = calculateAcceleration(RawData.ax);
    motionData->ay = calculateAcceleration(RawData.ay);
    motionData->az = calculateAcceleration(RawData.az);
    motionData->gx = calculateRotation(RawData.gx);
    motionData->gy = calculateRotation(RawData.gy);
    motionData->gz = calculateRotation(RawData.gz);

    return s;
}

float calculateTemperature(int16_t rawInput)
{
    return ((float)rawInput / 16.0f + 25.0f);
}

float calculateAcceleration(int16_t rawInput)
{
    float tmp = (float)rawInput;
    tmp *= LSM6DS3_FS_XL_SENSITIVITY[currentFSXL];
    return tmp;//(float)rawInput *  LSM6DS3_FS_XL_SENSITIVITY[currentFSXL];
}

float calculateRotation(int16_t rawInput)
{
    float tmp = (float)rawInput;
    tmp *= LSM6DS3_FS_G_SENSITIVITY[currentFSG];
    return tmp;// (float)rawInput * LSM6DS3_FS_G_SENSITIVITY[currentFSG];
}

status_t getRawMotionT(ImuRawData *imu)
{
	memset(IMUBuffer, 0, 22);
    LSM6_Read(LSM6DS3_OUT_TEMP_L, &IMUBuffer[0], 2);

    imu->T = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    getRawAcceleration(&imu->ax, &imu->ay, &imu->az);
  /*  imu->gx = ((((int16_t)IMUBuffer[3]) << 8) | IMUBuffer[2]);
    imu->gy = ((((int16_t)IMUBuffer[5]) << 8) | IMUBuffer[4]);
    imu->gz = ((((int16_t)IMUBuffer[7]) << 8) | IMUBuffer[6]);

    imu->ax = ((((int16_t)IMUBuffer[9]) << 8) | IMUBuffer[8]);
    imu->ay = ((((int16_t)IMUBuffer[11]) << 8) | IMUBuffer[10]);
    imu->az = ((((int16_t)IMUBuffer[13]) << 8) | IMUBuffer[12]);
*/
    return IMU_SUCCESS;
}

status_t getRawAcceleration(int16_t *ax, int16_t *ay, int16_t *az)
{
    LSM6_Read(LSM6DS3_OUTX_L_XL, &IMUBuffer[0], 6);
    *ax = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    *ay = ((((int16_t)IMUBuffer[3]) << 8) | IMUBuffer[2]);
    *az = ((((int16_t)IMUBuffer[5]) << 8) | IMUBuffer[4]);

    return IMU_SUCCESS;
}

status_t getRawRotation(int16_t *gx, int16_t *gy, int16_t *gz)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTX_L_G, 6, IMUBuffer);
    *gx = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    *gy = ((((int16_t)IMUBuffer[3]) << 8) | IMUBuffer[2]);
    *gz = ((((int16_t)IMUBuffer[5]) << 8) | IMUBuffer[4]);

    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawAccelerationX(int16_t *ax)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTX_L_XL, 2, IMUBuffer);
    *ax = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawAccelerationY(int16_t *ay)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTY_L_XL, 2, IMUBuffer);
    *ay = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawAccelerationZ(int16_t *az)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTZ_L_XL, 2, IMUBuffer);
    *az = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawTemperature(int16_t *t)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUT_TEMP_L, 2, IMUBuffer);
    *t = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawRotationX(int16_t *gx)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTX_L_G, 2, IMUBuffer);
    *gx = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawRotationY(int16_t *gy)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTY_L_G, 2, IMUBuffer);
    *gy = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawRotationZ(int16_t *gz)
{
    bool b = I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_OUTZ_L_G, 2, IMUBuffer);
    *gz = ((((int16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
    return b == true ? IMU_SUCCESS : IMU_ERROR;
}

status_t getRawPedometer(uint16_t *gz)
{
	 LSM6_Read(LSM6DS3_STEP_COUNTER_L, &IMUBuffer[0], 2);
	 *gz = ((((uint16_t)IMUBuffer[1]) << 8) | IMUBuffer[0]);
	 return IMU_SUCCESS;
}

status_t initGyro()
{

    currentFSG = LSM6DS3_FS_G_500dps;
    //LSM6DS3_CTRL3_C

  //  setGyroPerfMode(G_HM_MODE_DS);
    setGyroRange(currentFSG);
    setGyroODR(LSM6DS3_ODR_G_POWER_DOWN);
    return IMU_SUCCESS;
}
status_t initAccel()
{
    currentFSXL = LSM6DS3_FS_XL_4g;
    setAccelPerfMode(XL_HM_MODE_DS);
    setAccelRange(currentFSXL);

    setAccelBW(LSM6DS3_BW_XL_200Hz);
    setAccelODR(LSM6DS3_ODR_XL_208Hz);
    return IMU_SUCCESS;
}

status_t sleeptAccel()
{
    currentFSXL = LSM6DS3_FS_XL_4g;
    setAccelPerfMode(XL_HM_MODE_EN);
    setAccelRange(currentFSXL);

    setAccelBW(LSM6DS3_BW_XL_200Hz);
    setAccelODR(LSM6DS3_ODR_XL_POWER_DOWN);
    return IMU_SUCCESS;
}

void LSM6_GetPedometerData(void)
{
	TimingVars.timingEvtMin++;
	if(TimingVars.timingEvtMin>=60)
	{
		TimingVars.timingEvtMin=0;
		getRawPedometer(&BodyState.ped16);
		resetPed();
		BodyState.ped16+=9;
		if(BodyState.ped16>14)
		{

			PedArch.pedCount[PedArch.pedPtrTail]=BodyState.ped16;
			PedArch.pedTime[PedArch.pedPtrTail]=TimingVars.timingSec;
			PedArch.pedPtrTail++;
			BodyState.ped16=0;
		}
		if(PedArch.pedPtrTail==PED_ARCH_SIZE)
		{
			PedArch.pedPtrHead=PedArch.pedPtrTail;
			PedArch.pedPtrTail=0;
			PedArch.pedArchOver=1;
		}
	}

}
status_t resetPed(void)
{
	LSM6_Write(LSM6DS3_CTRL10_C, 0x00); // En pedometr function
	LSM6_Write(LSM6DS3_CTRL10_C, 0x06); // En pedometr function
	LSM6_Write(LSM6DS3_CTRL10_C, 0x04); // En pedometr function
	return IMU_SUCCESS;
}

status_t initPed()
{

	LSM6_Write(LSM6DS3_CTRL10_C, 0x00); // En pedometr function
	LSM6_Write(LSM6DS3_CTRL10_C, 0x06); // En pedometr function
	LSM6_Write(LSM6DS3_CTRL10_C, 0x04); // En pedometr function
//	I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_CTRL10_C, 0x3C); // En pedometr function
	LSM6_Write(LSM6DS3_TAP_CFG1, 0x40); // Enable pedometer algorithm

	LSM6_Write(LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_ACCESS_BIT_EN); // Enable embedded functions registers

	LSM6_Write(LSM6DS3_CONFIG_PEDO_THS_MIN, LSM6DS3_CONFIG_PEDO_THS_VALUE);


	LSM6_Write(LSM6DS3_SM_STEP_THS, LSM6DS3_SM_STEP_THS_VALUE);

	LSM6_Write(LSM6DS3_CONFIG_PEDO_DEB_MIN, LSM6DS3_CONFIG_PEDO_DEB_VALUE);

	LSM6_Write(LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_ACCESS_BIT_DS); // Enable embedded functions registers

//	I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_ACCESS_BIT_EN); // Enable embedded functions registers
	//Config pedometr emnedded function registers



//	I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_ACCESS_BIT_DS); // Enable embedded functions registers


}




status_t initInterrupts()
{
  //Activity/Inactivity recognition
 // I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_CTRL1_XL, 0x5A);    // Turn on the accelerometer, ODR_XL = 208 Hz, FS_XL = 2g
 I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_WAKE_UP_DUR, 0x00); // Set duration for Inactivity and for Activity detection
                                                            // SLEEP_DUR[3:0] : 1 LSB corresponds to 512*ODR_XL -- 0010b -> 4.92 s
// I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_INT1_CTRL, 0x40);                                                           // WAKE_DUR[1:0] : 1 LSB corresponds to ODR_XL -- 10b -> 9.62ms
 I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_WAKE_UP_THS, 0x00); // Set Activity/Inactivity threshold and detection
                                                            // WK_THS[5:0] : 1 LSB = (FS_XL)/(2^6) -- 000010b - > 62.5 mg

// I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_INT1_CTRL, 0x00);     // EN Acc_GyroData interrupt driven to INT1 pin
	I2CWriteByte(LSM6DS3_ADDRESS,LSM6DS3_FREE_FALL, 0x00);  //Enable Free fall
	I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_MD1_CFG, 0x00); // Enable pedometer algorithm
	I2CWriteByte(LSM6DS3_ADDRESS, LSM6DS3_INT1_CTRL, 0x00);     // EN Acc_GyroData interrupt driven to INT1 pin
}

//{wake_up,tap,d6d,func}
void getIMUInterruptSource(uint8_t* interruptSrc)
{
  I2CReadBytes(LSM6DS3_ADDRESS, LSM6DS3_WAKE_UP_SRC, 3, interruptSrc);
  I2CReadByte(LSM6DS3_ADDRESS, LSM6DS3_FUNC_SRC, &interruptSrc[3]);
}
// get int source, read WAKE_UP_SRC, D6D_SRC, TAP_SRC and FUNC_SRC.

status_t setAccelRange(LSM6DS3_FS_XL_t range)
{
    uint8_t tmpReg = 0;

    LSM6_Read(LSM6DS3_CTRL1_XL, &tmpReg, 1);
    tmpReg |= range<<2;
    LSM6_Write(LSM6DS3_CTRL1_XL, tmpReg);

    return IMU_SUCCESS;
}

status_t setAccelPerfMode(LSM6DS3_APF_t bw)
{
    uint8_t tmpReg = 0;

    LSM6_Read(LSM6DS3_CTRL6_C, &tmpReg, 1);
    tmpReg |= bw;
    LSM6_Write(LSM6DS3_CTRL6_C, tmpReg);

    return IMU_SUCCESS;
}

status_t setGyroPerfMode(LSM6DS3_GPF_t bw)
{
	 uint8_t tmpReg = 0;
	 LSM6_Read(LSM6DS3_CTRL7_G, &tmpReg, 1);

	 tmpReg |= bw;
	 LSM6_Write(LSM6DS3_CTRL7_G, tmpReg);

	 return IMU_SUCCESS;
}

status_t setAccelBW(LSM6DS3_BW_XL_t bw)
{
	uint8_t tmpReg = 0;
	LSM6_Read(LSM6DS3_CTRL1_XL, &tmpReg, 1);

	tmpReg |= bw;
	LSM6_Write(LSM6DS3_CTRL1_XL, tmpReg);

	return IMU_SUCCESS;

}

status_t setAccelODR(LSM6DS3_ODR_XL_t rate)
{
	uint8_t tmpReg = 0;
	LSM6_Read(LSM6DS3_CTRL1_XL, &tmpReg, 1);

	tmpReg |= rate;
	LSM6_Write(LSM6DS3_CTRL1_XL, tmpReg);
	LSM6_Read(LSM6DS3_CTRL1_XL, &tmpReg, 1);
	return IMU_SUCCESS;
}

status_t setGyroODR(LSM6DS3_ODR_G_t rate)
{
	uint8_t tmpReg = 0;
	LSM6_Read(LSM6DS3_CTRL2_G, &tmpReg, 1);

	tmpReg |= rate;
	LSM6_Write(LSM6DS3_CTRL2_G, tmpReg);

	return IMU_SUCCESS;

}

status_t setGyroRange(LSM6DS3_FS_G_t range)
{
	uint8_t tmpReg = 0;
	LSM6_Read(LSM6DS3_CTRL2_G, &tmpReg, 1);

	tmpReg |= range;
	LSM6_Write(LSM6DS3_CTRL2_G, tmpReg);
	LSM6_Read(LSM6DS3_CTRL2_G, &tmpReg, 1);
	return IMU_SUCCESS;

}

uint8_t LSM6_BodyStateDetection(void)
{
	LSM6_AnalyseCardiomoPos();
	if(StateFlags.cardomoDressedPosture==DOWN_POS)
	{
		if (RawData.ax > 4918) {
			return VERT_S;
		}
		else if (RawData.ay < 0) {
	       if (RawData.az < 0) {
	           return  GOR_BACK_S;
	       }
	       else {
	           return GOR_LEFT_S;
	       }
	   }
	   else if (RawData.az > 0) {
	       return GOR_STOM_S;
	   }
	   else {
	      return GOR_RIGHT_S;
	   }
	}
	else if(StateFlags.cardomoDressedPosture==UP_POS)
	{
		float k1 = 1.4;
		float k2 = -0.714285714285714;

		   if (RawData.ax < -5740) {
		       return  VERT_S;
		   }
		   else if (RawData.ay > (int16_t)(k1 * (float)RawData.az)) {
		       if (RawData.ay < (int16_t)(k2 * (float)RawData.az)) {
		           return GOR_BACK_S;
		       }
		       else  {
		           return GOR_LEFT_S;
		       }
		   }
		   else if (RawData.ay > (int16_t)(k2 * (float)RawData.az)) {
		       return GOR_STOM_S;
		   }
		   else {
		       return GOR_RIGHT_S;
		   }
	}
	else
	{
		//fixME undefined dressed posture state
	}


}


void LSM6_AnalyseCardiomoPos(void)
{
	if(RawData.ax<=0)
	{
		StateFlags.cardomoDressedPosture=UP_POS;
		return;
	}
	else if(RawData.ax>0)
	{
		StateFlags.cardomoDressedPosture=DOWN_POS;
		return;
	}
	else
	{
		StateFlags.cardomoDressedPosture=UNDEF_POS;
		return;
	}
}


void LSM6_CalcAngleAc(void)
{
	volatile double tmpSqrtf=0;
	volatile double tmpVar1=0;
	volatile double tmpVar2=0;
	volatile uint64_t raxf=0;
	volatile uint64_t rayf=0;
	volatile uint64_t razf=0;
	uint64_t angArr[3]={0};


	getRawMotionT(&RawData);

	raxf=(uint64_t)RawData.ax;
	rayf=(uint64_t)RawData.ay;
	razf=(uint64_t)RawData.az;

	g_AngleAc.mang=raxf*raxf+rayf*rayf+razf*razf;

	//g_AngleAc.mang=(float)sqrt(tmpVar1);
	//BodyState.postureFlag=1;


	if(g_AngleAc.mang>=L_STAY_LIM && g_AngleAc.mang<=H_STAY_LIM) { 	BodyState.activity=NOT_ACT; }
	else
	{
		BodyState.activity=ACT; BodyState.postureFlag=0; BodyState.posture=BODY_IN_ACT;
	}

	if(BodyState.postureFlag==1)
	{
		  BodyState.posture=LSM6_BodyStateDetection();
	}


		BodyState.postureFlag=0;
		if(StateFlags.fallDtcTmElapsed==1)
			{
			//	if(AngleGyro.gyroLimFall==1)
			//	{
				//	if(BodyState.posture!=0) BodyState.fall=1;
			//	}
			//	else BodyState.fall=0;

			//	g_AngleAc.LimFlagL=0;
			//	g_AngleAc.LimFlagH=0;
				StateFlags.fallDtcTmElapsed=0;
			}


	if(g_AngleAc.mang<FALL_LIM_L && g_AngleAc.LimFlagL==0)
	{
		g_AngleAc.LimFlagL=1;
		g_AngleAc.LimFlagH=0;

	}


	if(g_AngleAc.LimFlagL==1 && g_AngleAc.mang>FALL_LIM_H)
	{
		g_AngleAc.LimFlagH=1;
		g_AngleAc.LimFlagL=0;
	}


}


