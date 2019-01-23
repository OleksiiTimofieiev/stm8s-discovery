
#ifndef __LSM6DS3_H
#define __LSM6DS3_H

#define LSM6DS3_DEVICE_ID			0x69

#define LSM6DS3_ACC_GYRO_CTRL3_C          0X12
#define LSM6DS3_ACC_GYRO_IF_INC_ENABLED        0x04
#define LSM6DS3_ACC_GYRO_IF_INC_MASK 0x04
#define LSM6DS3_ACC_GYRO_BDU_MASK   0x40
#define LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE        0x40

/* FIFO mode -> currently disabled */
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5          0X0A
#define LSM6DS3_ACC_GYRO_FIFO_MODE_MASK   0x07
#define LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS        0x00

/* power section */
#define LSM6DS3_ACC_GYRO_CTRL1_XL          0X10
#define LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN        0x00
#define LSM6DS3_ACC_GYRO_ODR_XL_MASK        0xF0

/* g scale selection */
#define LSM6DS3_ACC_GYRO_FS_XL_MASK        0x0C
#define LSM6DS3_ACC_GYRO_FS_XL_2g        0x00

/* x,y,z section */
#define LSM6DS3_ACC_GYRO_CTRL9_XL          0X18
#define        LSM6DS3_ACC_GYRO_XEN_XL_MASK        0x08
#define        LSM6DS3_ACC_GYRO_YEN_XL_MASK        0x10
#define        LSM6DS3_ACC_GYRO_ZEN_XL_MASK        0x20
#define        LSM6DS3_ACC_GYRO_XEN_XL_ENABLED        0x08
#define        LSM6DS3_ACC_GYRO_YEN_XL_ENABLED        0x10
#define        LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED        0x20


#endif