/*
 * MPU9250.h
 *
 *  Created on: 24-Dec-2024
 *      Author: HP
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

/*************************************MPU9250 MACROS**************************************/

#define MPU9250_ADDRESS		(0x68<<1)
#define SELF_TEST_X_GYRO	0x00
#define SELF_TEST_Y_GYRO	0x01
#define SELF_TEST_Z_GYRO	0x02
#define SELF_TEST_X_ACCEL	0x0D
#define SELF_TEST_Y_ACCEL	0x0E
#define SELF_TEST_Z_ACCEL	0x0F

#define XG_OFFSET_H			0x13
#define XG_OFFSET_L			0x14
#define YG_OFFSET_H			0x15
#define YG_OFFSET_L			0x16
#define ZG_OFFSET_H			0x17
#define ZG_OFFSET_L			0x18

#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYR_CNFG_REG		0x1B
#define ACC_CNFG_REG		0x1C
#define ACCEL_CONFIG_2		0x1D
#define	LP_ACCEL_ODR		0x1E
#define	WOM_THR				0x1F
#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24

#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG         0x37      // INT pin configuration for bypass mode
#define INT_ENABLE       	0x38
#define INT_STATUS 			0x3A
#define ACCEL_XOUT_H_REG	0x3B	  // ACCELEROMETER DATA REG's
#define TEMP_OUT_H			0x41	  // TEMP SENSOR DATA REG's
#define TEMP_OUT_L			0x42
#define GYR_XOUT_H_REG		0x43	  // GYROSCOPE DATA REG's
#define EXT_SENS_DATA_00	0x49	  // 0x49 - 0x60 Ext-sens(0-23)

#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A

#define PWR_MNGMT_1_REG		0x6B
#define PWR_MGMT_2_REG		0x6C

#define FIFO_COUNTH			0x72
#define FIFO_COUNTL			0x73
#define FIFO_R_W			0x74

#define WHO_AM_I_REG		0x75

#define XA_OFFSET_H			0x77
#define XA_OFFSET_L			0x78
#define YA_OFFSET_H			0x7A
#define YA_OFFSET_L			0x7B
#define ZA_OFFSET_H			0x7D
#define ZA_OFFSET_L			0x7E

/*************************************MAGNETOMETER AKM8963**************************************/

/*Read-Only Registers*/
#define AKM8963_ADDRESS		(0x0C<<1)	//Device I2C Address
#define MAG_WIA_REG			0x00		//Device ID
#define MAG_INFO_REG		0x01		//Information
#define MAG_ST1_REG			0x02		//Status 1
#define MAG_HXL_REG			0x03		//Measurement data
#define MAG_ST2_REG			0x09		//Status 2

/* READ/WRITE */
#define MAG_CNTL1_REG		0x0A		//Control
#define MAG_CNTL2_REG		0x0B		//Reserved
#define MAG_ASTC_REG		0x0C		//Self-test
#define MAG_TS1_REG			0x0D		//Test 1 (DO NOT USE)
#define MAG_TS2_REG			0x0E		//Test 2 (DO NOT USE)
#define MAG_I2CIDS_REG		0x0F		//I2C disable

/*Read-Only Registers*/
#define MAG_ASAX_REG		0x10		//X-axis sensitivity adjustment value
#define MAG_ASAY_REG		0x11		//Y-axis sensitivity adjustment value
#define MAG_ASAZ_REG		0x12		//Z-axis sensitivity adjustment value

/*Structures*/
typedef struct
{
	uint8_t Read_Data;
	uint8_t Write_Data;
	char Tx_Buff[180];
}IMU_HandleTypeDef;

typedef struct
{
	int16_t x;
	float X;
	int16_t y;
	float Y;
	int16_t z;
	float Z;
	uint8_t Read_Data[6];
	uint8_t Write_Data;
}Gyro_HandleTypeDef;

typedef struct
{
	int16_t x;
	float X;
	int16_t y;
	float Y;
	int16_t z;
	float Z;
	uint8_t Read_Data[6];
	uint8_t Write_Data;
}Accel_HandleTypeDef;

typedef struct
{
	int16_t x;
	float X;
	int16_t y;
	float Y;
	int16_t z;
	float Z;
	uint8_t Read_Data[6];
	uint8_t Write_Data;
}Magneto_HandleTypeDef;

/*Function Declaration's*/
void MPU9250_Init(void);
void AK8963_Init(void);

void MPU9250_Read_Accel(void);
void MPU9250_Read_Gyro(void);
void MPU9250_Read_Magnet(void);

void MPU9250_Reset(void);
void AK8963_Reset(void);

#endif /* INC_MPU9250_H_ */
