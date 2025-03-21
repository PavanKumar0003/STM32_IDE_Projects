/*
 * MPU9250.c
 *
 *  Created on: 24-Dec-2024
 *  Author: Pavan Kumar
 *
 *  Q. Why BYPASSING?
 *  A. When bypass mode is enabled in the MPU9250 (BYPASS_EN = 1 in the INT_PIN_CFG register), the magnetometer (AK8963) is directly accessible
 *     from the host microcontroller via the auxiliary I2C bus. This allows you to communicate with the magnetometer as if it were connected directly
 *     to your MCU, bypassing the MPU9250's internal processing.
 *     In non-bypass mode, the MPU9250 manages the communication with the magnetometer internally. Magnetometer data is fetched by the MPU9250 and
 *     made available in its external sensor data registers (EXT_SENS_DATA). The host reads magnetometer data indirectly through these registers,
 *     requiring configuration of the MPU9250's I2C master settings.
 */

/*
 * MPU9250_init(): Initializes the Mpu9250 sensors (ACC+GYRO)
 * 1. Check for Dev ID matches(Default ID is 0x71)
 * 2. if Dev-ID matches then initialize the registers
 * 3. PWR_MNGMT_1_REG to Reset the internal registers and restores the default settings.  Write a 1 to
		set the reset, the bit will auto clear.
   4. ACC_CNFG_REG to config Fullscale range.(2g,4g,8g,16g)
   5. GYR_CNFG_REG to config Fullscale range(250dps,500dps,1000dps,2000dps)
   6. INT_PIN_CFG - INT Pin / Bypass Enable Configuration to change i2c_master interface pins to bypass mode
 */
#include "main.h"
#include <MPU9250.h>

/*Use structure Objects which are declared in main.c file here in MPU9250.c*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern Gyro_HandleTypeDef		Gyr;
extern Accel_HandleTypeDef		Acc;
extern Magneto_HandleTypeDef	Mag;
extern IMU_HandleTypeDef		MPU9250;

void MPU9250_Init(/*<Acc_full_scale_range>,<Gyro_full_scale_range>,<Bypass mode>,*/)
{
	HAL_I2C_Mem_Read (&hi2c1, MPU9250_ADDRESS, WHO_AM_I_REG, 1, &MPU9250.Read_Data, 1, 1000);		 //Know the Device ID

	if(MPU9250.Read_Data == 0x71)
	{
		MPU9250.Write_Data = 0x80;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MNGMT_1_REG, 1, &MPU9250.Write_Data, 1, 1000); //RESET Internal registers

		/*ACC cofiguration with +/- 2g*/
		Acc.Write_Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACC_CNFG_REG, 1, &Acc.Write_Data, 1, 1000);

		/*GYR cofiguration with +/- 250 deg/sec */
		Gyr.Write_Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYR_CNFG_REG, 1, &Gyr.Write_Data, 1, 1000);

		/*Bypass mode to read MAG data*/
		MPU9250.Write_Data = 0x22;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, &MPU9250.Write_Data, 1, 1000);
		MPU9250.Write_Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, &MPU9250.Write_Data, 1, 1000);	//RAW_RDY_EN
	}
}

void MPU9250_Read_Accel(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H_REG, 1, Acc.Read_Data, 6, 1000);
	Acc.x = (int16_t)(Acc.Read_Data[0] << 8 | Acc.Read_Data[1]);
	Acc.y = (int16_t)(Acc.Read_Data[2] << 8 | Acc.Read_Data[3]);
	Acc.z = (int16_t)(Acc.Read_Data[4] << 8 | Acc.Read_Data[5]);
	Acc.X = Acc.x / 16384.0;
	Acc.Y = Acc.x / 16384.0;
	Acc.Z = Acc.x / 16384.0;
}

void MPU9250_Read_Gyro(void)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYR_XOUT_H_REG, 1, Gyr.Read_Data, 6, 1000);
	Gyr.x = (int16_t)(Gyr.Read_Data[0] << 8 | Gyr.Read_Data[1]);
	Gyr.y = (int16_t)(Gyr.Read_Data[2] << 8 | Gyr.Read_Data[3]);
	Gyr.z = (int16_t)(Gyr.Read_Data[4] << 8 | Gyr.Read_Data[5]);
	Gyr.X = Gyr.x / 131.0;
	Gyr.Y = Gyr.y / 131.0;
	Gyr.Z = Gyr.z / 131.0;
}

void MPU9250_Reset() //as argument <Reset level>
{
	/*
	 *1.Reset signal paths using <SIGNAL_PATH_RESET>
	 *2.Hard Reset using <PWR_MGMT_1> which resets gyro and Acc
	 */
}

void AK8963_Init() //<measurement mode>
{
	HAL_I2C_Mem_Read(&hi2c1, AKM8963_ADDRESS, MAG_WIA_REG, 1, &Mag.Read_Data[0], 1, 1000);	//Dev ID
	if(Mag.Read_Data[0]== 0x48)
	{
		/*MAG configuration*/
		Mag.Write_Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, AKM8963_ADDRESS, MAG_CNTL1_REG, 1, &Mag.Write_Data, 1, 1000);	//PowerDown magnetometer
		HAL_Delay(50);
		Mag.Write_Data = 0x16;
		HAL_I2C_Mem_Write(&hi2c1, AKM8963_ADDRESS, MAG_CNTL1_REG, 1, &Mag.Write_Data, 1, 1000);	//continuous measurement mode1(8Hz)
	}
}


void MPU9250_Read_Magnet(void)
{
	//Mag.Write_Data = 0x16;
	HAL_I2C_Mem_Write(&hi2c1, AKM8963_ADDRESS, MAG_CNTL1_REG, 1, &Mag.Write_Data, 1, 1000);	//continuous measurement mode
	HAL_Delay(50);	//maintain a delay of 50ms to update the measurement mode changes in the registers
	HAL_I2C_Mem_Read(&hi2c1, AKM8963_ADDRESS, MAG_HXL_REG, 1, (uint8_t *)&(Mag.Read_Data), 6, 1000);
	Mag.x = (int16_t)(Mag.Read_Data[1] << 8 | Mag.Read_Data[0]);
	Mag.y = (int16_t)(Mag.Read_Data[3] << 8 | Mag.Read_Data[2]);
	Mag.z = (int16_t)(Mag.Read_Data[5] << 8 | Mag.Read_Data[4]);
	Mag.X = Mag.x / 0.6;
	Mag.Y = Mag.y / 0.6;
	Mag.Z = Mag.z / 0.6;
}

void AK8963_Reset(void)
{
	Mag.Write_Data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, AKM8963_ADDRESS, MAG_ASTC_REG, 1, (uint8_t *)&Mag.Write_Data, 1, 1000);	//Reset Magnetometer on <CONTRL-2> register SFTRST
}
