/*
 * ICM20948.h
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

#ifndef ICM20948_H_
#define ICM20948_H_

#include "stm32f4xx_hal.h"
#include "main.h"

// NEEDED FOR ALL ////////////////////////////////////////////////////////////////////////////////////////
#define ICM_CS_Pin 				(GPIO_PIN_0)
#define ICM_CS_GPIO_Port 		(GPIOB)
#define SPI_BUS					(&hspi1)
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// VARIAVEIS PARA O (1) //////////////////////////////////////////////////////////////////////////////////
// USER BANK 0 REGISTER MAP
#define WHO_AM_I_ICM20948  		(0x00) // Should return 0xEA
#define USER_CTRL          		(0x03)  // Bit 7 enable DMP, bit 3 reset DMP
#define LP_CONFIG		   		(0x05) // Not found in MPU-9250
#define PWR_MGMT_1         		(0x06) // Device defaults to the SLEEP mode
#define PWR_MGMT_2         		(0x07)
#define INT_PIN_CFG        		(0x0F)
#define INT_ENABLE         		(0x10)
#define INT_ENABLE_1	   		(0x11) // Not found in MPU-9250
#define INT_ENABLE_2	   		(0x12) // Not found in MPU-9250
#define INT_ENABLE_3	   		(0x13) // Not found in MPU-9250
#define I2C_MST_STATUS     		(0x17)
#define INT_STATUS         		(0x19)
#define INT_STATUS_1	   		(0x1A) // Not found in MPU-9250
#define INT_STATUS_2	   		(0x1B) // Not found in MPU-9250
#define INT_STATUS_3	   		(0x1C) // Not found in MPU-9250
#define DELAY_TIMEH		   		(0x28)	// Not found in MPU-9250
#define DELAY_TIMEL		   		(0x29)	// Not found in MPU-9250
#define ACCEL_XOUT_H       		(0x2D)
#define ACCEL_XOUT_L       		(0x2E)
#define ACCEL_YOUT_H       		(0x2F)
#define ACCEL_YOUT_L       		(0x30)
#define ACCEL_ZOUT_H      		(0x31)
#define ACCEL_ZOUT_L      		(0x32)
#define GYRO_XOUT_H        		(0x33)
#define GYRO_XOUT_L        		(0x34)
#define GYRO_YOUT_H        		(0x35)
#define GYRO_YOUT_L        		(0x36)
#define GYRO_ZOUT_H        		(0x37)
#define GYRO_ZOUT_L        		(0x38)
#define TEMP_OUT_H         		(0x39)
#define TEMP_OUT_L         		(0x3A)
#define EXT_SENS_DATA_00   		(0x3B)
#define EXT_SENS_DATA_01   		(0x3C)
#define EXT_SENS_DATA_02   		(0x3D)
#define EXT_SENS_DATA_03   		(0x3E)
#define EXT_SENS_DATA_04   		(0x3F)
#define EXT_SENS_DATA_05   		(0x40)
#define EXT_SENS_DATA_06   		(0x41)
#define EXT_SENS_DATA_07   		(0x42)
#define EXT_SENS_DATA_08   		(0x43)
#define EXT_SENS_DATA_09   		(0x44)
#define EXT_SENS_DATA_10   		(0x45)
#define EXT_SENS_DATA_11   		(0x46)
#define EXT_SENS_DATA_12   		(0x47)
#define EXT_SENS_DATA_13   		(0x48)
#define EXT_SENS_DATA_14   		(0x49)
#define EXT_SENS_DATA_15   		(0x4A)
#define EXT_SENS_DATA_16   		(0x4B)
#define EXT_SENS_DATA_17   		(0x4C)
#define EXT_SENS_DATA_18   		(0x4D)
#define EXT_SENS_DATA_19   		(0x4E)
#define EXT_SENS_DATA_20   		(0x4F)
#define EXT_SENS_DATA_21   		(0x50)
#define EXT_SENS_DATA_22   		(0x51)
#define EXT_SENS_DATA_23   		(0x52)
#define FIFO_EN_1          		(0x66)
#define FIFO_EN_2          		(0x67) // Not found in MPU-9250
#define FIFO_RST		   		(0x68) // Not found in MPU-9250
#define FIFO_MODE		   		(0x69) // Not found in MPU-9250
#define FIFO_COUNTH        		(0x70)
#define FIFO_COUNTL        		(0x71)
#define FIFO_R_W           		(0x72)
#define DATA_RDY_STATUS	   		(0x74) // Not found in MPU-9250
#define FIFO_CFG		   		(0x76) // Not found in MPU-9250
#define REG_BANK_SEL	   		(0x7F) // Not found in MPU-9250

// USER BANK 1 REGISTER MAP
#define SELF_TEST_X_GYRO  		(0x02)
#define SELF_TEST_Y_GYRO  		(0x03)
#define SELF_TEST_Z_GYRO  		(0x04)
#define SELF_TEST_X_ACCEL 		(0x0E)
#define SELF_TEST_Y_ACCEL 		(0x0F)
#define SELF_TEST_Z_ACCEL 		(0x10)
#define XA_OFFSET_H       		(0x14)
#define XA_OFFSET_L       		(0x15)
#define YA_OFFSET_H       		(0x17)
#define YA_OFFSET_L       		(0x18)
#define ZA_OFFSET_H       		(0x1A)
#define ZA_OFFSET_L       		(0x1B)
#define TIMEBASE_CORRECTION_PLL	(0x28)

// USER BANK 2 REGISTER MAP
#define GYRO_SMPLRT_DIV        	(0x00) // Not found in MPU-9250
#define GYRO_CONFIG_1      		(0x01) // Not found in MPU-9250
#define GYRO_CONFIG_2      		(0x02) // Not found in MPU-9250
#define XG_OFFSET_H       		(0x03) // User-defined trim values for gyroscope
#define XG_OFFSET_L       		(0x04)
#define YG_OFFSET_H       		(0x05)
#define YG_OFFSET_L       		(0x06)
#define ZG_OFFSET_H       		(0x07)
#define ZG_OFFSET_L       		(0x08)
#define ODR_ALIGN_EN			(0x09) // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_1     	(0x10) // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_2     	(0x11) // Not found in MPU-9250
#define ACCEL_INTEL_CTRL		(0x12) // Not found in MPU-9250
#define ACCEL_WOM_THR			(0x13) // Not found in MPU-9250 (could be WOM_THR)
#define ACCEL_CONFIG_1      	(0x14)
#define ACCEL_CONFIG_2     		(0x15) // Not found in MPU-9250 (could be ACCEL_CONFIG2)
#define FSYNC_CONFIG			(0x52) // Not found in MPU-9250
#define TEMP_CONFIG				(0x53) // Not found in MPU-9250
#define MOD_CTRL_USR			(0x54) // Not found in MPU-9250

//Magnetometer Registers
#define AK09916_ADDRESS  0x0C
#define WHO_AM_I_AK09916 0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     0x30  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    0x31  // Normal (0), Reset (1)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// VARIAVEIS PARA O (2) /////////////////////////////////////////////////////////////////////////////////////
uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];

#define USER_BANK_SEL		(0x7F)
#define USER_BANK_0			(0x00)
#define USER_BANK_1			(0x10)
#define USER_BANK_2			(0x20)
#define USER_BANK_3			(0x30)

#define PWR_MGMT_1 			(0x06)
#define PWR_MGMT_2			(0x07)
#define GYRO_CONFIG_1		(0x01)


#define CLK_BEST_AVAIL		(0x01)
#define GYRO_RATE_250		(0x00)
#define GYRO_LPF_17HZ 		(0x29)


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DESCRICAO DAS FUNCOES BASICAS DE ABTSRACAO SPI E I2C /////////////////////////////////////////////////////
void ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size);
void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size);
void ICM_ReadOneByte(uint8_t reg, uint8_t* pData);
void ICM_WriteOneByte(uint8_t reg, uint8_t Data);
void i2c_Mag_write(uint8_t reg,uint8_t value);
static uint8_t ICM_Mag_Read(uint8_t reg);
void ICM_ReadMagData(int16_t heading[3]);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCOES USADAS PRINCIPALMENTE PARA (2) MAS REAPROVEITADAS ALGUMAS PARA O (1) /////////////////////////////
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
uint8_t ICM_WHOAMI(void);
void ICM_AccelGyroOn(void);
void ICM_AccelGyroOff(void);
void ICM_SetClock(uint8_t clk);
void ICM_CSLow(void);
void ICM_CSHigh(void);
void ICM_Disable_I2C(void);
void ICM_SelectBank(uint8_t bank);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCOES USADAS PARA SO O (2) /////////////////////////////////////////////////////////////////////////////
void ICM_ReadAccelGyroData(void);
void ICM_ReadMagData(int16_t heading[3]);
uint16_t ICM_Initialize(void);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCOES SO USADAS PARA O (1) /////////////////////////////////////////////////////////////////////////////
void ICM_SelfTest(float * destination);
void ICM_Calibrate(float * gyroBias, float * accelBias);
void ICM_Magnetometer_Init();
void ICM_Magnetometer_Calibrate(float * bias_dest, float * scale_dest);
void readMagData(int16_t * destination);
void ICM_Init();
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCAO COMUM /////////////////////////////////////////////////////////////////////////////////////////////
void ICM_PowerOn();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* ICM20948_H_ */
