/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

// *** Three asterisks to the side of a line means this may change based on platform
#include "ICM20948.h"
#include <string.h>
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"

//#define DRCPATTISON 1
#define CORYCLINE 2

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCOES BASICAS DE ESCRITA E LEITURA POR SPI, TAMBEM A INTERFACE PARA USAR SPI PARA O MAGNETOMETRO EM I2C/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// LEITRYA DE VARIOS BYTES EM REGISTOS SEGUIDOS, SPI
	void ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
	{
		reg = reg | 0x80;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
		HAL_SPI_Receive_DMA(SPI_BUS, pData, Size);
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
	}

	// ESCRITA DE VARIOS BYTES EM REGISTOS SEGUIDOS, SPI
	void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
	{
		reg = reg & 0x7F;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
		HAL_SPI_Transmit_DMA(SPI_BUS, pData, Size);
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);

	}

	// LEITURA DE UM BYTE DOS REGISTOS DO ICM, SPI
	void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
	{
		reg = reg | 0x80;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
		while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
			;
		HAL_SPI_Receive_DMA(SPI_BUS, pData, 1);
		while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
			;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
	}

	// ESCRITA DE UM BYTE NOS REGISTOS DO ICM, SPI
	void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
	{
		reg = reg & 0x7F;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
		while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
			;
		HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
		while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
			;
		HAL_SPI_Transmit_DMA(SPI_BUS, &Data, 1);
		while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
			;
		HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
	}

	// ESCRITRA EM SO UM REGISTO DO SENSOR AUXILIAR MAGNETOMERO, ABSTRACAO DE I2C PARA SPI
	void i2c_Mag_write(uint8_t reg,uint8_t value)
	{
		ICM_WriteOneByte(0x7F, 0x30);

		HAL_Delay(1);
		ICM_WriteOneByte(0x03 ,0x0C);//mode: write

		HAL_Delay(1);
		ICM_WriteOneByte(0x04 ,reg);//set reg addr

		HAL_Delay(1);
		ICM_WriteOneByte(0x06 ,value);//send value

		HAL_Delay(1);
	}

	// LEITURA DE UM SO REGISTO DO SENSOR AUXILIAR MAGNETOMERO, ABSTRACAO DE I2C PARA SPI
	static uint8_t ICM_Mag_Read(uint8_t reg)
	{
		uint8_t  Data;
		ICM_WriteOneByte(0x7F, 0x30);
		HAL_Delay(1);
		ICM_WriteOneByte(0x03 ,0x0C|0x80);
		HAL_Delay(1);
		ICM_WriteOneByte(0x04 ,reg);// set reg addr
		HAL_Delay(1);
		ICM_WriteOneByte(0x06 ,0xff);//read
		HAL_Delay(1);
		ICM_WriteOneByte(0x7F, 0x00);
		ICM_ReadOneByte(0x3B,&Data);
		HAL_Delay(1);
		return Data;
	}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCAO PRINCIPAL COMUM A AMBOS ///////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ICM_PowerOn(void) {

	//RESET THE ICM
	ICM_SelectBank(USER_BANK_0);
	ICM_WriteOneByte(PWR_MGMT_1 , 0x80);
	HAL_Delay(10);
	ICM_SelectBank(USER_BANK_0);
	HAL_Delay(10);
	uint8_t whoami = 0xEA;
	uint8_t test = ICM_WHOAMI();

	if (test == whoami) {
/*
#if defined(DRCPATTISON)

		float selfTest[6];
		float gyroBias[3]  = {0, 0, 0}, accelBias[3] = {0, 0, 0},
		magBias[3]   = {0, 0, 0}, magScale[3]  = {0, 0, 0};

		ICM_SelfTest(selfTest);

		printf("x-axis self test: acceleration trim within : %f %% of factory value",selfTest[0]);

		printf("y-axis self test: acceleration trim within : %f %% of factory value",selfTest[1]);

		printf("z-axis self test: acceleration trim within : %f %% of factory value",selfTest[2]);

		printf("x-axis self test: gyration trim within : %f %% of factory value",selfTest[3]);

		printf("y-axis self test: gyration trim within : %f %% of factory value",selfTest[4]);

		printf("z-axis self test: gyration trim within : %f %% of factory value",selfTest[5]);

		fflush(stdout);

		ICM_SelectBank(USER_BANK_0);
		ICM_WriteOneByte(PWR_MGMT_1 , RESET);

		ICM_Calibrate(gyroBias,accelBias);

		ICM_Init();

		test = ICM_Mag_Read(WHO_AM_I_AK09916);

		if(test != 0x09){
			Error_Mag_Init=1;
		}
		else{

			//myIMU.getAres();
			//myIMU.getGres();
			//myIMU.getMres();
			  //DOTI

			 /*
			ICM_Magnetometer_Init();
			ICM_Magnetometer_Calibrate(magBias,magScale);
		}
#elif defined(CORYCLINE)*/

		ICM_CSHigh();
		HAL_Delay(10);
		ICM_SelectBank(USER_BANK_0);
		HAL_Delay(10);
		ICM_Disable_I2C();
		HAL_Delay(10);
		ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
		HAL_Delay(10);
		ICM_AccelGyroOff();
		HAL_Delay(20);
		ICM_AccelGyroOn();
		HAL_Delay(10);
		ICM_Initialize();
		HAL_Delay(10);
//#endif
	} else {
		printf("Failed WHO_AM_I.  %i is not 0xEA\r\n", test);
		HAL_Delay(100);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  FUNCOES DE FUNCIONAMENTO PARA O (2)  ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// INICIALIZACAO DO ICM/MAG PARA O (2) NAO FAZ CALIBRACAO, SO SET OS RANGES E SPECS
	uint16_t ICM_Initialize(void) {
		ICM_SelectBank(USER_BANK_2);
		HAL_Delay(20);
		ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
		HAL_Delay(10);

		// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
		ICM_WriteOneByte(0x00, 0x0A);
		HAL_Delay(10);

		// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
		ICM_WriteOneByte(0x14, (0x04 | 0x11));

		// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
		ICM_WriteOneByte(0x10, 0x00);
		HAL_Delay(10);

		// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
		ICM_WriteOneByte(0x11, 0x0A);
		HAL_Delay(10);

		ICM_SelectBank(USER_BANK_2);
		HAL_Delay(20);

		// Configure AUX_I2C Magnetometer (onboard ICM-20948)
		ICM_WriteOneByte(0x7F, 0x00); // Select user bank 0
		ICM_WriteOneByte(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
		ICM_WriteOneByte(0x03, 0x20); // I2C_MST_EN
		ICM_WriteOneByte(0x7F, 0x30); // Select user bank 3
		ICM_WriteOneByte(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
		ICM_WriteOneByte(0x02, 0x01); // I2C_SLV0 _DLY_ enable
		ICM_WriteOneByte(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

		// Initialize magnetometer
		i2c_Mag_write(0x32, 0x01); // Reset AK8963
		HAL_Delay(1000);
		i2c_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

		return 1337;
	}

	// LEITURA DOS DADOS DO ACELEROMETRO E GIROSCOPICO
	void ICM_ReadAccelGyroData(void) {
		uint8_t raw_data[12];
		ICM_readBytes(0x2D, raw_data, 12);

		accel_data[0] = (raw_data[0] << 8) | raw_data[1];
		accel_data[1] = (raw_data[2] << 8) | raw_data[3];
		accel_data[2] = (raw_data[4] << 8) | raw_data[5];

		gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
		gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
		gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

		accel_data[0] = accel_data[0] / 8;
		accel_data[1] = accel_data[1] / 8;
		accel_data[2] = accel_data[2] / 8;

		gyro_data[0] = gyro_data[0] / 250;
		gyro_data[1] = gyro_data[1] / 250;
		gyro_data[2] = gyro_data[2] / 250;
	}

	// LEITURA DE OS DADOS DO MAGNETOMERO, ABSTRACAO DE I2C PARA SPI
	void ICM_ReadMagData(int16_t heading[3])
	{
		uint8_t mag_buffer[10];
		mag_buffer[0] = ICM_Mag_Read(0x01);
		mag_buffer[1] = ICM_Mag_Read(0x11);
		mag_buffer[2] = ICM_Mag_Read(0x12);
		heading[0] = mag_buffer[1] | mag_buffer[2] << 8;
		mag_buffer[3] = ICM_Mag_Read(0x13);
		mag_buffer[4] = ICM_Mag_Read(0x14);
		heading[1] = mag_buffer[3] | mag_buffer[4] << 8;
		mag_buffer[5] = ICM_Mag_Read(0x15);
		mag_buffer[6] = ICM_Mag_Read(0x16);
		heading[2] = mag_buffer[5] | mag_buffer[6] << 8;
		i2c_Mag_write(0x31, 0x01);
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// FUNCOES DE FUNCIONAMENTO PARA O (1) ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	// SELF TEST, SO PARA VER SE OS RESULTADOS SAO RELACIONAVEIS COM OS DE FABRICA,
	// PARECE ME HAVER CONFIGURACOES CONTRADITORIAS
	void ICM_SelfTest(float * destination){

		uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
		uint8_t selfTest[6];
		int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
		float factoryTrim[6];
		uint8_t FS = 0;

		ICM_SelectBank(USER_BANK_0);
		ICM_SetClock(CLK_BEST_AVAIL);

		// Set gyro sample rate to 1 kHz
		ICM_WriteOneByte(0x00, 0x00);

		// Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
		ICM_WriteOneByte(GYRO_CONFIG_1, 0x11);

		// Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
		// Set full scale range for the accelerometer to 2 g
		ICM_WriteOneByte(ACCEL_CONFIG_1, 0x11);

		for (int ii = 0; ii < 200; ii++){

			printf("BHW::ii = %d", ii);

			// Read the six raw data registers into data array
			ICM_readBytes(ACCEL_XOUT_H, rawData, 6);
			aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
			aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

			// Read the six raw data registers sequentially into data array
			ICM_readBytes(GYRO_XOUT_H, rawData, 6);
			// Turn the MSB and LSB into a signed 16-bit value
			gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
			gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		}

		// Get average of 200 values and store as average current readings
		for (int ii =0; ii < 3; ii++)
		{
			aAvg[ii] /= 200;
			gAvg[ii] /= 200;
		}

		ICM_SelectBank(USER_BANK_2);

		// Configure the accelerometer for self-test
		// Enable self test on all three axes and set accelerometer range to +/- 2 g
		// Parece me que ha configuracoes no acelerometro que nao foram feitas
		ICM_WriteOneByte(ACCEL_CONFIG_2, 0x1C);

		// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
		ICM_WriteOneByte(GYRO_CONFIG_2,  0x38);
		HAL_Delay(25);  // Delay a while to let the device stabilize

		// Switch to user bank 0
		ICM_SelectBank(USER_BANK_0);

		// Get average self-test values of gyro and acclerometer
		for (int ii = 0; ii < 200; ii++)
		{
			// Read the six raw data registers into data array
			ICM_readBytes(ACCEL_XOUT_H, rawData, 6);
			// Turn the MSB and LSB into a signed 16-bit value
			aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
			aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

			// Read the six raw data registers sequentially into data array
			ICM_readBytes(GYRO_XOUT_H, rawData, 6);
			// Turn the MSB and LSB into a signed 16-bit value
			gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
			gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		}

		// Get average of 200 values and store as average self-test readings
		for (int ii =0; ii < 3; ii++)
		{
			aSTAvg[ii] /= 200;
			gSTAvg[ii] /= 200;
		}

		// Switch to user bank 2
		ICM_SelectBank(USER_BANK_2);

		// Configure the gyro and accelerometer for normal operation
		ICM_WriteOneByte( ACCEL_CONFIG_2, 0x00);
		ICM_WriteOneByte( GYRO_CONFIG_2,  0x00);
		HAL_Delay(25);  // Delay a while to let the device stabilize

		// Switch to user bank 1
		ICM_SelectBank(USER_BANK_1);

		// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
		// X-axis accel self-test results
		ICM_ReadOneByte(SELF_TEST_X_ACCEL, &selfTest[0]);
		// Y-axis accel self-test results
		ICM_ReadOneByte(SELF_TEST_Y_ACCEL, &selfTest[1]);
		// Z-axis accel self-test results
		ICM_ReadOneByte(SELF_TEST_Z_ACCEL, &selfTest[2]);
		// X-axis gyro self-test results
		ICM_ReadOneByte(SELF_TEST_X_GYRO, &selfTest[3]);
		// Y-axis gyro self-test results
		ICM_ReadOneByte(SELF_TEST_Y_GYRO, &selfTest[4]);
		// Z-axis gyro self-test results
		ICM_ReadOneByte(SELF_TEST_Z_GYRO, &selfTest[5]);

		// Switch to user bank 0
		ICM_SelectBank(USER_BANK_0);

		// Retrieve factory self-test value from self-test code reads
		// FT[Xa] factory trim calculation
		factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
		// FT[Ya] factory trim calculation
		factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
		// FT[Za] factory trim calculation
		factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
		// FT[Xg] factory trim calculation
		factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
		// FT[Yg] factory trim calculation
		factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
		// FT[Zg] factory trim calculation
		factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

		// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
		// of the Self-Test Response
		// To get percent, must multiply by 100
		for (int i = 0; i < 3; i++)
		{
			// Report percent differences
			destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;
			// Report percent differences
			destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/ factoryTrim[i+3] - 100.;
		}
		// destination missing to send such percentage, what does not make sense to me is the GYRO and Acelerometer configs
	}

	// FAZ A SUPOSTA CALIBRACAO PARA A RELACAO ACTUAL DO IMU
	void ICM_Calibrate(float * gyroBias, float * accelBias)
	{
		uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
		uint16_t ii, packet_count, fifo_count;
		int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};


		// reset device
		// Write a one to bit 7 reset bit; toggle reset device
		HAL_Delay(200);

		// get stable time source; Auto select clock source to be PLL gyroscope
		// reference if ready else use the internal oscillator, bits 2:0 = 001
		ICM_SelectBank(USER_BANK_0);
		ICM_SetClock(CLK_BEST_AVAIL);
		HAL_Delay(200);

		// Configure device for bias calculation
		// Disable all interrupts
		ICM_WriteOneByte(INT_ENABLE, 0x00);
		// Disable FIFO
		ICM_WriteOneByte(FIFO_EN_1, 0x00);
		ICM_WriteOneByte(FIFO_EN_2, 0x00);
		// Turn on internal clock source
		ICM_SetClock(0x00);
		// Disable I2C master
		//writeByte(ICM20948_ADDRESS, I2C_MST_CTRL, 0x00); Already disabled
		// Disable FIFO and I2C master modes
		ICM_WriteOneByte(USER_CTRL, 0x00);
		// Reset FIFO and DMP
		ICM_WriteOneByte(USER_CTRL, 0x08);
		ICM_WriteOneByte(FIFO_RST, 0x1F);
		HAL_Delay(10);
		ICM_WriteOneByte(FIFO_RST, 0x00);
		HAL_Delay(15);

		// Set FIFO mode to snapshot
		ICM_WriteOneByte(FIFO_MODE, 0x1F);
		// Switch to user bank 2
		ICM_SelectBank(USER_BANK_2);
		// Configure ICM20948 gyro and accelerometer for bias calculation
		// Set low-pass filter to 188 Hz
		ICM_WriteOneByte(GYRO_CONFIG_1, 0x01);
		// Set sample rate to 1 kHz
		ICM_WriteOneByte(GYRO_SMPLRT_DIV, 0x00);
		// Set gyro full-scale to 250 degrees per second, maximum sensitivity
		ICM_WriteOneByte(GYRO_CONFIG_1, 0x00);
		// Set accelerometer full-scale to 2 g, maximum sensitivity
		ICM_WriteOneByte(ACCEL_CONFIG_1, 0x00);

		uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
		uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

		// Switch to user bank 0
		ICM_SelectBank(USER_BANK_0);
		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		ICM_WriteOneByte(USER_CTRL, 0x40);  // Enable FIFO
		// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
		// ICM20948)
		ICM_WriteOneByte(FIFO_EN_2, 0x1E);
		HAL_Delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		// Disable gyro and accelerometer sensors for FIFO
		ICM_WriteOneByte(FIFO_EN_2, 0x00);
		// Read FIFO sample count
		ICM_readBytes(FIFO_COUNTH, data, 2);
		fifo_count = ((uint16_t)data[0] << 8) | data[1];
		// How many sets of full gyro and accelerometer data for averaging
		packet_count = fifo_count/12;

		for (ii = 0; ii < packet_count; ii++)
		{
			int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
			// Read data for averaging
			ICM_readBytes(FIFO_R_W, data, 12);
			// Form signed 16-bit integer for each sample in FIFO
			accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
			accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
			accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
			gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
			gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
			gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

			// Sum individual signed 16-bit biases to get accumulated signed 32-bit
			// biases.
			accel_bias[0] += (int32_t) accel_temp[0];
			accel_bias[1] += (int32_t) accel_temp[1];
			accel_bias[2] += (int32_t) accel_temp[2];
			gyro_bias[0]  += (int32_t) gyro_temp[0];
			gyro_bias[1]  += (int32_t) gyro_temp[1];
			gyro_bias[2]  += (int32_t) gyro_temp[2];
		}

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] /= (int32_t) packet_count;
		accel_bias[1] /= (int32_t) packet_count;
		accel_bias[2] /= (int32_t) packet_count;
		gyro_bias[0]  /= (int32_t) packet_count;
		gyro_bias[1]  /= (int32_t) packet_count;
		gyro_bias[2]  /= (int32_t) packet_count;

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		if (accel_bias[2] > 0L)
		{
			accel_bias[2] -= (int32_t) accelsensitivity;
		}
		else
		{
			accel_bias[2] += (int32_t) accelsensitivity;
		}

		// Construct the gyro biases for push to the hardware gyro bias registers,
		// which are reset to zero upon device startup.
		// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
		// format.
		data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
		// Biases are additive, so change sign on calculated average gyro biases
		data[1] = (-gyro_bias[0]/4)       & 0xFF;
		data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
		data[3] = (-gyro_bias[1]/4)       & 0xFF;
		data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
		data[5] = (-gyro_bias[2]/4)       & 0xFF;

		// Switch to user bank 2
		ICM_SelectBank(USER_BANK_2);

		// Push gyro biases to hardware registers
		ICM_WriteOneByte(XG_OFFSET_H, data[0]);
		ICM_WriteOneByte(XG_OFFSET_L, data[1]);
		ICM_WriteOneByte(YG_OFFSET_H, data[2]);
		ICM_WriteOneByte(YG_OFFSET_L, data[3]);
		ICM_WriteOneByte(ZG_OFFSET_H, data[4]);
		ICM_WriteOneByte(ZG_OFFSET_L, data[5]);

		// Output scaled gyro biases for display in the main program
		gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
		gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
		gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

		// Construct the accelerometer biases for push to the hardware accelerometer
		// bias registers. These registers contain factory trim values which must be
		// added to the calculated accelerometer biases; on boot up these registers
		// will hold non-zero values. In addition, bit 0 of the lower byte must be
		// preserved since it is used for temperature compensation calculations.
		// Accelerometer bias registers expect bias input as 2048 LSB per g, so that
		// the accelerometer biases calculated above must be divided by 8.

		// Switch to user bank 1
		ICM_SelectBank(USER_BANK_1);
		// A place to hold the factory accelerometer trim biases
		int32_t accel_bias_reg[3] = {0, 0, 0};
		// Read factory accelerometer trim values
		ICM_readBytes(XA_OFFSET_H, data, 2);
		accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		ICM_readBytes(YA_OFFSET_H, data, 2);
		accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		ICM_readBytes(ZA_OFFSET_H, data, 2);
		accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

		// Define mask for temperature compensation bit 0 of lower byte of
		// accelerometer bias registers
		uint32_t mask = 1uL;
		// Define array to hold mask bit for each accelerometer bias axis
		uint8_t mask_bit[3] = {0, 0, 0};

		for (ii = 0; ii < 3; ii++)
		{
		// If temperature compensation bit is set, record that fact in mask_bit
			if ((accel_bias_reg[ii] & mask))
			{
				mask_bit[ii] = 0x01;
			}
		}

		// Construct total accelerometer bias, including calculated average
		// accelerometer bias from above
		// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
		// (16 g full scale)
		accel_bias_reg[0] -= (accel_bias[0]/8);
		accel_bias_reg[1] -= (accel_bias[1]/8);
		accel_bias_reg[2] -= (accel_bias[2]/8);

		data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
		data[1] = (accel_bias_reg[0])      & 0xFF;
		// preserve temperature compensation bit when writing back to accelerometer
		// bias registers
		data[1] = data[1] | mask_bit[0];
		data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
		data[3] = (accel_bias_reg[1])      & 0xFF;
		// Preserve temperature compensation bit when writing back to accelerometer
		// bias registers
		data[3] = data[3] | mask_bit[1];
		data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
		data[5] = (accel_bias_reg[2])      & 0xFF;
		// Preserve temperature compensation bit when writing back to accelerometer
		// bias registers
		data[5] = data[5] | mask_bit[2];

		// Apparently this is not working for the acceleration biases in the ICM-20948
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers // ELE PROPRIO TEM ERROS
		ICM_WriteOneByte(XA_OFFSET_H, data[0]);
		ICM_WriteOneByte(XA_OFFSET_L, data[1]);
		ICM_WriteOneByte(YA_OFFSET_H, data[2]);
		ICM_WriteOneByte(YA_OFFSET_L, data[3]);
		ICM_WriteOneByte(ZA_OFFSET_H, data[4]);
		ICM_WriteOneByte(ZA_OFFSET_L, data[5]);

		// Output scaled accelerometer biases for display in the main program
		accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
		accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
		accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
		// Switch to user bank 0
		ICM_SelectBank(USER_BANK_0);
	}

	void ICM_Magnetometer_Init()
	{

		// Write code to initialise magnetometer
		// Bypass I2C master interface and turn on magnetometer
		//writeByte(ICM20948_ADDRESS, INT_PIN_CFG, 0x02); Already set in initICM20948

		// Configure the magnetometer for continuous read and highest resolution.
		// Enable continuous mode data acquisition Mmode (bits [3:0]),
		// 0010 for 8 Hz and 0110 for 100 Hz sample rates.

		// Set magnetometer data resolution and sample ODR
		i2c_Mag_write(AK09916_CNTL2, 0x08);
		HAL_Delay(10);
	}

	void ICM_Magnetometer_Calibrate(float * bias_dest, float * scale_dest)
	{
	  uint16_t ii = 0, sample_count = 0;
	  int32_t mag_bias[3]  = {0, 0, 0},
			  mag_scale[3] = {0, 0, 0};
	  int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
			  mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
			  mag_temp[3] = {0, 0, 0};

	  // Make sure resolution has been calculated
	  float mRes = 10.0f * 4912.0f / 32760.0f;

	  printf("Mag Calibration: Wave device in a figure 8 until done! 4 seconds to get ready followed by 15 seconds of sampling)\r\n");
	  delay(4000);

	  uint8_t Mmode = M_100HZ;

	  // shoot for ~fifteen seconds of mag data
	  // at 8 Hz ODR, new mag data is available every 125 ms
	  if (Mmode == M_8HZ)
	  {
		sample_count = 128;
	  }
	  // at 100 Hz ODR, new mag data is available every 10 ms
	  if (Mmode == M_100HZ)
	  {
		sample_count = 1500;
	  }

	  for (ii = 0; ii < sample_count; ii++)
	  {
		readMagData(mag_temp);  // Read the mag data

		for (int jj = 0; jj < 3; jj++)
		{
		  if (mag_temp[jj] > mag_max[jj])
		  {
			mag_max[jj] = mag_temp[jj];
		  }
		  if (mag_temp[jj] < mag_min[jj])
		  {
			mag_min[jj] = mag_temp[jj];
		  }
		}

		if (Mmode == M_8HZ)
		{
		  HAL_Delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
		}
		if (Mmode == M_100HZ)
		{
		  HAL_Delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
		}
	  }

	  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	  // Get hard iron correction
	  // Get 'average' x mag bias in counts
	  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
	  // Get 'average' y mag bias in counts
	  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
	  // Get 'average' z mag bias in counts
	  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

	  // Save mag biases in G for main program
	  bias_dest[0] = (float)mag_bias[0] * mRes;// * factoryMagCalibration[0];
	  bias_dest[1] = (float)mag_bias[1] * mRes;// * factoryMagCalibration[1];
	  bias_dest[2] = (float)mag_bias[2] * mRes;// * factoryMagCalibration[2];

	  // Get soft iron correction estimate
	  // Get average x axis max chord length in counts
	  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
	  // Get average y axis max chord length in counts
	  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
	  // Get average z axis max chord length in counts
	  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

	  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	  avg_rad /= 3.0;

	  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
	  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
	  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

	  printf("Mag Calibration done!");
	}

	void readMagData(int16_t * destination)
	{
	  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
	  // of data acquisition
	  uint8_t rawData[8];
	  // Wait for magnetometer data ready bit to be set
	  if (ICM_Mag_Read(AK09916_ST1) & 0x01)
	  {

		// Read the six raw data and ST2 registers sequentially into data array
		rawData[0]=ICM_Mag_Read(AK09916_XOUT_L);
		rawData[1]=ICM_Mag_Read(AK09916_XOUT_H);
		rawData[2]=ICM_Mag_Read(AK09916_YOUT_L);
		rawData[3]=ICM_Mag_Read(AK09916_YOUT_H);
		rawData[4]=ICM_Mag_Read(AK09916_ZOUT_L);
		rawData[5]=ICM_Mag_Read(AK09916_ZOUT_H);
		rawData[6]=ICM_Mag_Read(0x17);
		rawData[7]=ICM_Mag_Read(AK09916_ST2);

		uint8_t c = rawData[7]; // End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		  // Remove once finished

		if (!(c & 0x08))
		{
		  // Turn the MSB and LSB into a signed 16-bit value
		  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
		  // Data stored as little Endian
		  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
		  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	  }
	}

	void ICM_Init()
	{
		// Get stable time source
		// Auto select clock source to be PLL gyroscope reference if ready else
		ICM_WriteOneByte(PWR_MGMT_1, 0x01);
		HAL_Delay(200);

		// Switch to user bank 2
		ICM_SelectBank(USER_BANK_2);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set gyro bandwidth to 51.2 Hz,
		// respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion
		// update rates cannot be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the ICM20948, it is possible to get gyro sample rates of 32 kHz (!),
		// 8 kHz, or 1 kHz
		// Set gyroscope full scale range to 250 dps
		ICM_WriteOneByte(GYRO_CONFIG_1, 0x19);
		ICM_WriteOneByte(TEMP_CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + GYRO_SMPLRT_DIV)
		// Use a 220 Hz rate; a rate consistent with the filter update rate
		// determined inset in CONFIG above.

		ICM_WriteOneByte(GYRO_SMPLRT_DIV, 0x04);

		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
		// left-shifted into positions 4:3

		// Set accelerometer full-scale range configuration
		// Get current ACCEL_CONFIG register value
		uint8_t c ;
		ICM_ReadOneByte(ACCEL_CONFIG_1, &c);
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x06;  // Clear AFS bits [4:3]
		c = c | Ascale << 1; // Set full scale range for the accelerometer
		c = c | 0x01; // Set enable accel DLPF for the accelerometer
		c = c | 0x18; // and set DLFPFCFG to 50.4 hz
		// Write new ACCEL_CONFIG register value
		ICM_WriteOneByte(ACCEL_CONFIG_1, c);

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by
		// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
		// 1.13 kHz
		ICM_WriteOneByte(ACCEL_SMPLRT_DIV_2, 0x04);
		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because
		// of the GYRO_SMPLRT_DIV setting

		// Switch to user bank 0
		ICM_SelectBank(USER_BANK_0);

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
		// until interrupt cleared, clear on read of INT_STATUS, and enable
		// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
		// controlled by the Arduino as master.
		ICM_WriteOneByte(INT_PIN_CFG, 0x22);
		// Enable data ready (bit 0) interrupt
		ICM_WriteOneByte(INT_ENABLE_1, 0x01);
	}

	void readAccelData(int16_t * destination)
	{
	  uint8_t rawData[6];  // x/y/z accel register data stored here
	  // Read the six raw data registers into data array
	  ICM_readBytes(ACCEL_XOUT_H, 6, &rawData[0]);
	  // Turn the MSB and LSB into a signed 16-bit value
	  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
	  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
	  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
	}

	void readGyroData(int16_t * destination)
	{
	  uint8_t rawData[6];  // x/y/z gyro register data stored here
	  // Read the six raw data registers sequentially into data array
	  ICM_readBytes(GYRO_XOUT_H, 6, &rawData[0]);

	  // Turn the MSB and LSB into a signed 16-bit value
	  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
	  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
	  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
	}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCOES UTEIS ORIGINALMENTES USADAS SO PARA (2) NO ENTANTO ALGUMAS FORAM APROVEITADAS PARA (1) ///////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ICM_SelectBank(uint8_t bank) {
	ICM_WriteOneByte(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void) {
	ICM_WriteOneByte(0x03, 0x78);
}
void ICM_CSHigh(void) {
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, SET);
}
void ICM_CSLow(void) {
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, RESET);
}
void ICM_SetClock(uint8_t clk) {
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void) {
	ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM_AccelGyroOn(void) {
	ICM_WriteOneByte(0x07, (0x00 | 0x00));
}
uint8_t ICM_WHOAMI(void) {
	uint8_t spiData = 0x01;
	ICM_ReadOneByte(0x00, &spiData);
	return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
	ICM_WriteOneByte(GYRO_CONFIG_1, (rate|lpf));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
