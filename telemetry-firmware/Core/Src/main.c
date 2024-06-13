/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ICM20948.h"
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define V_ADC 3.3/4096

#define GPS_Enable 1;
#define GPS_Disables 0;
#define ICM_Enable 1;
#define ICM_Disable 0;
#define POT_Enable 1;
#define POT_Disable 0;
#define Servo_Enable 1;
#define Servo_Disable 0;

//#define DRCPATTISON 1
//#define CORYCLINE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
int  GPS_ready = 0;
int start_race = 0;
uint32_t GPS_BR=9600;
int time_count=0;
int time_total=0;
// SD CARD RELATED ///////////////////////////////////////////////////////////////////////////////////////////
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CAN ///////////////////////////////////////////////////////////////////////////////////////////////////////
CAN_TxHeaderTypeDef pTxHeader;
uint32_t pTxMailbox;

CAN_RxHeaderTypeDef pRxHeader;
uint32_t pRxMailbox;

CAN_FilterTypeDef CAN_filter1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CAN UNION

union CAN_M{
	int i[2];
	float f[2];
	uint8_t bytes[8];
} can_m;

union CAN_M tx_union;
union CAN_M rx_union;
uint8_t can_m_received=0;

#define Especial	50  // Total Voltage :: V1t :: P2t :: P3t :: P4t :: P5t :: Max Temp :: Pack Ref ::
#define PACK_V_1	51  // All 6 voltages
#define PACK_V_2	52  // All 6 voltages
#define PACK_V_3	53  // All 6 voltages
#define PACK_V_4	54  // All 6 voltages
#define PACK_V_5	55  // All 6 voltages
#define PACK_1_2	121 // All 4 temperatures
#define PACK_3_4	122 // All 4 temperatures
#define PACK_5		123 // All 4 temperatures
#define Tele_SR		90
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

int main(void)
{
  /* USER CODE BEGIN 1 */
	char *token, *string;
	uint8_t buff[1200];
	char buffStr[600];
	char nmeaSnt[80];
	uint8_t intSum;
	char hex[2];
	int i,j, file=0;

	////////////////////////////////////////////////////////////////////////////////////

	char *rawSum;
	char smNmbr[3];

	uint8_t cnt = 0;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	/*char daytime[6];
	char valid;
	char lat[9];
	char hemLat;
	char lon[10];
	char hemLon;
	char speed[4];
	char track[6];
	char date[6];
	char variation[5];
	char orientation;

	char *daytime_raw;
	char *valid_raw;
	char *lat_raw;
	char *hemLat_raw;
	char *lon_raw;
	char *hemLon_raw;
	char *speed_raw;
	char *track_raw;
	char *date_raw;
	char *variation_raw;
	char *orientation_raw;*/
//////////////////////////////////////////////////// EXTRA ME

	uint8_t valid_5,valid_4,valid_3,valid_2,valid_1,valid_t;
	uint8_t pack[6][6], tempack[5][4], templacas[3];
	/////////////////////////////////////////


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

	uint32_t value[2];
	uint32_t suspension1,suspension2;
	HAL_StatusTypeDef manager;


	// Inicializar UART4 e o ADC /////////////
	//HAL_UART_Receive_DMA(&huart4, buff, 1200);
	//HAL_Delay(100); //necessario?
	//HAL_ADC_Start_DMA(&hadc1, value, 2); //sem pot's
	//////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	// CAN Initiations ///////////////////////////////////////////////////////////////////////////////////////
  	HAL_CAN_Start(&hcan2);
  	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  	//HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_filter1.FilterBank = 14;
	CAN_filter1.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_filter1.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_filter1.FilterIdHigh = 0x0000;
	CAN_filter1.FilterIdLow = 0x0000; //00FF
	CAN_filter1.FilterMaskIdHigh = 0x0000; //FFFF
	CAN_filter1.FilterMaskIdLow = 0x0000; //FF00
	CAN_filter1.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_filter1.FilterActivation = ENABLE;
	CAN_filter1.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan2, &CAN_filter1);

	// USED FOR PUTTING BAUDRATE AT 115200 /////
	/*uint8_t *set = "$PMTK251,115200*1F\r\n";
	manager=HAL_UART_Transmit(&huart4, set, 21, 1000);
	if(manager != HAL_OK)
	{
		printf("Error 1: Changing GPS baudrate.\r\n");
		while(1){

		}
	}*/
	//GPS_BR=115200;
	//HAL_UART_DeInit(&huart4);
	//////////////MX_UART4_Init();
	//HAL_UART_Init(&huart4);
	//HAL_UART_Receive_DMA(&huart4, buff, 1200);
	////////////////////////////////////////////

	// ICM INITIALIZATIONS ///////////////////////////////////////////////////////////////////////////////////
#if defined(ICM_Enable)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); // SDCard
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET); // IMU
	ICM_PowerOn();
	HAL_Delay(10); // Ainda falta ver se estes valores do ICM são normais antes de estar a ler.
#endif
	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	//SD CARD INIT ///////////////////////////////////////////////////////////////////////////////////////////
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
	{

		printf("Error 2: Mounting SDCARD.\r\n");
		while(1)
		{

		}
	}

	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	printf("SD CARD Total Size: \t%lu\n",total);
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	printf("SD CARD Free Space: \t%lu\n\n",free_space);

	if(free_space < total/8)
	{
		printf("Error 3: 1/8 left. \r\n");
		while(1){

		}
	}

	/*while(1){

	}*/
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	int var=0;
	while(1)
	{
		pTxHeader.DLC=1;
		pTxHeader.IDE=CAN_ID_STD;
		pTxHeader.RTR=CAN_RTR_DATA;
		pTxHeader.StdId=Especial;
		tx_union.bytes[0]=224;
		tx_union.bytes[1]=250;
		tx_union.bytes[2]=260;
		tx_union.bytes[3]=180;
		tx_union.bytes[4]=165;
		tx_union.bytes[5]=165;
		HAL_CAN_AddTxMessage(&hcan2, &pTxHeader, &tx_union, &pTxMailbox);
		while(1)
		{
			for (var=0; var < 1000; var++ )
			{

			}
			break;
		}

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	while(1)
	{
		can_m_received=1;
		if(can_m_received==1)
		{
			if(pRxHeader.StdId==Tele_SR)
			{
				if(rx_union.bytes[0]==0)
				{
					start_race=1;
					tx_union.bytes[0]=1;
				}
				else if(rx_union.bytes[0]==1)
				{
					start_race=0;
					tx_union.bytes[0]=0;
				}
					pTxHeader.DLC=1;
					pTxHeader.IDE=CAN_ID_STD;
					pTxHeader.RTR=CAN_RTR_DATA;
					pTxHeader.StdId=Tele_SR;
					HAL_CAN_AddTxMessage(&hcan2, &pTxHeader, &tx_union, &pTxMailbox);
			}

			//tensões para os packs de bateria
			for(i=50;i<56;i++)
			{
				if(pRxHeader.StdId==i && start_race==1){
					pack[i-50][0]=rx_union.bytes[0];
					pack[i-50][1]=rx_union.bytes[1];
					pack[i-50][2]=rx_union.bytes[2];
					pack[i-50][3]=rx_union.bytes[3];
					pack[i-50][4]=rx_union.bytes[4];
					pack[i-50][5]=rx_union.bytes[5];
					//printf("p1 %d  p2 %d  p3 %d  p4 %d  p5 %d pt %d\n\r",pack[i-50][0],pack[i-50][1],pack[i-50][2],pack[i-50][3],pack[i-50][4],pack[i-50][5]);
				}
			}
			pRxHeader.StdId=121;
			start_race=1;
			//temperatura para o primeiro e segundo pack de baterias
			if(pRxHeader.StdId==121 && start_race==1){
				tempack[0][0]/rx_union.bytes[0];
				tempack[0][1]=rx_union.bytes[1];
				tempack[0][2]=rx_union.bytes[2];
				tempack[0][3]=rx_union.bytes[3];
				tempack[1][0]=rx_union.bytes[4];
				tempack[1][1]=rx_union.bytes[5];
				tempack[1][2]=rx_union.bytes[6];
				tempack[1][3]=rx_union.bytes[7];
				//printf("p1c1 %d  p1c2 %d  p1c3 %d  p1c4 %d  p2c1 %d p2c2 %d p2c3 %d p2c4 %d \n\r",tempack[0][0],tempack[0][1],tempack[0][2],tempack[0][3],tempack[1][0],tempack[1][1],tempack[1][2], tempack[1][3]);
			}
			//temperatura para o terceiro e quarto pack de baterias
			if(pRxHeader.StdId==121 && start_race==1){
				tempack[2][0]=rx_union.bytes[0];
				tempack[2][1]=rx_union.bytes[1];
				tempack[2][2]=rx_union.bytes[2];
				tempack[2][3]=rx_union.bytes[3];
				tempack[3][0]=rx_union.bytes[4];
				tempack[3][1]=rx_union.bytes[5];
				tempack[3][2]=rx_union.bytes[6];
				tempack[3][3]=rx_union.bytes[7];
				//printf("p1 %d  p2 %d  p3 %d  p4 %d  p5 %d pt %d\n\r",pack[i-50][0],pack[i-50][1],pack[i-50][2],pack[i-50][3],pack[i-50][4],pack[i-50][5]);
			}
			//temperatura para o quinto pack de baterias
			if(pRxHeader.StdId==121 && start_race==1){
				tempack[4][0]=rx_union.bytes[0];
				tempack[4][1]=rx_union.bytes[1];
				tempack[4][2]=rx_union.bytes[2];
				tempack[4][3]=rx_union.bytes[3];
				//printf("p1 %d  p2 %d  p3 %d  p4 %d  p5 %d pt %d\n\r",pack[i-50][0],pack[i-50][1],pack[i-50][2],pack[i-50][3],pack[i-50][4],pack[i-50][5]);
			}
			//temperatura para as placas
			for(i=131;i<134;i++)
			{
				if(pRxHeader.StdId==i && start_race==1)
				{
					templacas[i-131]=rx_union.bytes[0];
				}
			}

			can_m_received=0;
		}

		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

			if(can_m_received==1)
			{
				if(pRxHeader.StdId==Tele_SR)
				{
					if(rx_union.bytes[0]==0)
					{
						start_race=1;
						tx_union.bytes[0]=1;
					}
					else if(rx_union.bytes[0]==1)
					{
						start_race=0;
						tx_union.bytes[0]=0;
					}
					can_m_received=0;
					pTxHeader.DLC=1;
					pTxHeader.IDE=CAN_ID_STD;
					pTxHeader.RTR=CAN_RTR_DATA;
					pTxHeader.StdId=Tele_SR;

				HAL_CAN_AddTxMessage(&hcan2, &pTxHeader, &tx_union, &pTxMailbox);
				}
			}

		if(start_race==1)
		{
			valid_t=1;
			if(time_count>=10 && valid_t==1)
			{

				//GPS READINGS ///////////////////////////////////////////////////////////////////////////
#if defined(GPS_Enable)
				while(GPS_ready!=1)
				{
					GPS_ready=1;
				}

				memset(buffStr, 0, 600);

				sprintf(buffStr, "%s", buff);

				string = strdup(buffStr);

				while ((token = strsep(&string, "\n")) != NULL)
				{

					memset(nmeaSnt, 0, 80);

					sprintf(nmeaSnt, "%s", token);

					printf("%s", nmeaSnt);

					// selecting only $GNGLL sentences, combined GPS and GLONASS
					// on my GPS sensor this good NMEA sentence is always 50 characters
					if ( strstr(nmeaSnt, "$GPRMC") != 0 && strlen(nmeaSnt) > 49 && strstr(nmeaSnt, "*") != 0)
					{

						rawSum = strstr(nmeaSnt, "*");

						memcpy(smNmbr, &rawSum[1], 2);

						smNmbr[2] = '\0';

						intSum = nmea0183_checksum(nmeaSnt);

						// "%X" unsigned hexadecimal integer (capital letters)
						sprintf(hex, "%X", intSum);

						// checksum data verification, if OK, then we can really trust
						// the data in the the NMEA sentence
						int empties[12];
						memset(empties, 0, sizeof(empties));
						if (strstr(smNmbr, hex) != NULL)
						{
							int i=0, j=0, empties_n=0;
							while(nmeaSnt[i]!='\0')
							{
								if(nmeaSnt[i]==',' && nmeaSnt[i+1]==',')
								{
									empties_n++;
									empties[j]=empties_n;
									j++;
								}
								else if(nmeaSnt[i]==',' && nmeaSnt[i+1]!=',')
								{
									empties_n++;
								}
								else if(nmeaSnt[i]==',' && nmeaSnt[i+1]=='*')
								{
									empties_n++;
									empties[j]=empties_n;
									j++;
								}

								i++;
							}
							i=0;
							//printf("STRING:%s\n",nmeaSnt);

							// we want to send the string "nmeaSnt"
							// splitting the good NMEA sentence into the tokens by the comma delimiter
							/*for (char *pV = strtok(nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ",")) {
								printf("STRING:%s\n",pV);
								printf("CNT %d and empties[i] %d\n\r",cnt,empties[i]);
								fflush(stdout);
								while(cnt==empties[i])
								{
									i++;
									cnt++;
								}
								switch (cnt) {

									case 1:
										daytime_raw=strdup(pV);
										printf("1: %s\n\r",daytime_raw);
										break;
									case 2:
										valid_raw=strdup(pV);
										printf("2: %s\n\r",valid_raw);
										if(strcmp(valid_raw,"V"))
										{
											GPS_retry=1;
											pV=NULL;
										}
										else
										{
											GPS_retry=0;
										}
										break;
									case 3:
										lat_raw=strdup(pV);
										printf("3: %s\n\r",lat_raw);
										break;
									case 4:
										hemLat_raw=strdup(pV);
										printf("4: %s\n\r",hemLat_raw);
										break;
									case 5:
										lon_raw=strdup(pV);
										printf("5: %s\n\r",lon_raw);
										break;
									case 6:
										hemLon_raw=strdup(pV);
										printf("6: %s\n\r",hemLon_raw);
										break;
									case 7:
										speed_raw=strdup(pV);
										printf("7: %s\n\r",speed_raw);
										break;
									case 8:
										track_raw=strdup(pV);
										printf("8: %s\n\r",track_raw);
										break;
									case 9:
										date_raw=strdup(pV);
										printf("9: %s\n\r",date_raw);
										break;
									case 10:
										variation_raw=strdup(pV);
										printf("10: %s\n\r",variation_raw);
										break;
									case 11:
										orientation_raw=strdup(pV);
										printf("11: %s\n\r",orientation_raw);
										break;
								}
								fflush(stdout);
								cnt++;
							}  // end for()*/
							//cnt=0;
						} // end of of the checksum data verification


						//send the data to spi here


					} // end of $GNGLL sentences selection
					/*
					if (strstr(nmeaSnt, "$GPVTG") != 0 && strlen(nmeaSnt) > 29 && strstr(nmeaSnt, "*") != 0)
					{

							rawSum = strstr(nmeaSnt, "*");

							memcpy(smNmbr, &rawSum[1], 2);

							smNmbr[2] = '\0';

							intSum = nmea0183_checksum(nmeaSnt);

							// "%X" unsigned hexadecimal integer (capital letters)
							sprintf(hex, "%X", intSum);

							// checksum data verification, if OK, then we can really trust
							// the data in the the NMEA sentence
							int empties[12];
							memset(empties, 0, sizeof(empties));
							if (strstr(smNmbr, hex) != NULL) {
								int i=0, j=0, empties_n=0;

								while(nmeaSnt[i]!='\0')
								{
									if(nmeaSnt[i]==',' && nmeaSnt[i+1]==',')
									{
										empties_n++;
										empties[j]=empties_n;
										j++;
									}
									else if(nmeaSnt[i]==',' && nmeaSnt[i+1]!=',')
									{
										empties_n++;
									}
									else if(nmeaSnt[i]==',' && nmeaSnt[i+1]=='*')
									{
										empties_n++;
										empties[j]=empties_n;
										j++;
									}

										i++;
									}
									i=0;
							}
							//send the data to spi here
							//printf("STRING:%s\n",nmeaSnt);

					}// end of $GNVTG sentences selection*/
				}
			}
#endif
				// END GPS READINGS ///////////////////////////////////////////////////////////////////////

				// SUSPENSION READINGS ////////////////////////////////////////////////////////////////////
#if defined(POT_Enable)
				/*suspension1=value[0];
				suspension2=value[1];*/
#endif
				// END SUSPENSION READINGS ////////////////////////////////////////////////////////////////

				// START ICM READINGS /////////////////////////////////////////////////////////////////////
#if defined(ICM_Enable)

#if defined(DRCPATTISON)
				uint8_t var;
				ICM_ReadOneByte(INT_STATUS_1,&var);
				if (var & 0x01)
				{
					int16_t accelCount[3], ax, ay, az, aRes;
				    readAccelData(accelCount);  // Read the x/y/z adc values

				    // Now we'll calculate the accleration value into actual g's
				    // This depends on scale being set
				    ax = (float)accelCount[0] * aRes; // - myIMU.accelBias[0];
				    ay = (float)accelCount[1] * aRes; // - myIMU.accelBias[1];
				    az = (float)accelCount[2] * aRes; // - myIMU.accelBias[2];

				    int16_t gyroCount[3], gx, gy, gz, gRes;
				    readGyroData(gyroCount);  // Read the x/y/z adc values

				    // Calculate the gyro value into actual degrees per second
				    // This depends on scale being set
				    gx = (float)gyroCount[0] * gRes;
				    gy = (float)gyroCount[1] * gRes;
				    gz = (float)gyroCount[2] * gRes;

				    int16_t magCount[3], mx, my, mz, magBias[3], mRes;
				    readMagData(magCount);  // Read the x/y/z adc values

				    // Calculate the magnetometer values in milliGauss
				    // Include factory calibration per data sheet and user environmental
				    // corrections
				    // Get actual magnetometer value, this depends on scale being set
				    mx = (float)magCount[0] * mRes - magBias[0];
				    my = (float)magCount[1] * mRes - magBias[1];
				    mz = (float)magCount[2] * mRes - magBias[2];
				  }
#elif defined(CORYCLINE)
				ICM_SelectBank(USER_BANK_0);
				HAL_Delay(10);

				// Obtain accelerometer and gyro data
				ICM_ReadAccelGyroData();

				// Obtain magnetometer data
				ICM_ReadMagData(mag_data);

				// Print raw, but joined, axis data values to screen
				printf(	"(Ax: %u | Ay: %u | Az: %u) \n\r (Gx: %u | Gy: %u | Gz: %u)\n\r"
						"(Mx: %i | My: %i | Mz: %i)\n\r",accel_data[0], accel_data[1], accel_data[2],
						gyro_data[0], gyro_data[1], gyro_data[2],mag_data[0], mag_data[1], mag_data[2]);
				fflush(stdout);
#endif
#endif
				// END ICM READINGS ///////////////////////////////////////////////////////////////////////

				// RECORD SDCARD //////////////////////////////////////////////////////////////////////////

				//tempo,temperaturas,tensão,gps
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET); // SDCard
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);

			    if(file==0){
			    	fresult = f_open(&fil, "file1.txt", FA_CREATE_ALWAYS | FA_WRITE);
			    	file=1;
			    }
			    else{
			    	fresult = f_open(&fil, "file1.txt", FA_OPEN_APPEND | FA_WRITE);
			    }

				clear_buffer();

				/* Writing text */
			    sprintf(buffer, "p1c1 %d  p1c2 %d  p1c3 %d  p1c4 %d  p2c1 %d p2c2 %d p2c3 %d p2c4 %d \n\r", tempack[0][0],tempack[0][1],tempack[0][2],tempack[0][3],tempack[1][0],tempack[1][1],tempack[1][2], tempack[1][3]);
				//sprintf(buffer, "%s \n\r", nmeaSnt);
			    fresult = f_write(&fil, buffer, strlen(buffer), &bw);

			    //printf("%s\r\n",buffer);

				if (fresult != FR_OK){
					printf("Error4: Could not write.\r\n");
				}

				/* Close file */
				fresult = f_close(&fil);

			  	if (fresult != FR_OK){
			  		printf("Error4: Could not close file.\r\n");
			  	}

			  	clear_buffer();

				//printf("%s\r\n",buffer);

			  	fresult = f_open(&fil, "file1.txt", FA_READ);

			  	f_read (&fil, buffer, f_size(&fil), &br);

			  	printf("%s\r\n", buffer);

			  	fresult = fclose(&fil);

			  	clear_buffer();

			  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET); // SDCard*/
				///////////////////////////////////////////////////////////////////////////////////////////
				time_count = 0;
		}
	}
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 900-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  //Sempre que mexer no ficheiro .ioc lembrar-se de alterar Baudrate para huart4.Init.BaudRate = GPS_BR;

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

int nmea0183_checksum(char *msg) {

	int checksum = 0;
	int j = 0;

	// the first $ sign and the last two bytes of original CRC + the * sign
	for (j = 1; j < strlen(msg) - 4; j++) {
		checksum = checksum ^ (unsigned) msg[j];
		//printf("byte: %c\n\r",msg[j]);
		fflush(stdout);
	}

	printf("CKSUM %d\n\r", checksum);
	return checksum;

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {

	GPS_ready = 1;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	GPS_ready = 0;

}

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
