/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MY_PARAMS 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef sensor;
FATFS *pSDFatfs = NULL;
RTC_TimeTypeDef time;
RTC_DateTypeDef date;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void printParams(bmp280_params_t* params);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  sensor.i2c = &hi2c1;
  sensor.addr = BMP280_I2C_ADDRESS_0;

  bmp280_params_t sensorParams;
  FRESULT res;

#if !MY_PARAMS
  bmp280_init_default_params(&sensorParams);
#else

  sensorParams.filter = BMP280_FILTER_OFF;
  sensorParams.mode = BMP280_MODE_FORCED;
  sensorParams.oversampling_humidity = BMP280_ULTRA_LOW_POWER;
  sensorParams.oversampling_pressure = BMP280_ULTRA_LOW_POWER;
  sensorParams.oversampling_temperature = BMP280_ULTRA_LOW_POWER;
  //sensorParams.standby = BMP280_STANDBY_;
#endif

  bool initCode = false;
  initCode =  bmp280_init(&sensor, &sensorParams);
  if(initCode)
  {
	  printParams(&sensorParams);
  }
  pSDFatfs = &SDFatFS;
  res = f_mount(pSDFatfs, "", 1);

  if(!res == FR_OK)
	  Error_Handler();
  res = f_mount(NULL, "", 0);

 // bool initCard_Sensor = (!res && initCode);
  //HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, (GPIO_PinState)initCard_Sensor);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bool isMeasuring = true;
  FIL logFile;
  char writeBuffer[150];
  unsigned int bytesToWrite = strlen(writeBuffer);
  unsigned int bytesWritten;
  float temp = 0.0f, press= 0.0f, hum = 0.0f;
  const float pressureScale = 1.0f / 1000.0f;

  while (1)
  {
	  HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);
	  bmp280_force_measurement(&sensor);
	  isMeasuring = true;
	  while(isMeasuring)
	  {
		  isMeasuring= bmp280_is_measuring(&sensor);

	  }

	  bmp280_read_float(&sensor, &temp, &press, &hum);
	  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	  snprintf(writeBuffer,
			  sizeof(writeBuffer),
			  "%d/%d/%d  %d:%d:%d : Temperature: %0.2f%cC, Pressure: %0.2fhPa\r\n",
			  date.Date, date.Month, date.Year,
			  time.Hours, time.Minutes, time.Seconds,
			  temp,176, press*pressureScale);
	  HAL_UART_Transmit(&huart2, (uint8_t*)writeBuffer, strlen(writeBuffer), 20);

	  res = f_mount(pSDFatfs, "", 1);
	  res = f_open(&logFile, "log.txt", FA_OPEN_APPEND |
			  FA_WRITE);
	  if(res != FR_OK)
	  {
		  printf("Opening file failed! res = %d\r\n", res);
		  continue;
	  }

	  res = f_write(&logFile, writeBuffer, bytesToWrite, &bytesWritten);
	  if(res != FR_OK)
	  {
		  printf("Writing operation failed! res = %d\r\n", res);
		  continue;
	  }
	  if(bytesWritten < bytesToWrite)
	  {
		  printf("Disk is full!\r\n");
	  }
	  res = f_close(&logFile);
	  if(res != FR_OK)
	  {
		  printf("Closing operation failed! res= %d\r\n", res);
		  continue;
	  }
	  res = f_mount(NULL, "", 0);
	  HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);
	  HAL_SuspendTick();
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x708, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	  HAL_ResumeTick();
	  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	  SystemClock_Config();

  }
  /* USER CODE END 3 */
}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 236;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
return;
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 19;
  sTime.Minutes = 47;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 26;
  sDate.Year = 25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserButton_Pin */
  GPIO_InitStruct.Pin = UserButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UserButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GreenLED_Pin */
  GPIO_InitStruct.Pin = GreenLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GreenLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Detect_Pin */
  GPIO_InitStruct.Pin = Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Detect_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
void printParams(bmp280_params_t* params)
{

	//filter
	switch(params->filter)
	{
	case BMP280_FILTER_OFF:
		printf("\r\nFILTER OFF\r\n");
		break;
	case BMP280_FILTER_2:
		printf("\r\nFILTER COEFFICIENT: 2\r\n");
		break;
	case BMP280_FILTER_4:
		printf("\r\nFILTER COEFFICIENT: 4\r\n");
		break;
	case BMP280_FILTER_8:
		printf("\r\nFILTER COEFFICIENT: 8\r\n");
		break;
	case BMP280_FILTER_16:
		printf("\r\nFILTER COEFFICIENT: 16\r\n");
		break;
	default:
		printf("\r\nFilter code not recognized!\r\n");
		break;
	}
	//mode
	switch(params->mode)
	{
		case BMP280_MODE_SLEEP:
			printf("MODE: SLEEP\r\n");
			break;
		case BMP280_MODE_FORCED:
			printf("MODE: FORCED\r\n");
			break;
		case BMP280_MODE_NORMAL:
			printf("MODE: NORMAL\r\n");
			break;
		default:
			printf("Mode code not recognized!\r\n");
			break;
	}
	//oversampling
	BMP280_Oversampling* oversampling = &params->oversampling_pressure;
	const char* text[3] = {"PRESSURE", "TEMPERATURE", "HUMIDITY"};
	for(short i = 0; i <3; i++)
	{

		switch(*oversampling)
		{
			case BMP280_SKIPPED:
				printf("%s OVERSAMPLING: SKIPPED\r\n", text[i]);
				break;
			case BMP280_ULTRA_LOW_POWER:
				printf("%s OVERSAMPLING: ULTRA LOW POWER\r\n", text[i]);
				break;
			case BMP280_LOW_POWER:
				printf("%s OVERSAMPLING: LOW POWER\r\n", text[i]);
				break;
			case BMP280_STANDARD:
				printf("%s OVERSAMPLING: STANDARD\r\n", text[i]);
				break;
			case BMP280_HIGH_RES:
				printf("%s OVERSAMPLING: HIGH RESOLUTION\r\n", text[i]);
				break;
			case BMP280_ULTRA_HIGH_RES:
				printf("%s OVERSAMPLING: ULTRA HIGH RESOLUTION\r\n", text[i]);
				break;
			default:
				printf("Oversampling code not recognized!\r\n");
				break;
		}
		if(i <2)
			++oversampling;
	}

	//standby

	switch(params->standby)
	{
	case BMP280_STANDBY_05:
		printf("STANDBY TIME 0.5 ms\r\n");
		break;
	case BMP280_STANDBY_62:
		printf("STANDBY TIME 62 ms\r\n");
		break;
	case BMP280_STANDBY_125:
		printf("STANDBY TIME 125 ms\r\n");
		break;
	case BMP280_STANDBY_250:
		printf("STANDBY TIME 250 ms\r\n");
		break;
	case BMP280_STANDBY_500:
		printf("STANDBY TIME 500 ms\r\n");
		break;
	case BMP280_STANDBY_1000:
		printf("STANDBY TIME 1s\r\n");
		break;
	case BMP280_STANDBY_2000:
		printf("STANDBY TIME 2s\r\n");
		break;
	case BMP280_STANDBY_4000:
		printf("STANDBY TIME 4s\r\n");
		break;
	default:
		printf("Standby code not recognized\r\n");
		break;

	}
	printf("\r\n");
}


void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef* handler)
{
	if(handler == &hrtc)
	{
		//printf("Woke up\r\n");

	}
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
