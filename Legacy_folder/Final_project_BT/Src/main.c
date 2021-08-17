/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iks01a2_motion_sensors.h"
#include "iks01a2_env_sensors.h" //i added
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE 128
#define INPUT_BUFFER_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char initMessage[MAX_BUF_SIZE] = ">Accelerometer && env sensors initialization: completed \r\n";
char enablingMessage[MAX_BUF_SIZE] = ">Accelerometer && env sensors enabling: completed\r\n";

char startMessage[MAX_BUF_SIZE];
char parameterMessage[MAX_BUF_SIZE];
char outputData[MAX_BUF_SIZE];
char dataOut[MAX_BUF_SIZE];
char letterMessage[MAX_BUF_SIZE];

int start_acquisition= 1;

int data_counter=0;
int sample_n = 150;

float accelerometer_odr;
float accelerometer_sens;
int32_t accelerometer_fs;

float temperature_odr; //i added
float temperature_gv;//i added
float pressure_gv;
float humidity_gv;

IKS01A2_MOTION_SENSOR_Axes_t acceleration;
IKS01A2_ENV_SENSOR_Capabilities_t cap; //i added

ai_float in_data[AI_NETWORK_IN_1_SIZE];
ai_float out_data[AI_NETWORK_OUT_1_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  // Accelerometer initialization && env sensor init
    int init_acc = IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0 , MOTION_ACCELERO );
    int init_temperature = IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE);
    int init_humidity = IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_HUMIDITY);
    int init_pressure = IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);
    if ( init_acc == 0 && init_temperature ==0 && init_humidity ==0 && init_pressure ==0 ){
    	  if(HAL_UART_Transmit(&huart2, (uint8_t*)initMessage, MAX_BUF_SIZE, 1000) != HAL_OK){Error_Handler();}
    	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY){};
      }//if

    // Accelerometer enabling
    int enable_acc = IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO );
    int enable_temperature = IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
    int enable_humidity = IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY);
    int enable_pressure = IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);

    if ( enable_acc == 0 && enable_temperature == 0 && enable_humidity == 0 && enable_pressure == 0){
    	  if(HAL_UART_Transmit(&huart2, (uint8_t*)enablingMessage, MAX_BUF_SIZE, 1000) != HAL_OK){Error_Handler();}
    	  while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY){};
      }//if

    IKS01A2_MOTION_SENSOR_GetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &accelerometer_odr);
    IKS01A2_MOTION_SENSOR_GetSensitivity(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &accelerometer_sens);
    IKS01A2_MOTION_SENSOR_GetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &accelerometer_fs);

    IKS01A2_ENV_SENSOR_GetOutputDataRate(IKS01A2_HTS221_0, ENV_TEMPERATURE, &temperature_odr);
    IKS01A2_ENV_SENSOR_GetValue(IKS01A2_HTS221_0, ENV_TEMPERATURE, &temperature_gv);
    IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LPS22HB_0, ENV_PRESSURE, &pressure_gv);
    IKS01A2_ENV_SENSOR_GetValue(IKS01A2_HTS221_0, ENV_HUMIDITY, &humidity_gv);

    snprintf(parameterMessage, MAX_BUF_SIZE, "> Output rate:%f Hz	Sensitivity:%f g FullScale:%ld g\r\n TEMP is %.3f, HUMIDITY %.3f%%, PRESSURE %.3f mBar",accelerometer_odr, accelerometer_sens, accelerometer_fs,temperature_gv, humidity_gv, pressure_gv);
    if(HAL_UART_Transmit(&huart2, (uint8_t*)parameterMessage, MAX_BUF_SIZE, 1000) != HAL_OK){Error_Handler();}

    //Start timer
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 100;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA6
                           PA7 PA8 PA9 PA10
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//uint16_t get_ADC()
//{
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1000);
//	uint16_t value = HAL_ADC_GetValue(&hadc1);
//
//	HAL_ADC_Stop(&hadc1);
//
//	return value;
//}

//float ConvertTemp(uint16_t D_ADC)
//{
//	float Vref = 3.3, V25 = 0.76, Avg_Slope = 0.0025;
//	float V_ADC = D_ADC * (Vref / 4095.0);
//	float temp = ((V_ADC - V25)/Avg_Slope) + 25;
//
//	return temp;
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	start_acquisition=1;
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim){
	if(data_counter == sample_n){
		data_counter=0;
		start_acquisition=0;
		aiRun(&in_data, &out_data);
		snprintf(outputData, MAX_BUF_SIZE, "%.3f %.3f %.3f %.3f probabilities\r\n", out_data[0], out_data[1], out_data[2], out_data[3]);
		if(HAL_UART_Transmit(&huart2, (uint8_t*)outputData, MAX_BUF_SIZE, 1000) != HAL_OK){Error_Handler();}
		while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY){}
		int position=0;
		float maxvalue = out_data[0];
		for(int ind=1; ind<5; ind++){
			if(out_data[ind]>maxvalue){
				maxvalue = out_data[ind];
				position = ind;
			}
		}
		switch(position){
			case(0):
				snprintf(letterMessage, MAX_BUF_SIZE, ">Move is: Block \r\n");//aeiou
				break;
			case(1):
				snprintf(letterMessage, MAX_BUF_SIZE, ">Move is: Side punch \r\n");
				break;
			case(2):
				snprintf(letterMessage, MAX_BUF_SIZE, ">Move is: Upper cut \r\n");
				break;
			case(3):
				snprintf(letterMessage, MAX_BUF_SIZE, ">Move is: Dodge \r\n");
				break;
			default:
				snprintf(letterMessage, MAX_BUF_SIZE, ">Not enough sure what move you did \r\n");
				break;

				}

		if(HAL_UART_Transmit(&huart2, (uint8_t*)letterMessage, MAX_BUF_SIZE, 1000) != HAL_OK){Error_Handler();}
		while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY){}
		memset(letterMessage, 0, sizeof(letterMessage));
		memset(outputData, 0, sizeof(outputData));


	}else if(start_acquisition == 1){
//		float converted_temp = ConvertTemp(get_ADC()); //my part
		data_counter ++;
		IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &acceleration);
		IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LSM6DSL_0, ENV_TEMPERATURE, &temperature_gv);
		IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LSM6DSL_0, ENV_HUMIDITY, &humidity_gv);
		IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LPS22HB_0, ENV_PRESSURE, &pressure_gv);

		snprintf(dataOut, MAX_BUF_SIZE, "%d: %ld,%ld,%ld,%ld,%ld,%ld\r\n",data_counter,(int32_t)acceleration.x, (int32_t)acceleration.y, (int32_t)acceleration.z,
				(int32_t) temperature_gv, (int32_t) humidity_gv, (int32_t) pressure_gv);
		HAL_UART_Transmit(&huart2, (uint8_t*)dataOut, MAX_BUF_SIZE, 1000);
		in_data[data_counter] = acceleration.x;
		in_data[data_counter+sample_n] =  acceleration.y;
		in_data[data_counter+2*sample_n] = acceleration.z;
		//in_data[data_counter+2*sample_n] = temperature_gv;
		//in_data[data_counter+2*sample_n] = humidity_gv;
		//in_data[data_counter+2*sample_n] = pressure_gv;
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
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
