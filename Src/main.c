/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265359
#define R 6371008.8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t rx_data[700];

char latitude_degrees[2];
char longitude_degrees[3];
char latitude_minutes[8];
char longitude_minutes[8];
char tell_time[9];

float Lat_in_DD,Long_in_DD,utc_time;

int x;
float  y;

float Lat0,Long0;
float Lat1,Long1;
float Lat2,Long2;

double lat0,lat1,lat2;
double long0,long1,long2;
double dx10,dy10,a10,c10,d10;
double dx20,dy20,a20,c20,d20;
double dx21,dy21,a21,c21,d21;
double Ctheta,angle;
double ax,ay,bx,by,cz;

double final_angle;
char buffer[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive(&huart5,rx_data,sizeof rx_data,500);
	printf("%s \n\r",rx_data);
}
void float_to_char(){
	int ret = snprintf(buffer, sizeof buffer, "%f", final_angle);
}
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
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// NEO6-MV2 GPS UART collect data recived at port UART5
		HAL_UART_Receive_IT(&huart5,rx_data,sizeof rx_data);
		//to find index of required data in NMEA format
		for(int i=0;i<1200;i++){
			if (rx_data[i] == '$'){
				if(rx_data[i+1] == 'G'){
					if(rx_data[i+2] == 'P' || rx_data[i+2] =='N'){
						if(rx_data[i+3] == 'G'){
							if(rx_data[i+4] == 'L'){
								if(rx_data[i+5] == 'L'){
									if(rx_data[i+44] == 'A'){									
										//LAT
										//LAT DEGREE COMPONENT
										latitude_degrees[0] = rx_data[i+7];
										latitude_degrees[1] = rx_data[i+8];
										//LAT DECIMAL MINUTE COMPONENT
										latitude_minutes[0] = rx_data[i+9];
										latitude_minutes[1] = rx_data[i+10];									
										latitude_minutes[2] = rx_data[i+11];
										latitude_minutes[3] = rx_data[i+12];
										latitude_minutes[4] = rx_data[i+13];
										latitude_minutes[5] = rx_data[i+14];
										latitude_minutes[6] = rx_data[i+15];
										latitude_minutes[7] = rx_data[i+16];										
										//LONG									
										//LONG DEGREE COMPONENT
										longitude_degrees[0] = rx_data[i+20];
										longitude_degrees[1] = rx_data[i+21];
										longitude_degrees[2] = rx_data[i+22];
										//LONG DECIMAL MINUTE COMPONENT
										longitude_minutes[0] = rx_data[i+23];
										longitude_minutes[1] = rx_data[i+24];									
										longitude_minutes[2] = rx_data[i+25];
										longitude_minutes[3] = rx_data[i+26];
										longitude_minutes[4] = rx_data[i+27];
										longitude_minutes[5] = rx_data[i+28];
										longitude_minutes[6] = rx_data[i+29];
										longitude_minutes[7] = rx_data[i+30];										
										//TIME
										tell_time[0] = rx_data[i+34];
										tell_time[1] = rx_data[i+35];
										tell_time[2] = rx_data[i+36];
										tell_time[3] = rx_data[i+37];
										tell_time[4] = rx_data[i+38];
										tell_time[5] = rx_data[i+39];
										tell_time[6] = rx_data[i+40];
										tell_time[7] = rx_data[i+41];
										tell_time[8] = rx_data[i+42];
									}else{
										printf("invalid data \n\r");
										break;
									}								   
								}
							}
						}
					}
				}
			}
		}		
		
    // Char array to floats
		x = 17;//atof(latitude_degrees);
		y = atof(latitude_minutes);
		Lat_in_DD = x+(y/60);
		//Lat_in_DD = atof(latitude_degrees)+(atof(latitude_minutes)/60);
		Long_in_DD = atof(longitude_degrees)+(atof(longitude_minutes)/60);
		utc_time = atof(tell_time);

    // Buttons & data processing
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET){
      // Button 1 to set location of stepper
			Lat0 = Lat_in_DD;
			Long0 = Long_in_DD;
		}
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_SET){
      // Button 2 to set location of initial line
			Lat1 = Lat_in_DD;
			Long1 = Long_in_DD;
		} 
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_SET){
      // Button 3 to set location of second line 
			Lat2 = Lat_in_DD;
			Long2 = Long_in_DD;
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_SET){
      // Button 4 to set new line/s and trasnmit data
			//old line
			Lat1 = Lat2;
			Long1 = Long2;
			//new line
			Lat2 = Lat_in_DD;
			Long2 = Long_in_DD;

      /*a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
      c = 2 ⋅ atan2( √a, √(1−a) )
      d = R ⋅ c*/

      lat0 = Lat0 * (PI / 180); long0 = Long0 * (PI / 180);
      lat1 = Lat1 * (PI / 180); long1 = Long1 * (PI / 180);
      lat2 = Lat2 * (PI / 180); long2 = Long2 * (PI / 180);
      
      dx10 = long1 - long0;
      dy10 = lat1 - lat0;

      a10 = pow(sin(dy10 / 2), 2) + cos(lat0) * cos(lat1) * pow(sin(dx10 / 2), 2);
      c10 = 2 * atan2(sqrt(a10), sqrt(1 - a10));
      d10 = R * c10;      

      dx20 = long2 - long0;
      dy20 = lat2 - lat0;

      a20 = pow(sin(dy20 / 2), 2) + cos(lat0) * cos(lat2) * pow(sin(dx20 / 2), 2);
      c20 = 2 * atan2(sqrt(a20), sqrt(1 - a20));
      d20 = R * c20; // distance between new line and stepper;

      dx21 = long2 - long1;
      dy21 = lat2 - lat1;

      a21 = pow(sin(dy21 / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dx21 / 2), 2);
      c21 = 2 * atan2(sqrt(a21), sqrt(1 - a21));
      d21 = R * c21;
      //d21 = a, d10 = b, d20 = c
      Ctheta = (pow(d10, 2) + pow(d20, 2) - pow(d21, 2)) / (2 * d10 * d20);
      angle = acos(Ctheta);
      angle = angle * (180 / PI);
      //printf("angle = %f\n", angle);

      ax = Long1 - Long0;
      ay = Lat1 - Lat0;
      bx = Long2 - Long0;
      by = Lat2 - Lat0;
      cz = (ax * by) - (ay * bx);
      //printf("%.17g \n", cz); 17d 9f

      if (cz < 0)
      {
        final_angle = (-1)*angle;
      }
      else if (cz > 0)
      {
        final_angle = angle;
      }
			
      /*diff_m = (mslope2 - mslope1) / (1 + (mslope1 * mslope2));
			angle_rad = atan(diff_m);
			final_angle = (atan(angle_rad))*(180/PI);*/
			//float to char*
			//char* ftoa(double final_angle,char* test_tx);
			float_to_char();			
			//UART-4 FOR BLUETOOTH
			HAL_UART_Transmit(&huart4,buffer,sizeof buffer,100);

		}
		HAL_Delay(400);		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
