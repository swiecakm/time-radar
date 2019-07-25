/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hd44780.h"
#include "rtc.h"
#include "stdlib.h"
#include "AM2302.h"
#include "rtc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum SetTimePositions 
{
	MINUTES = 0,
	HOURS = 1,
	YEAR = 2,
	MONTH = 3,
	DAY = 4,
	NONE = 5
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
unsigned char helloMessage[] = "Hello!";
unsigned char versionMessage[] = "Clock v1.0";

RTC_DateTime_t *dateTime;

int B1_pushed = 0;
int B1_PushedTime = 0;
int B1_LastPushedTime = 0;

uint8_t UART_received= 0;

//temperature and humidity sensor variables


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void UpdateDateTimeMessage(RTC_DateTime_t *, unsigned char*);
int GetArrowPosition(enum SetTimePositions);
void IncrementDateTime(enum SetTimePositions position);
void UpdateTemperatureMessage(unsigned char*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UpdateDateTimeMessage(RTC_DateTime_t *dateTime, unsigned char *dateTimeMessage)
{
	dateTimeMessage[0] = (uint8_t)(0xF & (dateTime->Date >> 4)) + '0';
	dateTimeMessage[1] = (uint8_t)(0xF & dateTime->Date) + '0';
	
	dateTimeMessage[3] = (uint8_t)(0xF & (dateTime->Month >> 4)) + '0';
	dateTimeMessage[4] = (uint8_t)(0xF & dateTime->Month) + '0';
	
	dateTimeMessage[6] = '2';
	dateTimeMessage[7] = '0';
	
	dateTimeMessage[8] = (uint8_t)(0xF & (dateTime->Year >> 4)) + '0';
	dateTimeMessage[9] = (uint8_t)(0xF & dateTime->Year) + '0';
	
	dateTimeMessage[11] = (uint8_t)(0xF & (dateTime->Hours >> 4)) + '0';
	dateTimeMessage[12] = (uint8_t)(0xF &  dateTime->Hours) + '0';
	
	dateTimeMessage[14] = (uint8_t)(0xF & (dateTime->Minutes >> 4)) + '0';
	dateTimeMessage[15] = (uint8_t)(0xF &  dateTime->Minutes) + '0';
}

void UpdateTemperatureMessage(unsigned char *message)
{
	uint16_t temperature = AM2302_GetTemperature();
	message[5] = 'C';
	message[4] = (uint8_t)223;
	message[3] = (uint8_t)(temperature % 10) + '0';
	message[2] = ',';
	message[1] = (uint8_t)((temperature % 100) / 10) + '0';
	message[0] = (uint8_t)((temperature % 1000) / 100) + '0';
	
	uint16_t humidity = AM2302_GetHumidity();
	message[15] = '%';
	message[14] = (uint8_t)(humidity % 10) + '0';
	message[13] = ',';
	message[12] = (uint8_t)((humidity % 100) / 10) + '0';
	message[11] = (uint8_t)((humidity % 1000) / 100) + '0';
	if ((uint8_t)((humidity % 10000) / 1000) == 0)
	{
		message[10] = ' ';
	}
	else
	{
		message[10] = (uint8_t)((humidity % 10000) / 1000) + '0';
	}
	
	if ((uint8_t)((humidity % 100000) / 10000) == 0)
	{
		message[9] = ' ';
	}
	else
	{
		message[9] = (uint8_t)((humidity % 100000) / 10000) + '0';
	}
}

int GetArrowPosition(enum SetTimePositions position)
{
		int arrowPosition = 0;
		switch (position)
		{
			case MINUTES:
				arrowPosition = 15; break;
			case HOURS:
				arrowPosition = 12; break;
			case YEAR:
				arrowPosition = 9; break;
			case MONTH:
				arrowPosition = 4; break;
			case DAY:
				arrowPosition = 1; break;
			default:
				arrowPosition = -1; break;
		}
		return arrowPosition;
}

void IncrementDateTime(enum SetTimePositions position)
{
		switch (position)
		{
			case MINUTES:
				RTC_IncrementMinutes(&hi2c1); break;
			case HOURS:
				RTC_IncrementHours(&hi2c1); break;
			case YEAR:
				RTC_IncrementYear(&hi2c1); break;
			case MONTH:
				RTC_IncrementMonth(&hi2c1); break;
			case DAY:
				RTC_IncrementDate(&hi2c1); break;
			default:
				break;
		}
}

void SetButtonPushed(void)
{
	B1_pushed = 1;
}

void SetButtonNotPushed(void)
{
	B1_pushed = 0;
}

void IncrementB1PushedTime(void)
{
	B1_PushedTime ++;
}

void ResetB1PushedTime(void)
{
	//longer than 100 ms
	if (B1_PushedTime > 1)
	{
		B1_LastPushedTime = B1_PushedTime;
	}
	B1_PushedTime = 0;
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
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HD44780_Initialize();
	
	/*HD44780_SendCommand(72);
	HAL_Delay(100);*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HD44780_SendMessage(helloMessage);
	HAL_Delay(1000);
	HD44780_GoToSecondLine();
	HD44780_SendMessage(versionMessage);
	HAL_Delay(2000);
	HD44780_GoToFirstLine();
		
	uint8_t prevMinutes = -1;
	unsigned char dateTimeMessage[] = "  .  .       :  ";
	unsigned char secondLineMessage[] = "                ";
	
	enum SetTimePositions currentPosition = NONE;
	
	HAL_NVIC_EnableIRQ(TIM14_IRQn);
	HAL_TIM_Base_Start_IT(&htim14);
	
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	
	dateTime =  RTC_InitializeDateTime(&hi2c1);
	
	HAL_UART_Receive_IT(&huart1, &UART_received, 1);
		
	while (1)
  {
		RTC_RefreshDateTime(&hi2c1, dateTime);
		int arrowPosition = -1;
		
		if (dateTime->Minutes != prevMinutes || B1_pushed || B1_LastPushedTime > 0)
		{
			AM2302_SendRequest();
			
			//If button hold for over 2s
			if (B1_pushed && B1_PushedTime > 20)
			{
				IncrementDateTime(currentPosition);
				RTC_RefreshDateTime(&hi2c1, dateTime);
			}
			prevMinutes = dateTime->Minutes;
			UpdateDateTimeMessage(dateTime, dateTimeMessage);			
			HD44780_Clear();
			HD44780_SendMessage(dateTimeMessage);
			HAL_Delay(100);
			HD44780_GoToSecondLine();
			//0.1s - 1s
			if(B1_LastPushedTime > 1 && B1_LastPushedTime < 10)
			{
				if(currentPosition == NONE)
				{
					currentPosition = MINUTES;
				}
				else
				{
					currentPosition++;
				}
				arrowPosition = GetArrowPosition(currentPosition); 

				for (int i=0; i<strlen((const char*)secondLineMessage); i++)
				{
					if (i != arrowPosition)
					{
						secondLineMessage[i] = ' ';
					}
					else
					{
						secondLineMessage[i] = '^';
					}			
				}		
			}
			else if (currentPosition == NONE)
			{
				UpdateTemperatureMessage(secondLineMessage);
			}
			if(B1_LastPushedTime > 0)
			{
				B1_LastPushedTime = 0;
			}
			
			HD44780_SendMessage(secondLineMessage);
			HAL_Delay(100);
			HD44780_GoToFirstLine();
		}

				
		HAL_Delay(800);
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
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
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 1;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, HD44780_RS_Pin|HD44780_E_Pin|HD44780_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HD44780_D0_Pin|HD44780_D1_Pin|HD44780_D2_Pin|HD44780_D3_Pin 
                          |BLUE_LED_Pin|GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HD44780_D4_Pin|HD44780_D5_Pin|HD44780_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AM2302_DATA_GPIO_Port, AM2302_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HD44780_RS_Pin HD44780_E_Pin HD44780_D7_Pin */
  GPIO_InitStruct.Pin = HD44780_RS_Pin|HD44780_E_Pin|HD44780_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : HD44780_D0_Pin HD44780_D1_Pin HD44780_D2_Pin HD44780_D3_Pin 
                           BLUE_LED_Pin GREEN_LED_Pin */
  GPIO_InitStruct.Pin = HD44780_D0_Pin|HD44780_D1_Pin|HD44780_D2_Pin|HD44780_D3_Pin 
                          |BLUE_LED_Pin|GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HD44780_D4_Pin HD44780_D5_Pin HD44780_D6_Pin */
  GPIO_InitStruct.Pin = HD44780_D4_Pin|HD44780_D5_Pin|HD44780_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AM2302_DATA_Pin */
  GPIO_InitStruct.Pin = AM2302_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AM2302_DATA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
