#include "stm32f0xx_hd44780.h"
#include "main.h"
#include "..\Drivers/STM32F0xx_HAL_Driver\Inc\stm32f0xx_hal_tim.h"
#include "string.h"
#include "stdlib.h"
#include "circularbuf.h"

//#define BUFF_SIZE 128

//Initial commands
#define HD44780_COMMAND_CLEAR 1
#define HD44780_COMMAND_RETURN_HOME 2
#define HD44780_COMMAND_INCR_DDRAM_AND_NO_SHIFT 6
#define HD44780_COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF 12
#define HD44780_COMMAND_DISPLAY_MOVE_DISPLAY 16
#define HD44780_COMMAND_8BIT_TWO_LINES_5x8 56


uint16_t HD44780_OUTPINS[8] = {
	HD44780_D0_Pin,
	HD44780_D1_Pin,
	HD44780_D2_Pin,
	HD44780_D3_Pin,
	HD44780_D4_Pin,
	HD44780_D5_Pin,
	HD44780_D6_Pin,
	HD44780_D7_Pin
};

const int HD44780_COMMAND_DELAY = 100;
const uint8_t HD44780_COMMANDS_BUFF_SIZE = (uint8_t)128;

circular_buffer_t *commands_buffer;

TIM_HandleTypeDef TIM_HandleInitStruct;

void TIM16_IRQHandler(void)
{
 HAL_TIM_IRQHandler(&TIM_HandleInitStruct);
}

void HD44780_Initialize(void)
{
	commands_buffer = circular_buf_initialize(HD44780_COMMANDS_BUFF_SIZE);
	HAL_GPIO_WritePin(GPIOC, HD44780_RS_Pin, GPIO_PIN_RESET);
	
	HD44780_SendCommand(HD44780_COMMAND_CLEAR);
	for(int i=0; i<8000; i++) {}

	HD44780_SendCommand(HD44780_COMMAND_RETURN_HOME);
	for(int i=0; i<8000; i++) {}
	
	HD44780_SendCommand(HD44780_COMMAND_INCR_DDRAM_AND_NO_SHIFT);
	for(int i=0; i<8000; i++) {}
	
	HD44780_SendCommand(HD44780_COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	for(int i=0; i<8000; i++) {}
	
	HD44780_SendCommand(HD44780_COMMAND_8BIT_TWO_LINES_5x8);
	for(int i=0; i<8000; i++) {}
	
	HAL_GPIO_WritePin(GPIOC, HD44780_RS_Pin, GPIO_PIN_SET);
	for(int i=0; i<8000; i++) {}
	
	HD44780_InitializeTimer();
}

void HD44780_SendMessage(char message[])
{
	for(int i=0; i<strlen(message); i++) {
		circular_buf_put(commands_buffer, message[i]);
	}
}

void HD44780_SendCommand(int data)
{
	HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_SET);
	for (int i=0; i<8; i++) 
	{
		int value = (1 & (data >> i));
		HAL_GPIO_WritePin(GPIOC, HD44780_OUTPINS[i], value);
	}
	HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_RESET);
}

enum BusStates 
{
	LOW_STATE,
	HIGH_STATE
};

enum BusStates eState = HIGH_STATE;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (eState == LOW_STATE && !circular_buf_empty(commands_buffer))
	{
		HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_SET);
		eState = HIGH_STATE;
		int data = circular_buf_get(commands_buffer);	
		for (int i=0; i<8; i++) 
		{
			int value = (1 & (data >> i));
						
			HAL_GPIO_WritePin(GPIOC, HD44780_OUTPINS[i], value);
		}
	}
	else if (eState == HIGH_STATE)
	{
		HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_RESET);
		eState = LOW_STATE;
	}
}

void HD44780_InitializeTimer(void)
{
	__HAL_RCC_TIM16_CLK_ENABLE();
	
	//10 ms period
	TIM_HandleInitStruct.Instance = TIM16;
	TIM_HandleInitStruct.Init.Prescaler = 48000 - 1;
	TIM_HandleInitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_HandleInitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_HandleInitStruct.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM_HandleInitStruct.Init.Period = 10 - 1;
	
	HAL_TIM_Base_Init(&TIM_HandleInitStruct);
	__HAL_TIM_CLEAR_FLAG(&TIM_HandleInitStruct, TIM_SR_UIF);
	
	HAL_NVIC_EnableIRQ(TIM16_IRQn);
	HAL_TIM_Base_Start_IT(&TIM_HandleInitStruct);
}

