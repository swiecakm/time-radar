#include "stm32f0xx_hd44780.h"
#include "main.h"
#include "..\Drivers/STM32F0xx_HAL_Driver\Inc\stm32f0xx_hal_tim.h"
#include "string.h"

#define BUFF_SIZE 128

//Initial commands
#define HD44780_COMMAND_CLEAR 1
#define HD44780_COMMAND_RETURN_HOME 2
#define HD44780_COMMAND_INCR_DDRAM_AND_NO_SHIFT 6
#define HD44780_COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF 12
#define HD44780_COMMAND_DISPLAY_MOVE_DISPLAY 16
#define HD44780_COMMAND_8BIT_TWO_LINES_5x8 56

int8_t commands_buff[BUFF_SIZE];
int8_t *head = &commands_buff[0];
int8_t *tail = &commands_buff[0];

void HD44780_PutOnBuffer(int value)
{
	*head = value;
	if(head < &commands_buff[0] + BUFF_SIZE) 
	{
		head++;
	}
	else
	{
		head = &commands_buff[0];
	}
}

int HD44780_GetFromBuffer(void)
{
	int value = *tail;
	if(tail < &commands_buff[0] + BUFF_SIZE) 
	{
		tail++;
	}
	else
	{
		tail = &commands_buff[0];
	}
	return value;
}

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

TIM_HandleTypeDef TIM_HandleInitStruct;

void TIM16_IRQHandler(void)
{
 HAL_TIM_IRQHandler(&TIM_HandleInitStruct);
}

void HD44780_Initialize(void)
{
	HAL_GPIO_WritePin(GPIOC, HD44780_RS_Pin, GPIO_PIN_RESET);
	
	HD44780_SendCommand(HD44780_COMMAND_CLEAR);
	HAL_Delay(HD44780_COMMAND_DELAY);

	HD44780_SendCommand(HD44780_COMMAND_RETURN_HOME);
	HAL_Delay(HD44780_COMMAND_DELAY);
	
	HD44780_SendCommand(HD44780_COMMAND_INCR_DDRAM_AND_NO_SHIFT);
	HAL_Delay(HD44780_COMMAND_DELAY);
	
	HD44780_SendCommand(HD44780_COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	HAL_Delay(HD44780_COMMAND_DELAY);
	
	HD44780_SendCommand(HD44780_COMMAND_8BIT_TWO_LINES_5x8);
	HAL_Delay(HD44780_COMMAND_DELAY);
	
	HAL_GPIO_WritePin(GPIOC, HD44780_RS_Pin, GPIO_PIN_SET);
	HAL_Delay(HD44780_COMMAND_DELAY);
	
	HD44780_InitializeTimer();
}

void HD44780_SendMessage(char message[])
{
	for(int i=0; i<strlen(message); i++) {
	  HD44780_PutOnBuffer(message[i]);
	}
}

void HD44780_SendCommand(int data)
{
	HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_SET);
	//HAL_Delay(HD44780_COMMAND_DELAY);
	for (int i=0; i<1000; i++) 
	{
	}
	for (int i=0; i<8; i++) 
	{
		int value = (1 & (data >> i));
		HAL_GPIO_WritePin(GPIOC, HD44780_OUTPINS[i], value);
	}
	//HAL_Delay(HD44780_COMMAND_DELAY);
		for (int i=0; i<1000; i++) 
	{
	}
	HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_RESET);
	//HAL_Delay(HD44780_COMMAND_DELAY);
		for (int i=0; i<1000; i++) 
	{
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(tail != head)
	{
		while(tail < head)
		{
			int value = HD44780_GetFromBuffer();
			HD44780_SendCommand(value);
		}
	}
}

void HD44780_InitializeTimer(void)
{
	__HAL_RCC_TIM16_CLK_ENABLE();
	
	
	TIM_HandleInitStruct.Instance = TIM16;
	TIM_HandleInitStruct.Init.Prescaler = 48000 - 1;
	TIM_HandleInitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_HandleInitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_HandleInitStruct.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM_HandleInitStruct.Init.Period = 2000 - 1;
	
	HAL_TIM_Base_Init(&TIM_HandleInitStruct);
	__HAL_TIM_CLEAR_FLAG(&TIM_HandleInitStruct, TIM_SR_UIF);
	
	HAL_NVIC_EnableIRQ(TIM16_IRQn);
	HAL_TIM_Base_Start_IT(&TIM_HandleInitStruct);
}

