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

//other used commands
#define HD44780_COMMAND_GO_TO_SECOND_LINE 192


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

GPIO_TypeDef *HD44780_OUTPORTS[8] = {
	HD44780_D0_GPIO_Port,
	HD44780_D1_GPIO_Port,
	HD44780_D2_GPIO_Port,
	HD44780_D3_GPIO_Port,
	HD44780_D4_GPIO_Port,
	HD44780_D5_GPIO_Port,
	HD44780_D6_GPIO_Port,
	HD44780_D7_GPIO_Port
};

const int HD44780_COMMAND_DELAY = 100;
const uint8_t HD44780_COMMANDS_BUFF_SIZE = (uint8_t)128;
const uint8_t HD44780_CHARACTERS_BUFF_SIZE = (uint8_t)128;

circular_buffer_t *commands_buffer;
circular_buffer_t *characters_buffer;

TIM_HandleTypeDef TIM_HandleInitStruct;

void TIM16_IRQHandler(void)
{
 HAL_TIM_IRQHandler(&TIM_HandleInitStruct);
}

void HD44780_Initialize(void)
{
	commands_buffer = circular_buf_initialize(HD44780_COMMANDS_BUFF_SIZE);
	characters_buffer = circular_buf_initialize(HD44780_CHARACTERS_BUFF_SIZE);
	
	circular_buf_put(commands_buffer, HD44780_COMMAND_CLEAR);
	circular_buf_put(commands_buffer, HD44780_COMMAND_RETURN_HOME);
	circular_buf_put(commands_buffer, HD44780_COMMAND_INCR_DDRAM_AND_NO_SHIFT);
	circular_buf_put(commands_buffer, HD44780_COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
	circular_buf_put(commands_buffer, HD44780_COMMAND_8BIT_TWO_LINES_5x8);
		
	HD44780_InitializeTimer();
}

void HD44780_Clear(void)
{
	circular_buf_put(commands_buffer, HD44780_COMMAND_CLEAR);
}

void HD44780_GoToFirstLine(void)
{
	circular_buf_put(commands_buffer, HD44780_COMMAND_RETURN_HOME);
}

void HD44780_GoToSecondLine(void)
{
	circular_buf_put(commands_buffer, HD44780_COMMAND_GO_TO_SECOND_LINE);
}

void HD44780_SendMessage(unsigned char message[])
{
	for(int i=0; i<strlen((char*)message); i++) {
		circular_buf_put(characters_buffer, (uint8_t)message[i]);
	}
}

enum BusStates 
{
	LOW_STATE,
	HIGH_STATE
};

enum BusStates eState = HIGH_STATE;

void HD44780_SetDataFromBufferOnLCDPins(circular_buffer_t *buff)
{
	uint8_t data = circular_buf_get(buff);	
	for (int i=0; i<8; i++) 
	{
		uint8_t value = (1 & (data >> i));
		HAL_GPIO_WritePin(HD44780_OUTPORTS[i], HD44780_OUTPINS[i], value);
	}
}

void HD44780_SetDataOnLCDPinsIfAvailable()
{
	if (!circular_buf_empty(commands_buffer))
	{
		//send LCD command in priority
		HAL_GPIO_WritePin(HD44780_RS_GPIO_Port, HD44780_RS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HD44780_E_GPIO_Port, HD44780_E_Pin, GPIO_PIN_SET);
		HD44780_SetDataFromBufferOnLCDPins(commands_buffer);
	}
	else if (!circular_buf_empty(characters_buffer))
	{
		//if no commands send characters
		HAL_GPIO_WritePin(HD44780_RS_GPIO_Port, HD44780_RS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(HD44780_E_GPIO_Port, HD44780_E_Pin, GPIO_PIN_SET);
		HD44780_SetDataFromBufferOnLCDPins(characters_buffer);
	}
}

void HD44780_WriteDataToLCD()
{
	HAL_GPIO_WritePin(GPIOC, HD44780_E_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (eState == LOW_STATE)
	{
		HD44780_SetDataOnLCDPinsIfAvailable();
		eState = HIGH_STATE;
	}
	else if (eState == HIGH_STATE)
	{
		HD44780_WriteDataToLCD();
		eState = LOW_STATE;
	}
}

void HD44780_InitializeTimer(void)
{
	__HAL_RCC_TIM16_CLK_ENABLE();
	
	//1 ms period
	TIM_HandleInitStruct.Instance = TIM16;
	TIM_HandleInitStruct.Init.Prescaler = 4800 - 1;
	TIM_HandleInitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_HandleInitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM_HandleInitStruct.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM_HandleInitStruct.Init.Period = 10 - 1;
	
	HAL_TIM_Base_Init(&TIM_HandleInitStruct);
	__HAL_TIM_CLEAR_FLAG(&TIM_HandleInitStruct, TIM_SR_UIF);
	
	HAL_NVIC_EnableIRQ(TIM16_IRQn);
	HAL_TIM_Base_Start_IT(&TIM_HandleInitStruct);
}

