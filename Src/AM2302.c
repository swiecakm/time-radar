#include "main.h"

int AM2302_BitsReceivedCount = 0;
int AM2302_RequestSent = 0;
int AM2302_BitsReceived[40];
uint64_t AM2302_ReceivedData;

void AM2302_SendRequest()
{
	AM2302_ReceivedData = 0;
	AM2302_RequestSent = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		int signalLength = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
		if(AM2302_RequestSent)
		{
			AM2302_BitsReceivedCount = 0;
			AM2302_RequestSent = 0;
		}
		else
		{
			AM2302_BitsReceivedCount++;
			//first two falling edges is from initialization of transmission
			if (AM2302_BitsReceivedCount > 2 && AM2302_BitsReceivedCount <= 42)
			{
				uint8_t bitValue;
				if(signalLength < 90)
				{
					bitValue = 0;
				}
				else
				{
					bitValue = 1;
				}
				//higher bit first
				uint8_t bitPosition = 42 - AM2302_BitsReceivedCount;
				uint64_t mask = 1 << bitPosition; 
				AM2302_ReceivedData = (AM2302_ReceivedData & ~mask) | ((bitValue << bitPosition) & mask); 
			}
		}
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	}
}

uint16_t AM2302_GetHumidity()
{
	return (0xFFFF & (AM2302_ReceivedData >> 24));
}

uint16_t AM2302_GetTemperature()
{
	return (0xFFFF & (AM2302_ReceivedData >> 8));
}

uint8_t AM2302_ChecksumCorrect()
{
	
	uint16_t humidity = AM2302_GetHumidity();
	uint16_t temperature = AM2302_GetTemperature();
	
	uint16_t checksum = 0xFF & AM2302_ReceivedData;
	uint16_t calculatedchecksum = 0xFF &(
			(0xFF & (uint16_t)humidity) + 
			(0xFF & ((uint16_t)humidity >> 8)) + 
			(0xFF & (uint16_t)temperature) + 
			(0xFF & ((uint16_t)temperature >> 8))
	);
	
	return checksum == calculatedchecksum;
}