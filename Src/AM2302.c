#include "main.h"

int AM2302_BitsReceivedCount = 0;
int AM2302_RequestSent = 0;
uint16_t AM2302_BitsReceivedLengths[45];
uint64_t AM2302_ReceivedData;

void AM2302_SendRequest()
{
	AM2302_ReceivedData = 0;
	for(int i=0; i<45; i++)
	{
		AM2302_BitsReceivedLengths[i] = 0;
 	}
	AM2302_RequestSent = 1;
	//mimimal star signal length is 1ms
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}


void AM2302_TranslateSignalLengths()
{
	uint64_t bitValue;
	//first two falling edges in signal are:
	//1 - edge from host pulldown to request data from sensor
	//2 - edge from sensor pulldown to signalize transmission start
	for(int i=2; i<42; i++)
	{
		//for bit 0: 50us low + 26us high = 76us
		//for bit 1: 50us low + 70us high = 120us
		//AM2302_BitsReceivedLengths is in us
		if(AM2302_BitsReceivedLengths[i] < 100)
		{
			bitValue = 0;
		}
		else
		{
			bitValue = 1;
		}
		
		uint64_t bitPosition = 41 - i;
		uint64_t mask = ((uint64_t)1) << bitPosition; 
		//i do not check checksum but there is function available for this
		AM2302_ReceivedData = (AM2302_ReceivedData & ~mask) | ((bitValue << bitPosition) & mask);
	}
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
			AM2302_BitsReceivedLengths[AM2302_BitsReceivedCount] = signalLength;
			AM2302_BitsReceivedCount++;
		}
		
		if(AM2302_BitsReceivedCount == 42)
		{
			//i do this at last bit received to keep this callback light during transmission
			AM2302_TranslateSignalLengths();
		}
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	}
}

uint16_t AM2302_GetHumidity()
{
	uint64_t mask = 0xFFFF;
	//to get humidity divide this value by 10
	return (mask & (AM2302_ReceivedData >> 24));
}

uint16_t AM2302_GetTemperature()
{
	uint64_t mask = 0xFFFF;
	//the sign is the highest bit
	//to get temperatrue divide this value by 10
	return (mask & (AM2302_ReceivedData >> 8));
}

uint8_t AM2302_ChecksumCorrect()
{
	uint64_t mask = 0xFF;
	uint64_t humidity = AM2302_GetHumidity();
	uint64_t temperature = AM2302_GetTemperature();
	
	uint64_t checksum = mask & AM2302_ReceivedData;
	uint64_t calculatedchecksum = mask &(
			(mask & humidity) + 
			(mask & (humidity >> (uint64_t)8)) + 
			(mask & temperature) + 
			(mask & (temperature >> (uint64_t)8))
	);
	
	return checksum == calculatedchecksum;
}