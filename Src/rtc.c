#include "rtc.h"
#include "stm32f0xx_hal.h"

uint8_t IncrementBDCValue(uint8_t, uint8_t);

//only for numbers < 100
uint8_t IncrementBDCValue(uint8_t value, uint8_t max)
{
	value = value + 1;
	//repair invalid bcd state
	if ((0xF & value) > 9)
	{
		//ommit all invalid states 10 - 15
		value = value + 6;
	}
	if (value > max)
	{
		value = 0x0;
	}
	return value;
}

void IncrementMinutes(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime)
{
	sTime->Minutes = IncrementBDCValue(sTime->Minutes, 0x60);
	HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BCD);
}

void IncrementHours(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime)
{
	sTime->Hours = IncrementBDCValue(sTime->Hours, 0x24);
	HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BCD);
}

void IncrementYear(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Year = IncrementBDCValue(sDate->Year, 0x99);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void IncrementMonth(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Month = IncrementBDCValue(sDate->Month, 0x12);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void IncrementDay(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Date = IncrementBDCValue(sDate->Date, 0x31);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void UpdateDateTime(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, RTC_TimeTypeDef *sTime)
{
	HAL_RTC_WaitForSynchro(hrtc);
	HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BCD);
}