#include "rtc.h"
#include "stm32f0xx_hal.h"

#define DS1307_DEVICE_ADDRESS (0x68 << 1)
#define DS1307_SECONDS_ADDRESS 0x00
#define DS1307_MINUTES_ADDRESS 0x01
#define DS1307_HOURS_ADDRESS 0x02
#define DS1307_DAY_ADDRESS 0x03
#define DS1307_DATE_ADDRESS 0x04
#define DS1307_MONTH_ADDRESS 0x05
#define DS1307_YEAR_ADDRESS 0x06

uint8_t RTC_IncrementBDCValue(uint8_t, uint8_t);

//only for numbers < 100
uint8_t RTC_IncrementBDCValue(uint8_t value, uint8_t max)
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

void RTC_IncrementMinutes(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime)
{
	sTime->Minutes = RTC_IncrementBDCValue(sTime->Minutes, 0x60);
	HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BCD);
}

void RTC_IncrementHours(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime)
{
	sTime->Hours = RTC_IncrementBDCValue(sTime->Hours, 0x24);
	HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BCD);
}

void RTC_IncrementYear(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Year = RTC_IncrementBDCValue(sDate->Year, 0x99);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void RTC_IncrementMonth(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Month = RTC_IncrementBDCValue(sDate->Month, 0x12);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void RTC_IncrementDay(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate)
{
	sDate->Date = RTC_IncrementBDCValue(sDate->Date, 0x31);
	HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void RTC_RefreshDateTime(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, RTC_TimeTypeDef *sTime)
{
	HAL_RTC_WaitForSynchro(hrtc);
	HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BCD);
}

void RTC_SetDefaultTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime)
{
	sTime->Hours = (2<<4) + 2;
  sTime->Minutes = (2<<4) + 2;; 
  sTime->Seconds = 0;
	HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BCD);
}