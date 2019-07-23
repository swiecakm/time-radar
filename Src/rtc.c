#include "rtc.h"
#include "stm32f0xx_hal.h"
#include "stdlib.h"

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

RTC_DateTime_t * RTC_InitializeDateTime(I2C_HandleTypeDef *hi2c)
{
	RTC_DateTime_t *dateTime = (RTC_DateTime_t *) malloc(sizeof(RTC_DateTime_t));
	RTC_RefreshDateTime(hi2c, dateTime);
	return dateTime;
}

void RTC_IncrementMinutes(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = 0x00;
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MINUTES_ADDRESS, 1, &value, 1, 100);
	value = RTC_IncrementBDCValue(value, 0x60);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MINUTES_ADDRESS, 1, &value, 1, 100);
}

void RTC_IncrementHours(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = 0x00;
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_HOURS_ADDRESS, 1, &value, 1, 100);
	value = RTC_IncrementBDCValue(value, 0x24);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_HOURS_ADDRESS, 1, &value, 1, 100);
}

void RTC_IncrementYear(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = 0x00;
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_YEAR_ADDRESS, 1, &value, 1, 100);
	value = RTC_IncrementBDCValue(value, 0x99);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_YEAR_ADDRESS, 1, &value, 1, 100);
}

void RTC_IncrementMonth(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = 0x00;
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MONTH_ADDRESS, 1, &value, 1, 100);
	value = RTC_IncrementBDCValue(value, 0x12);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MONTH_ADDRESS, 1, &value, 1, 100);
}

void RTC_IncrementDate(I2C_HandleTypeDef *hi2c)
{
	uint8_t value = 0x00;
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DATE_ADDRESS, 1, &value, 1, 100);
	value = RTC_IncrementBDCValue(value, 0x31);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DATE_ADDRESS, 1, &value, 1, 100);
}

void RTC_RefreshDateTime(I2C_HandleTypeDef *hi2c, RTC_DateTime_t *dateTime)
{
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_SECONDS_ADDRESS, 1, &dateTime->Seconds, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MINUTES_ADDRESS, 1, &dateTime->Minutes, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_HOURS_ADDRESS, 1, &dateTime->Hours, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DAY_ADDRESS, 1, &dateTime->Day, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DATE_ADDRESS, 1, &dateTime->Date, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MONTH_ADDRESS, 1, &dateTime->Month, 1, 100);
	HAL_I2C_Mem_Read(hi2c, DS1307_DEVICE_ADDRESS, DS1307_YEAR_ADDRESS, 1, &dateTime->Year, 1, 100);
}

void RTC_SetDefaultTime(I2C_HandleTypeDef *hi2c)
{
	uint8_t zeroVal = 0x00, oneVal = 0x01, yearVal = 0x19;
	
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_SECONDS_ADDRESS, 1, &zeroVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MINUTES_ADDRESS, 1, &oneVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_HOURS_ADDRESS, 1, &zeroVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DAY_ADDRESS, 1, &oneVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_DATE_ADDRESS, 1, &oneVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_MONTH_ADDRESS, 1, &oneVal, 1, 100);
	HAL_I2C_Mem_Write(hi2c, DS1307_DEVICE_ADDRESS, DS1307_YEAR_ADDRESS, 1, &yearVal, 1, 100);
}