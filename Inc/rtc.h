#ifndef _rtc
#define _rtc

#include "stm32f0xx_hal.h"

typedef struct
{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Day;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
} RTC_DateTime_t;

RTC_DateTime_t * RTC_InitializeDateTime(I2C_HandleTypeDef *);
void RTC_IncrementMinutes(I2C_HandleTypeDef *);
void RTC_IncrementHours(I2C_HandleTypeDef *);
void RTC_IncrementYear(I2C_HandleTypeDef *);
void RTC_IncrementMonth(I2C_HandleTypeDef *);
void RTC_IncrementDate(I2C_HandleTypeDef *);
void RTC_RefreshDateTime(I2C_HandleTypeDef *, RTC_DateTime_t *);
void RTC_SetDefaultTime(I2C_HandleTypeDef *);

#endif