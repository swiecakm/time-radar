#ifndef _rtc
#define _rtc

#include "stm32f0xx_hal.h"

void IncrementMinutes(RTC_HandleTypeDef*, RTC_TimeTypeDef*);
void IncrementHours(RTC_HandleTypeDef*, RTC_TimeTypeDef*);
void IncrementYear(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void IncrementMonth(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void IncrementDay(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void UpdateDateTime(RTC_HandleTypeDef*, RTC_DateTypeDef*, RTC_TimeTypeDef*);

#endif