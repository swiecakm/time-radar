#ifndef _rtc
#define _rtc

#include "stm32f0xx_hal.h"

void RTC_IncrementMinutes(RTC_HandleTypeDef*, RTC_TimeTypeDef*);
void RTC_IncrementHours(RTC_HandleTypeDef*, RTC_TimeTypeDef*);
void RTC_IncrementYear(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void RTC_IncrementMonth(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void RTC_IncrementDay(RTC_HandleTypeDef*, RTC_DateTypeDef*);
void RTC_RefreshDateTime(RTC_HandleTypeDef*, RTC_DateTypeDef*, RTC_TimeTypeDef*);
void RTC_SetDefaultTime(RTC_HandleTypeDef *, RTC_TimeTypeDef *);

#endif