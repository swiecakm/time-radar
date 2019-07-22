#ifndef _AM2302
#define _AM2302

#include "stm32f0xx_hal.h"

void AM2302_SendRequest();
uint16_t AM2302_GetHumidity();
uint16_t AM2302_GetTemperature();
#endif


	