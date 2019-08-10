#ifndef _AM2302
#define _AM2302

#include "stm32f0xx_hal.h"

void AM2302_SendRequest(void);
uint16_t AM2302_GetHumidity(void);
uint16_t AM2302_GetTemperature(void);
uint8_t AM2302_ChecksumCorrect(void);
#endif


	