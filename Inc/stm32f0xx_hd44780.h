#ifndef _HD44780
#define _HD44780
void HD44780_Initialize(void);
	
void HD44780_InitializeTimer(void);

void HD44780_SendMessage(unsigned char[]);

void HD44780_Clear(void);

void HD44780_GoToSecondLine(void);

void HD44780_GoToFirstLine(void);
#endif
