#ifndef _HD44780
#define _HD44780
void HD44780_Initialize(void);
	
void HD44780_SendCommand(int);

void HD44780_InitializeTimer(void);

void HD44780_SendMessage(char[]);
#endif
