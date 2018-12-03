
#ifndef _HD44780
#define _HD44780
extern const int HD44780_COMMAND_DELAY;

void HD44780_Initialize(void);
	
void HD44780_SendCommand(int);
#endif
