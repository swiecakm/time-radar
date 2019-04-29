#ifndef _HD44780
#define _HD44780
void HD44780_Initialize(void);
	
void HD44780_SendCommand(int);

void InitializeTimer(void);

void PutOnBuffer(int);

int GetFromBuffer(void);
#endif
