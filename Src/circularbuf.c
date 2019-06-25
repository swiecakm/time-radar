#include "circularbuf.h"
#define BUFF_SIZE 127

int8_t commands_buff[BUFF_SIZE];
int8_t head = 0;
int8_t tail = 0;
int8_t size = BUFF_SIZE;

void circular_buf_put(int8_t value)
{
	commands_buff[head] = value;
	if(head < size) 
	{
		head++;
	}
	else
	{
		head = 0;
	}
}

int circular_buf_get(void)
{
	int value = commands_buff[tail];
	if(tail < size) 
	{
		tail++;
	}
	else
	{
		tail = 0;
	}
	return value;
}

int circular_buf_empty(void)
{
	return head == tail;
}
