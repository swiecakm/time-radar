#include "circularbuf.h"
#define BUFF_SIZE 127


circular_buffer_t* circular_buf_initialize_new(void);

circular_buffer_t* circular_buf_initialize()
{
	circular_buffer_t *commands_buf = (circular_buffer_t *) malloc(sizeof(circular_buffer_t));
	commands_buf->commands_buff = (int8_t *) malloc(sizeof(int8_t)*BUFF_SIZE);
	commands_buf->head = 0;
	commands_buf->tail = 0;
	commands_buf->size = BUFF_SIZE;
	return commands_buf;
}

void circular_buf_put(circular_buffer_t* commands_buf, int8_t value)
{
	commands_buf->commands_buff[commands_buf->head] = value;
	if(commands_buf->head < commands_buf->size) 
	{
		commands_buf->head++;
	}
	else
	{
		commands_buf->head = 0;
	}
}

int circular_buf_get(circular_buffer_t* commands_buf)
{
	int value = commands_buf->commands_buff[commands_buf->tail];
	if(commands_buf->tail < commands_buf->size) 
	{
		commands_buf->tail++;
	}
	else
	{
		commands_buf->tail = 0;
	}
	return value;
}

int circular_buf_empty(circular_buffer_t* commands_buf)
{
	return commands_buf->head == commands_buf->tail;
}
