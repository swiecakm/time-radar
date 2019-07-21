#include "circularbuf.h"

circular_buffer_t* circular_buf_initialize(uint8_t buff_size)
{
	circular_buffer_t *buff = (circular_buffer_t *) malloc(sizeof(circular_buffer_t));
	buff->buff_array = (uint8_t *) malloc(sizeof(uint8_t)*buff_size);
	buff->head = 0;
	buff->tail = 0;
	buff->size = buff_size;
	return buff;
}

void circular_buf_put(circular_buffer_t* buff, uint8_t value)
{
	buff->buff_array[buff->head] = value;
	if(buff->head < buff->size) 
	{
		buff->head++;
	}
	else
	{
		buff->head = 0;
	}
}

uint8_t circular_buf_get(circular_buffer_t* buff)
{
	uint8_t value = buff->buff_array[buff->tail];
	if(buff->tail < buff->size) 
	{
		buff->tail++;
	}
	else
	{
		buff->tail = 0;
	}
	return value;
}

int circular_buf_empty(circular_buffer_t* buff)
{
	return buff->head == buff->tail;
}
