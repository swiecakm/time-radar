

#ifndef _circularbuf
#define _circularbuf

#include "main.h"
#include "stdbool.h"
#include "stdlib.h"

typedef struct
{
	int8_t *buff_array;
	uint8_t head ;
	uint8_t tail;
	uint8_t size;
} circular_buffer_t;

circular_buffer_t* circular_buf_initialize(uint8_t);

int circular_buf_empty(circular_buffer_t*);

int circular_buf_get(circular_buffer_t*);

void circular_buf_put(circular_buffer_t*, int8_t);

#endif
