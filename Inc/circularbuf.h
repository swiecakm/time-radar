

#ifndef _circularbuf
#define _circularbuf

#include "main.h"
#include "stdbool.h"
#include "stdlib.h"

typedef struct
{
	int8_t *commands_buff;
	int8_t head ;
	int8_t tail;
	int8_t size;
} circular_buffer_t;

circular_buffer_t* circular_buf_initialize(void);

int circular_buf_empty(circular_buffer_t*);

int circular_buf_get(circular_buffer_t*);

void circular_buf_put(circular_buffer_t*, int8_t);

#endif
