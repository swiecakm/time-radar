

#ifndef _circularbuf
#define _circularbuf

#include "main.h"
#include "stdbool.h"

void circular_buf_initialize(void);

int circular_buf_empty(void);

int circular_buf_get(void);

void circular_buf_put(int8_t data);

#endif
