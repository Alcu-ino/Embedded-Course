#include <stdio.h>
#include <stdint.h>

typedef struct{
    int32_t* data;
    size_t size;
    size_t capacity;
} Buffer;

Buffer* initialize(size_t capacity);
uint8_t add(Buffer* buffer, int32_t value);
uint8_t reset(Buffer* buffer);