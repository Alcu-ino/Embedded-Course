#include "buffer.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define OK 0
#define OK_PIENO 1

Buffer* initialize(size_t capacity){
    Buffer* buffer = (Buffer*) malloc(sizeof(Buffer));  
    buffer -> capacity = capacity;
    buffer -> data = NULL;
    buffer -> size = 0;
    return buffer;
}

uint8_t add(Buffer* buffer, int32_t value){
    while(buffer->data == NULL){
        buffer->data = (int32_t*) malloc(buffer->capacity * sizeof(int32_t));
    }
    if (buffer->size == buffer->capacity-1 ){
        return OK_PIENO;
    }
    buffer->data[buffer->size] = value;
    buffer->size++;
    return OK;
}

uint8_t reset(Buffer* buffer){
    buffer -> data = NULL;
    buffer -> size = 0;
    return 0;
}