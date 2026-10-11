/*
 * encoder.h
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "stdint.h"

#define RES 4
#define CPR 600
#define DMA_BUFFER_SIZE 2

typedef struct {
    volatile uint16_t timcount_array[DMA_BUFFER_SIZE];
    float ts;
    uint32_t cpr;
    uint8_t res;
    volatile float rad_angle;
    volatile float angle;
    volatile float prev_angle;
    volatile float w;
} Encoder_HandleTypeDef;

void reset_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void init_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void update_Encoder(Encoder_HandleTypeDef *);
#endif
