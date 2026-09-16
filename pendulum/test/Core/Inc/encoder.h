/*
 * encoder.h
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "stdint.h"

#define Tc 840/(84*pow(10,6))
#define RES 4
#define CPR 600
#define DMA_BUFFER_SIZE 2

typedef struct {
    uint16_t timcount_array[DMA_BUFFER_SIZE];
    float ts;
    uint32_t cpr;
    uint8_t res;
    volatile float angle;
    volatile float prev_angle;
    volatile float w;
    volatile float a;
} Encoder_HandleTypeDef;

void reset_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void init_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void update_Encoder(Encoder_HandleTypeDef *);
