/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "stdint.h"

#define Tc 1/(84*pow(10,6))
#define RES 4
#define CPR 600

typedef struct {
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
