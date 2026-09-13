/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "stdint.h"

#define Ts 0.001
#define RES 4
#define CPR 600

typedef struct {
    float ts;
    uint32_t cpr;
    uint8_t res;
    float angle;
    float prev_angle;
    float w;
    float a;
} Encoder_HandleTypeDef;

void reset_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void init_Encoder(Encoder_HandleTypeDef *, uint32_t, uint8_t, float );
void update_Encoder(Encoder_HandleTypeDef *);
