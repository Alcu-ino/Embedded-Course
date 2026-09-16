/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "encoder.h"
#include "main.h"
#include "math.h"

void reset_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr, uint8_t res, float t){
    encoder->ts = t;
    encoder->cpr = cpr;
    encoder->res = res;
    encoder->angle = 0;
    encoder->prev_angle = 0;
    encoder->w = 0;
    encoder->a = 0;
};

void init_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr, uint8_t res, float t){
    encoder->timcount_array[0]=0;
    encoder->timcount_array[1]=0;
    encoder->ts = t;
    encoder->cpr = cpr;
    encoder->res = res;
    encoder->angle = 0;
    encoder->prev_angle = 0;
    encoder->w = 0;
    encoder->a = 0;
};

void update_Encoder(Encoder_HandleTypeDef *encoder){
    if (DMA2_Stream5->NDTR) 
    {
        encoder->angle   = encoder->timcount_array[0];
        encoder->prev_angle = encoder->timcount_array[1];
    } 
    else 
    {
        encoder->angle   = encoder->timcount_array[1];
        encoder->prev_angle = encoder->timcount_array[0];
    }
    encoder->angle = (float)TIM1->CNT * 360 / (float)(encoder->cpr * encoder->res);
    volatile float delta= encoder->angle - encoder->prev_angle;
    volatile float delta_angle = atan2f(sinf(delta), cosf(delta));
    encoder->w = delta_angle / encoder->ts;
    encoder->a = 0;
};
