/*
 * encoder.c
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
    encoder->rad_angle = 0;
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
    encoder->rad_angle = 0;
};

void update_Encoder(Encoder_HandleTypeDef *encoder)
{
    uint32_t n;
    uint16_t c0, c1;

    do {
        n  = DMA1_Stream6->NDTR;
        c0 = encoder->timcount_array[0];
        c1 = encoder->timcount_array[1];
    } while (n != DMA1_Stream6->NDTR);

    uint16_t nuovo = (n == 1u) ? c0 : c1;     /* NDTR==1: ultimo scritto [0] */

    /* l'angolo usato nel controllo precedente diventa il vecchio */
    encoder->prev_angle = encoder->angle;

    float k = 360.0f / (float)(encoder->cpr * encoder->res);
    encoder->angle     = (float)nuovo * k;
    encoder->rad_angle = encoder->angle * (float)M_PI / 180.0f;

    /* velocita' tra il campione di questo controllo e quello del precedente */
    float delta = (encoder->angle - encoder->prev_angle) * (float)M_PI / 180.0f;
    delta = atan2f(sinf(delta), cosf(delta));
    encoder->w = delta / encoder->ts;
};