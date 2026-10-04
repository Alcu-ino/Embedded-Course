/*
 * lqr.c
 *
 *  Created on: 4 oct 2026
 *      Author: vito
 */
// deve essere integrativo motor_drv->fcurrent * m_per_step*dt !!
//state x = [pos, vel, angle, ang_vel] -> [motor_drv->position*m_per_step, motor_drv->fcurrent * m_per_step, encoder->rad_angle, encoder->w]
// dt  = 1.0f/ motor_drv->motor->fs
#include "lqr.h"

void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc){
    float a_m_s2 = 0;
    float K[4] = {-10.0000, -38.2160, -235.6668, -69.6459};
    float dt = 1.0f/ motor_drv->motor->fs;
    float x[4] = {motor_drv->position*m_per_step, motor_drv->fcurrent * m_per_step, encoder->rad_angle - (float)M_PI, encoder->w};
    for(int i = 0; i < 4; i++){
        a_m_s2 -= K[i] * x[i];
    }
    *acc = a_m_s2/m_per_step;
}

