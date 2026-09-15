/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "motor.h"

void drv_set_microsteps(drv_t *drv, drv_microstep_t microsteps){
    drv->microsteps = microsteps;
    HAL_GPIO_WritePin(drv->m0_port, drv->m0_pin, (microsteps & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m1_port, drv->m1_pin, (microsteps & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m2_port, drv->m2_pin, (microsteps & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void drv_init(drv_t *drv, drv_microstep_t microsteps, GPIO_TypeDef *m0_port, uint16_t m0_pin, GPIO_TypeDef *m1_port, uint16_t m1_pin, GPIO_TypeDef *m2_port, uint16_t m2_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_TypeDef *rst_slp_port, uint16_t rst_slp_pin){
    drv->m0_port = m0_port;
    drv->m0_pin = m0_pin;
    drv->m1_port = m1_port;
    drv->m1_pin = m1_pin;
    drv->m2_port = m2_port;
    drv->m2_pin = m2_pin;
    drv_set_microsteps(drv, microsteps);
    drv->dir_port = dir_port;
    drv->dir_pin = dir_pin;
    drv->rst_slp_port = rst_slp_port;
    drv->rst_slp_pin = rst_slp_pin;
}

void drv_set_direction(drv_t *drv, direction_t dir){
    drv->dir = dir;
    HAL_GPIO_WritePin(drv->dir_port, drv->dir_pin, (dir == DIRECTION_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *htim, uint32_t tim_channel, uint32_t fmax, uint32_t fclk,uint32_t fs){
    motor->steps_per_rev = steps_per_rev;
    motor->htim_PWM = htim;
    motor->tim_channel = tim_channel;
    motor->fmax = fmax;
    motor->fclk = fclk;
    motor->fs = fs;
}

void motor_drv_init(motor_drv_t *motor_drv, drv_t *drv, motor_t *motor){
    motor_drv->drv = drv;
    motor_drv->motor = motor;
    motor_drv->fcurrent = 0;
    motor_drv->pwm_on = 0;
    motor_drv->state = MOTOR_DRV_STOPPED;
}

void motor_acc(float acc, motor_drv_t *motor_drv){ //acc è in step/s^2
    uint32_t arr;
    uint32_t ccr;
    
    TIM_HandleTypeDef *htim = motor_drv->motor->htim_PWM;
    uint32_t psc  = htim->Instance->PSC;
    uint32_t ch = motor_drv->motor->tim_channel;
    
    float fmax = motor_drv->motor->fmax;
    float fclk = motor_drv->motor->fclk;
    float fs = motor_drv->motor->fs;
    float fmin = FMIN(fclk, psc);

    float v = motor_drv->fcurrent + acc/fs;
    if (v >=  fmax) v = fmax;
    if (v <= -fmax) v = -fmax;

    // direzione dal segno della VELOCITÀ (non di acc)
    if      (v > 0) drv_set_direction(motor_drv->drv, DIRECTION_CW);
    else if (v < 0) drv_set_direction(motor_drv->drv, DIRECTION_CCW);
    // se v == 0 lascio l'ultima direzione, tanto sto per fermarmi
    
    motor_drv->fcurrent = v;
    float F = fabsf(v);

    // sotto fmin: fermo l'uscita, niente calcolo ARR (evita /0)
    if (F <= fmin) {
        motor_drv->state = MOTOR_DRV_STOPPED;
        HAL_TIM_PWM_Stop(htim, ch);
        motor_drv->pwm_on = 0;
        return;
    }

    motor_drv->state = (acc >= 0) ? MOTOR_DRV_ACCEL : MOTOR_DRV_DECEL;
    arr = (uint32_t)(fclk / ((psc + 1) * F) - 1.0f);
    if (arr > 0xFFFF) arr = 0xFFFF;
    ccr = arr / 2;
    
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, ch, ccr);
    htim->Instance->EGR = TIM_EGR_UG;

    if (motor_drv->pwm_on == 0) {
        HAL_TIM_PWM_Start(htim, ch);
    }
    motor_drv->pwm_on = 1;
}