/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
 #include "encoder.h"

void drv_set_microsteps(drv_t *drv, drv_microstep_t microsteps){
    drv->microsteps = microsteps;
    HAL_GPIO_WritePin(drv->m0_port, drv->m0_pin, (microsteps & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m1_port, drv->m1_pin, (microsteps & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m2_port, drv->m2_pin, (microsteps & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void drv_init(drv_t *drv, drv_microstep_t microsteps, GPIO_TypeDef *m0_port, uint16_t m0_pin, GPIO_TypeDef *m1_port, uint16_t m1_pin, GPIO_TypeDef *m2_port, uint16_t m2_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_TypeDef *rst_slp_port, uint16_t rst_slp_pin){
    drv_set_microsteps(drv, microsteps);
    drv->m0_port = m0_port;
    drv->m0_pin = m0_pin;
    drv->m1_port = m1_port;
    drv->m1_pin = m1_pin;
    drv->m2_port = m2_port;
    drv->m2_pin = m2_pin;
    drv->dir_port = dir_port;
    drv->dir_pin = dir_pin;
    drv->rst_slp_port = rst_slp_port;
    drv->rst_slp_pin = rst_slp_pin;
}

void drv_set_direction(drv_t *drv, direction_t dir){
    drv->dir = dir;
    HAL_GPIO_WritePin(drv->dir_port, drv->dir_pin, (dir == DIRECTION_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *step_htim, uint32_t step_tim_clk, uint32_t step_tim_channel, uint32_t fmax){
    motor->steps_per_rev = steps_per_rev;
    motor->fmax = fmax;
    motor->step_htim = step_htim;
    motor->step_tim_clk = step_tim_clk;
    motor->step_tim_channel = step_tim_channel;
}
void motor_drv_init(motor_drv_t *motor_drv, drv_t *drv, motor_t *motor){
    motor_drv->drv = drv;
    motor_drv->motor = motor;
    motor_drv->fcurrent = 0;
    motor_drv->state = MOTOR_DRV_STOPPED;
    motor_drv->pwm_on = 0;
}

void motor_acc(float acc){
    float v = motor_drv->fcurrent + acc * Tc;
    float fmax = motor_drv->motor->fmax;

    if (v >=  fmax)? v = fmax: v;
    if (v <= -fmax) v = -fmax: v;

    // direzione dal segno della VELOCITÀ (non di acc)
    if      (v > 0) drv_set_direction(motor_drv->drv, DIRECTION_CW);
    else if (v < 0) drv_set_direction(motor_drv->drv, DIRECTION_CCW);
    // se v == 0 lascio l'ultima direzione, tanto sto per fermarmi
    motor_drv->fcurrent = v;
    float fmag = fabsf(v);

    // sotto fmin: fermo l'uscita, niente calcolo ARR (evita /0)
    if (fmag < FMIN) {
        motor_drv->state = MOTOR_DRV_STOPPED;
        __HAL_TIM_DISABLE(&HTIM_STEP);   // oppure HAL_TIM_PWM_Stop(...)
        return;
    }

    motor_drv->state = (acc >= 0) ? MOTOR_DRV_ACCEL : MOTOR_DRV_DECEL; // solo bookkeeping

    // f_step = 84e6 / ((PSC+1)(ARR+1))  ->  ARR = 84e6/((PSC+1)*fmag) - 1
    uint32_t period = (uint32_t)((TIMER_CLK / ((float)(PSC + 1) * fmag)) + 0.5f);
    uint32_t arr = period - 1U;

    __HAL_TIM_SET_AUTORELOAD(&HTIM_STEP, arr);
    __HAL_TIM_SET_COMPARE(&HTIM_STEP, STEP_CHANNEL, arr / 2U); // duty 50%, o larghezza fissa ~2µs
    // se il timer era fermo, riavvialo qui
}

void motor_drv_update(motor_drv_t *motor_drv){};//pwm on off, forse questa funzione non serve
//RICORDATI DI CHIARIRE A COSA SERVE LO SCOPO DEL TIMER SLAVE
//VEDI BENE LE FUNZIONI DEI TIMER