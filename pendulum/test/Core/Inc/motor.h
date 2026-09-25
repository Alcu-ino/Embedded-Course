/*
 * motor.h
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <math.h>
#include <stdint.h>

/* TIM2 (e TIM5) su STM32F4 hanno contatore a 32 bit.
   Con un timer a 16 bit usare 0xFFFFUL (fmin sale a fclk/65536). */
#define ARRMAX 0xFFFFFFFFUL
#define FMIN(fclk,psc) ((float)(fclk) / (((float)(psc) + 1.0f) * ((float)ARRMAX)))

#define MOTOR_FMIN_PRATICA  5.0f     /* [Hz] sotto questa frequenza STEP ci si ferma      */
#define STEP_PULSE_S        2.5e-6f  /* [s]  durata impulso STEP alto (DRV8825 >= 1.9 us) */

typedef enum {
    DIRECTION_CW  = 0,
    DIRECTION_CCW = 1
} direction_t;

typedef struct {
    uint16_t steps_per_rev;
    TIM_HandleTypeDef *htim_PWM;  /* STEP timer */
    float fclk;                   /* [Hz] clock del timer (prima del prescaler) */
    float fmax;                   /* [Hz] frequenza STEP massima */
    float fs;                     /* [Hz] frequenza di chiamata di motor_acc */
    uint32_t tim_channel;         /* Timer channel */
} motor_t;

typedef enum {
    DRV8825_FULL_STEP = 1,
    DRV8825_HALF_STEP = 2,
    DRV8825_4_STEPS   = 4,
    DRV8825_8_STEPS   = 8,
    DRV8825_16_STEPS  = 16,
    DRV8825_32_STEPS  = 32
} drv_microstep_t;

typedef struct {
    drv_microstep_t microsteps;
    GPIO_TypeDef *m0_port;
    uint16_t m0_pin;

    GPIO_TypeDef *m1_port;
    uint16_t m1_pin;

    GPIO_TypeDef *m2_port;
    uint16_t m2_pin;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    direction_t dir;              /* sempre allineato al livello reale del pin DIR */

    GPIO_TypeDef *rst_slp_port;
    uint16_t rst_slp_pin;
} drv_t;

typedef enum {
    MOTOR_DRV_STOPPED,
    MOTOR_DRV_ACCEL,
    MOTOR_DRV_RUN,
    MOTOR_DRV_DECEL
} motion_state_t;

typedef struct {
    drv_t *drv;                   /* Pointer to the driver */
    motor_t *motor;               /* Pointer to motor */
    motion_state_t state;         /* Motion state */
    float fcurrent;               /* [step/s] velocita' con segno */
    uint8_t pwm_on;               /* PWM is running [boolean] */
} motor_drv_t;

void drv_init(drv_t *drv, drv_microstep_t microsteps,
              GPIO_TypeDef *m0_port, uint16_t m0_pin,
              GPIO_TypeDef *m1_port, uint16_t m1_pin,
              GPIO_TypeDef *m2_port, uint16_t m2_pin,
              GPIO_TypeDef *dir_port, uint16_t dir_pin,
              GPIO_TypeDef *rst_slp_port, uint16_t rst_slp_pin);
void drv_set_microsteps(drv_t *drv, drv_microstep_t microsteps);
void drv_set_direction(drv_t *drv, direction_t dir);

void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *htim,
                uint32_t tim_channel, float fmax, float fclk, float fs);
void motor_drv_init(motor_drv_t *motor_drv, drv_t *drv, motor_t *motor);
void motor_acc(float acc, motor_drv_t *motor_drv);

#endif /* MOTOR_H */