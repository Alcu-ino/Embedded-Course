/*
 * motor.h
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "main.h"
#include "math.h"

#define ARRMAX 0xFFFFFFFFUL
#define FMIN(fclk,psc) ((float)(fclk) / (((float)(psc) + 1.0f) * ((float)ARRMAX)))

typedef enum {
    DIRECTION_CW=0,
    DIRECTION_CCW=1
} direction_t;

typedef struct {
    uint16_t steps_per_rev;
    TIM_HandleTypeDef *htim_PWM; /* STEP timer */
    uint32_t fclk;
    uint32_t fmax;
    uint32_t fs;
    uint32_t tim_channel; /* Timer channel */
} motor_t;


typedef enum {
    DRV8825_FULL_STEP=1,
    DRV8825_HALF_STEP=2,
    DRV8825_4_STEPS=4,
    DRV8825_8_STEPS=8,
    DRV8825_16_STEPS=16,
    DRV8825_32_STEPS=32
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
    direction_t dir;

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
    drv_t *drv; /* Pointer to the driver */
    motor_t *motor; /* Pointer to motor */
    motion_state_t state;
    int32_t fcurrent;/* Motion state */
    uint8_t pwm_on; /* PWM is running [boolean] */
} motor_drv_t;

void drv_init(drv_t *drv, drv_microstep_t microsteps, GPIO_TypeDef *m0_port, uint16_t m0_pin, GPIO_TypeDef *m1_port, uint16_t m1_pin, GPIO_TypeDef *m2_port, uint16_t m2_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_TypeDef *rst_slp_port, uint16_t rst_slp_pin);
void drv_set_microsteps(drv_t *drv, drv_microstep_t microsteps);
void drv_set_direction(drv_t *drv, direction_t dir);

void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *htim, uint32_t tim_channel, uint32_t fmax, uint32_t fclk,uint32_t fs);
void motor_drv_init(motor_drv_t *motor_drv, drv_t *drv, motor_t *motor);
void motor_acc(float acc, motor_drv_t *motor_drv);
