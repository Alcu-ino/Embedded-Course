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

#define MOTOR_FMIN_PRATICA  1.0f     /* [Hz] sotto questa frequenza STEP ci si ferma      */
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
    float fs;                     /* [Hz] fixed motor_acc update rate */
    float amax;                   /* [step/s^2] absolute acceleration limit */
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

/* The physical mapping and levels must be measured on the real machine. */
#define MOTOR_LIMIT_A 1u
#define MOTOR_LIMIT_B 2u

typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_CONFIG,
    MOTOR_FAULT_COMMAND,
    MOTOR_FAULT_TIMER,
    MOTOR_FAULT_BOTH_LIMITS,
    MOTOR_FAULT_UNEXPECTED_LIMIT,
    MOTOR_FAULT_RELEASE,
    MOTOR_FAULT_TIMEOUT,
    MOTOR_FAULT_TRAVEL,
    MOTOR_FAULT_LIMIT,
    MOTOR_FAULT_CANCELLED
} motor_fault_t;

typedef struct {
    GPIO_PinState a_pressed_level;
    GPIO_PinState b_pressed_level;
    uint8_t cw_limit;              /* MOTOR_LIMIT_A or MOTOR_LIMIT_B */
    float speed;                  /* search/traverse speed [step/s] */
    float latch_speed;            /* final switch approach [step/s] */
    float acceleration;           /* [step/s^2] */
    uint32_t debounce_ms;
    uint32_t backoff_steps;        /* clearance after stable release */
    uint32_t max_release_steps;
    uint32_t min_travel_steps;
    uint32_t max_travel_steps;
    uint32_t release_timeout_ms;
    uint32_t phase_timeout_ms;
    uint32_t total_timeout_ms;
} motor_homing_config_t;

typedef struct {
    drv_t *drv;
    motor_t *motor;
    motion_state_t state;
    float fcurrent;               /* commanded signed velocity [step/s] */
    volatile uint8_t pwm_on;
    float position;               /* compatibility snapshot; use accessor below */
    volatile int32_t step_position;/* completed STEP pulses, CW positive */
    volatile uint32_t emitted_steps;
    volatile motor_fault_t fault;
    uint32_t travel_steps;
    uint8_t homed;
    uint8_t homing_configured;
    volatile uint8_t homing_active;
    volatile uint8_t homing_cancel_requested;
    volatile uint8_t pulse_limit_stop_mask;
    volatile uint8_t pulse_limit_hit; /* raw contact latched until phase observes it */
    volatile uint8_t pulse_stop_enabled;
    volatile uint32_t pulse_stop_at;
    motor_homing_config_t homing;
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
/* Only TIM2 channel 1 in PWM1 mode is supported. Wire the corresponding HAL
 * PWM pulse-finished callback to motor_pulse_finished_isr(), and give TIM2 a
 * higher NVIC priority than the control timer. A compare interrupt counts each
 * completed fixed-width pulse; update flags/software UG events are NOT steps. */
void motor_pulse_finished_isr(motor_drv_t *motor_drv);
int32_t motor_position_steps(const motor_drv_t *motor_drv);
void motor_stop(motor_drv_t *motor_drv);
int motor_set_limits(motor_drv_t *motor_drv, float max_speed, float max_acceleration);
motor_homing_config_t motor_homing_default_config(void);
int motor_configure_homing(motor_drv_t *motor_drv, const motor_homing_config_t *config);
uint8_t motor_read_limits(const motor_drv_t *motor_drv);
int motor_clear_fault(motor_drv_t *motor_drv);
void motor_homing_cancel(motor_drv_t *motor_drv);
/* Blocking, main-context only; SysTick and TIM2 interrupts must remain enabled.
 * Calls HAL_Delay(1), returns measured travel on success, zero with latched fault
 * on failure. Normal control must not issue motor commands during this call. */
uint32_t motor_homing(motor_drv_t *motor_drv);
#endif /* MOTOR_H */