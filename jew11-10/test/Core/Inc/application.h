#ifndef PENDULUM_APPLICATION_H
#define PENDULUM_APPLICATION_H

#include "encoder.h"
#include "motor.h"
#include "pendulum_control.h"

typedef enum {
    APP_WAIT_CONFIGURATION = 0,
    APP_WAIT_START,
    APP_ZEROING,
    APP_HOMING,
    APP_SETTLING,
    APP_HOMED_IDLE,
    APP_WAIT_BALANCE,
    APP_SWING_UP,
    APP_BALANCE,
    APP_FAULT
} application_mode_t;

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_CONFIGURATION,
    APP_FAULT_USER_STOP,
    APP_FAULT_ENCODER,
    APP_FAULT_NOT_STATIONARY,
    APP_FAULT_HOMING,
    APP_FAULT_LIMIT,
    APP_FAULT_CONTROLLER,
    APP_FAULT_MOTOR,
    APP_FAULT_DEADLINE,
    APP_FAULT_CAPTURE_LOST
} application_fault_t;

typedef enum {
    APP_RUN_SWINGUP_BALANCE = 0,
    APP_RUN_HOMING_ONLY,
    APP_RUN_BALANCE_ONLY
} application_run_mode_t;

typedef struct {
    uint8_t hardware_confirmed;
    application_run_mode_t run_mode;
    float metres_per_step;
    pendulum_config_t control;
} application_config_t;

typedef struct {
    motor_drv_t *motor;
    Encoder_HandleTypeDef *encoder;
    application_config_t config;
    pendulum_control_t controller;
    volatile application_mode_t mode;
    volatile application_fault_t fault;
    volatile uint8_t homing_requested;
    volatile uint8_t homing_complete;
    volatile uint32_t homing_travel;
    uint32_t phase_ticks;
    uint32_t stationary_ticks;
    uint16_t button_ticks;
    uint16_t zero_anchor_count;
    uint8_t zero_anchor_valid;
    uint8_t button_down;
    uint8_t last_button_raw;
    volatile float commanded_acceleration;
    volatile uint8_t limits;
} application_t;

void application_init(application_t *app, motor_drv_t *motor,
                      Encoder_HandleTypeDef *encoder, const application_config_t *config);
/* Called exactly at control.dt_s in the control IRQ. This is the sole writer
 * of the lifecycle state after init; the foreground only posts homing results. */
void application_tick(application_t *app, uint16_t encoder_count, uint8_t button_pressed);
/* Main-loop service: runs blocking homing while the control IRQ remains alive
 * for encoder sampling and cancellation. It issues no balance commands. */
void application_service(application_t *app);
void application_stop(application_t *app, application_fault_t reason);
/* DWT cycle measurements: interval=0 skips the first-interval check. */
void application_check_timing(application_t *app, uint32_t interval_cycles,
                              uint32_t runtime_cycles, uint32_t period_cycles);

#endif
