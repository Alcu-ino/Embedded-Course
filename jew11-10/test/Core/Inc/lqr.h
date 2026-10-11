#ifndef LQR_H
#define LQR_H
#include <math.h>
#include "encoder.h"
#include "motor.h"
#include "pendulum_control.h"
#include "app_config.h"
/* Compatibility name uses the same configured geometry/microstepping. */
#define m_per_step APP_METRES_PER_STEP
/* Compatibility entry point: shared pole-placement balance law, step/s^2.
 * Active firmware uses pendulum_control_step for limits and mode selection.
 */
void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc);
#endif
