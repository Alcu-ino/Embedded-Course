#ifndef SWINGUP_H
#define SWINGUP_H
#include "encoder.h"
#include "motor.h"
#include "pendulum_control.h"

/* Compatibility wrapper for old lab callers. The application now calls the
 * pure SI-unit controller directly with its measured stroke/configuration.
 */
typedef enum {
    SWINGUP_IN_CORSO = 0,
    SWINGUP_CATTURA = 1,
    SWINGUP_GUASTO = 2
} swingup_stato_t;

typedef struct {
    pendulum_control_t controller;
    float energia;  /* normalized energy, upright=2 (not joules) */
    float errore;   /* wrapped upright error rad */
    float acc;     /* acceleration step/s^2 */
} swingup_t;

void swingup_init(swingup_t *su);
swingup_stato_t swingup_update(swingup_t *su, const Encoder_HandleTypeDef *encoder,
                              motor_drv_t *motor_drv);
#endif
