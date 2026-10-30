#ifndef OFFLINE_AUDIT_HARNESS_H
#define OFFLINE_AUDIT_HARNESS_H

#include "lqr.h"
#include "swingup.h"

extern Encoder_HandleTypeDef encoder;
extern motor_drv_t motor_drv;
extern swingup_t su;
extern volatile float acc;
extern volatile uint8_t control_on;
extern volatile uint8_t guasto;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
