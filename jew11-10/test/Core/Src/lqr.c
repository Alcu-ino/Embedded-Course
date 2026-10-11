#include "lqr.h"

void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc)
{
    const pendulum_input_t input = {
        .position_m = motor_drv->position * m_per_step,
        .velocity_m_s = motor_drv->fcurrent * m_per_step,
        .angle_rad = encoder->rad_angle,
        .angular_velocity_rad_s = encoder->w
    };
    *acc = pendulum_balance_acceleration(APP_EFFECTIVE_LENGTH_M, &input) / m_per_step;
}
