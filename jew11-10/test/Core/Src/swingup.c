#include "swingup.h"
#include "lqr.h"

void swingup_init(swingup_t *su)
{
    pendulum_config_t config = pendulum_default_config();
    config.dt_s = 1.0f / APP_CONTROL_HZ;
    config.effective_length_m = APP_EFFECTIVE_LENGTH_M;
    config.max_accel_m_s2 = APP_MAX_STEP_ACCEL * APP_METRES_PER_STEP;
    config.max_speed_m_s = APP_MAX_STEP_RATE_HZ * APP_METRES_PER_STEP;
    pendulum_control_init(&su->controller, &config);
    su->energia = 0.0f;
    su->errore = 3.14159265358979323846f;
    su->acc = 0.0f;
}

swingup_stato_t swingup_update(swingup_t *su, const Encoder_HandleTypeDef *encoder,
                              motor_drv_t *motor_drv)
{
    const pendulum_input_t input = {
        .position_m = motor_drv->position * m_per_step,
        .velocity_m_s = motor_drv->fcurrent * m_per_step,
        .angle_rad = encoder->rad_angle,
        .angular_velocity_rad_s = encoder->w
    };
    const pendulum_output_t output = pendulum_control_step(&su->controller, &input);
    su->energia = output.normalized_energy;
    su->errore = output.upright_error_rad;
    su->acc = output.acceleration_m_s2 / m_per_step;
    if (output.mode == PENDULUM_FAULT) {
        motor_stop(motor_drv);
        return SWINGUP_GUASTO;
    }
    if (output.mode == PENDULUM_BALANCE) return SWINGUP_CATTURA;
    motor_acc(su->acc, motor_drv);
    return SWINGUP_IN_CORSO;
}
