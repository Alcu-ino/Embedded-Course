#include "application.h"
#include <math.h>
#include <string.h>

static uint32_t ticks_for(const application_t *app, float seconds)
{
    return (uint32_t)ceilf(seconds / app->config.control.dt_s);
}

static void enter_mode(application_t *app, application_mode_t mode)
{
    app->phase_ticks = 0u;
    app->stationary_ticks = 0u;
    app->zero_anchor_valid = 0u;
    app->mode = mode;
}

void application_stop(application_t *app, application_fault_t reason)
{
    if (app == NULL || app->motor == NULL) return;
    /* Cancellation remains visible to the foreground homing loop. */
    motor_homing_cancel(app->motor);
    motor_stop(app->motor);
    app->commanded_acceleration = 0.0f;
    app->homing_requested = 0u;
    if (app->fault == APP_FAULT_NONE) app->fault = reason;
    app->mode = APP_FAULT;
}

void application_init(application_t *app, motor_drv_t *motor,
                      Encoder_HandleTypeDef *encoder, const application_config_t *config)
{
    memset(app, 0, sizeof(*app));
    app->motor = motor;
    app->encoder = encoder;
    if (config == NULL || motor == NULL || encoder == NULL) {
        app->fault = APP_FAULT_CONFIGURATION;
        app->mode = APP_FAULT;
        return;
    }
    app->config = *config;
    motor_stop(motor);
    if ((unsigned)config->run_mode > APP_RUN_BALANCE_ONLY ||
        !isfinite(config->metres_per_step) || config->metres_per_step <= 0.0f ||
        !pendulum_control_init(&app->controller, &config->control)) {
        application_stop(app, APP_FAULT_CONFIGURATION);
        return;
    }
    enter_mode(app, config->hardware_confirmed ? APP_WAIT_START : APP_WAIT_CONFIGURATION);
}

static uint8_t button_rising(application_t *app, uint8_t raw)
{
    raw = raw != 0u;
    if (raw != app->last_button_raw) {
        app->last_button_raw = raw;
        app->button_ticks = 0u;
    }
    if (app->button_ticks < ticks_for(app, 0.030f)) app->button_ticks++;
    if (app->button_ticks >= ticks_for(app, 0.030f) && raw != app->button_down) {
        app->button_down = raw;
        return raw;
    }
    return 0u;
}

void application_tick(application_t *app, uint16_t encoder_count, uint8_t button_pressed)
{
    if (app == NULL || app->motor == NULL || app->encoder == NULL) return;
    if (app->mode == APP_FAULT) return; /* fault reset requires explicit board reset */
    /* Before the physical down calibration, measure stationarity in raw
     * counts. A deliberately uncalibrated estimator must not block startup. */
    const uint32_t counts = app->encoder->cpr * (uint32_t)app->encoder->res;
    app->encoder->raw_count = encoder_count;
    const uint8_t sensor_ok = app->encoder->calibrated ?
        encoder_process_count(app->encoder, encoder_count) :
        (counts >= 2u && counts <= 65536u && encoder_count < counts && app->mode < APP_HOMING);
    app->limits = motor_read_limits(app->motor);
    const uint8_t pressed = button_rising(app, button_pressed);
    if (app->mode == APP_WAIT_CONFIGURATION) return;
    if (pressed && app->mode != APP_WAIT_START) {
        application_stop(app, APP_FAULT_USER_STOP);
        return;
    }
    if (!sensor_ok) {
        application_stop(app, APP_FAULT_ENCODER);
        return;
    }
    app->phase_ticks++;
    if (app->mode == APP_WAIT_START) {
        if (pressed) enter_mode(app, APP_ZEROING);
        return;
    }
    if (app->mode == APP_ZEROING) {
        /* The user must start with the pendulum hanging DOWN. An incremental
         * encoder cannot distinguish a motionless top from a motionless bottom. */
        if (!app->zero_anchor_valid) {
            app->zero_anchor_count = encoder_count;
            app->zero_anchor_valid = 1u;
        }
        int32_t delta = (int32_t)encoder_count - (int32_t)app->zero_anchor_count;
        if (delta > (int32_t)(counts / 2u)) delta -= (int32_t)counts;
        if (delta < -(int32_t)(counts / 2u)) delta += (int32_t)counts;
        if (delta >= -2 && delta <= 2) app->stationary_ticks++;
        else {
            app->stationary_ticks = 0u;
            app->zero_anchor_count = encoder_count;
        }
        if (app->stationary_ticks >= ticks_for(app, 1.0f)) {
            encoder_calibrate_down(app->encoder, encoder_count);
            if (!app->encoder->calibrated) {
                application_stop(app, APP_FAULT_ENCODER);
                return;
            }
            app->homing_complete = 0u;
            app->homing_travel = 0u;
            enter_mode(app, APP_HOMING);
            app->homing_requested = 1u;
        } else if (app->phase_ticks >= ticks_for(app, 10.0f)) {
            application_stop(app, APP_FAULT_NOT_STATIONARY);
        }
        return;
    }
    if (app->mode == APP_HOMING) {
        if (app->homing_complete) {
            app->homing_complete = 0u;
            if (app->homing_travel == 0u || app->motor->fault != MOTOR_FAULT_NONE) {
                application_stop(app, APP_FAULT_HOMING);
                return;
            }
            app->config.control.track_half_width_m =
                0.5f * (float)app->homing_travel * app->config.metres_per_step;
            if (!pendulum_control_init(&app->controller, &app->config.control)) {
                application_stop(app, APP_FAULT_CONFIGURATION);
                return;
            }
            enter_mode(app, app->config.run_mode == APP_RUN_HOMING_ONLY ? APP_HOMED_IDLE :
                app->config.run_mode == APP_RUN_BALANCE_ONLY ? APP_WAIT_BALANCE : APP_SETTLING);
        }
        return;
    }
    /* A physical endpoint during settling/control is a latched stop. */
    if (app->limits != 0u) {
        application_stop(app, APP_FAULT_LIMIT);
        return;
    }
    if (app->motor->fault != MOTOR_FAULT_NONE) {
        application_stop(app, APP_FAULT_MOTOR);
        return;
    }
    if (app->mode == APP_HOMED_IDLE) return;
    if (app->mode == APP_WAIT_BALANCE) {
        const pendulum_input_t input = {
            .position_m = (float)motor_position_steps(app->motor) * app->config.metres_per_step,
            .velocity_m_s = 0.0f,
            .angle_rad = app->encoder->rad_angle,
            .angular_velocity_rad_s = app->encoder->w
        };
        /* For staged commissioning, remain stopped while the pendulum is
         * raised by hand. Only engage after the normal capture guard passes. */
        const float error = pendulum_wrap_angle(input.angle_rad - 3.14159265358979323846f);
        if (fabsf(error) < 0.15f && fabsf(input.angular_velocity_rad_s) < 1.0f &&
            fabsf(pendulum_balance_acceleration(app->config.control.effective_length_m, &input)) <=
                app->config.control.max_accel_m_s2) {
            const pendulum_output_t output = pendulum_control_step(&app->controller, &input);
            if (output.mode == PENDULUM_BALANCE) enter_mode(app, APP_BALANCE);
            else if (output.mode == PENDULUM_FAULT) application_stop(app, APP_FAULT_CONTROLLER);
        } else app->controller.capture_elapsed_s = 0.0f;
        return;
    }
    if (app->mode == APP_SETTLING) {
        const float bottom_error = pendulum_wrap_angle(app->encoder->rad_angle);
        if (fabsf(bottom_error) < 0.0872665f && fabsf(app->encoder->w) < 0.08f)
            app->stationary_ticks++;
        else app->stationary_ticks = 0u;
        if (app->stationary_ticks >= ticks_for(app, 1.0f)) enter_mode(app, APP_SWING_UP);
        else if (app->phase_ticks >= ticks_for(app, 30.0f))
            application_stop(app, APP_FAULT_NOT_STATIONARY);
        return;
    }
    if (app->mode == APP_SWING_UP || app->mode == APP_BALANCE) {
        const pendulum_input_t input = {
            .position_m = (float)motor_position_steps(app->motor) * app->config.metres_per_step,
            .velocity_m_s = app->motor->fcurrent * app->config.metres_per_step,
            .angle_rad = app->encoder->rad_angle,
            .angular_velocity_rad_s = app->encoder->w
        };
        const pendulum_output_t output = pendulum_control_step(&app->controller, &input);
        if (output.mode == PENDULUM_FAULT) {
            application_stop(app, APP_FAULT_CONTROLLER);
            return;
        }
        if (app->config.run_mode == APP_RUN_BALANCE_ONLY && output.mode != PENDULUM_BALANCE) {
            application_stop(app, APP_FAULT_CAPTURE_LOST);
            return;
        }
        app->mode = output.mode == PENDULUM_BALANCE ? APP_BALANCE : APP_SWING_UP;
        app->commanded_acceleration = output.acceleration_m_s2;
        motor_acc(output.acceleration_m_s2 / app->config.metres_per_step, app->motor);
        if (app->motor->fault != MOTOR_FAULT_NONE) application_stop(app, APP_FAULT_MOTOR);
    }
}

void application_service(application_t *app)
{
    if (app == NULL || app->mode != APP_HOMING || !app->homing_requested) return;
    app->homing_requested = 0u;
    const uint32_t travel = motor_homing(app->motor);
    /* Publish the result last; lifecycle decisions belong to the tick. */
    app->homing_travel = travel;
    app->homing_complete = 1u;
}

void application_check_timing(application_t *app, uint32_t interval_cycles,
                              uint32_t runtime_cycles, uint32_t period_cycles)
{
    if (app == NULL || app->mode < APP_ZEROING || app->mode == APP_FAULT) return;
    /* Small interrupt-entry jitter is allowed; a lost 1 ms tick is not.
     * Runtime at/above a full period cannot sustain the required sample rate. */
    if (period_cycles == 0u || runtime_cycles >= period_cycles ||
        interval_cycles > period_cycles + period_cycles / 2u)
        application_stop(app, APP_FAULT_DEADLINE);
}
