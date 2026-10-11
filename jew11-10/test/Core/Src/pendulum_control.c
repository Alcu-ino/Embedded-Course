#include "pendulum_control.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PENDULUM_PI 3.14159265358979323846f
#define PENDULUM_GRAVITY 9.81f

static float clampf(float x, float lo, float hi)
{
    return fminf(hi, fmaxf(lo, x));
}

float pendulum_wrap_angle(float angle_rad)
{
    return remainderf(angle_rad, 2.0f * PENDULUM_PI);
}

pendulum_config_t pendulum_default_config(void)
{
    pendulum_config_t config = {
        .dt_s = 0.001f,
        .effective_length_m = 0.70555556f,
        .max_accel_m_s2 = 0.31415927f,
        .max_speed_m_s = 0.15707963f,
        .track_half_width_m = 0.45f,
        .travel_margin_m = 0.03f,
        .swing_timeout_s = 90.0f,
        .energy_gain = 0.25f,
        .swing_position_gain = 6.0f,
        .swing_velocity_gain = 0.0f
    };
    return config;
}

static int valid_config(const pendulum_config_t *p)
{
    return p != NULL && isfinite(p->dt_s) && p->dt_s >= 0.0001f && p->dt_s <= 0.02f
        && isfinite(p->effective_length_m) && p->effective_length_m >= 0.01f
        && isfinite(p->max_accel_m_s2) && p->max_accel_m_s2 > 0.0f
        && isfinite(p->max_speed_m_s) && p->max_speed_m_s > 0.0f
        && isfinite(p->travel_margin_m) && p->travel_margin_m > 0.0f
        && isfinite(p->track_half_width_m) && p->track_half_width_m > 2.0f * p->travel_margin_m
        && isfinite(p->swing_timeout_s) && p->swing_timeout_s > 0.0f
        && isfinite(p->energy_gain) && p->energy_gain > 0.0f
        && isfinite(p->swing_position_gain) && p->swing_position_gain >= 0.0f
        && isfinite(p->swing_velocity_gain) && p->swing_velocity_gain >= 0.0f;
}

int pendulum_control_init(pendulum_control_t *controller, const pendulum_config_t *config)
{
    if (controller == NULL) return 0;
    memset(controller, 0, sizeof(*controller));
    if (!valid_config(config)) {
        controller->output.mode = PENDULUM_FAULT;
        controller->output.fault = PENDULUM_FAULT_CONFIG;
        return 0;
    }
    controller->config = *config;
    controller->kickoff_sign = 1.0f;
    return 1;
}

float pendulum_balance_acceleration(float length, const pendulum_input_t *input)
{
    /* Desired characteristic polynomial (s+2)^2*(s+3)^2:
     * s^4 +10*s^3 +37*s^2 +60*s +36.
     * Under the angle convention above the cart gains are positive and the
     * angle gains negative. Positive upright error means bob leans left.
     */
    const float kx = 36.0f * length / PENDULUM_GRAVITY;
    const float kv = 60.0f * length / PENDULUM_GRAVITY;
    const float kw = length * (10.0f + kv);
    const float kt = length * (37.0f + kx) + PENDULUM_GRAVITY;
    const float error = pendulum_wrap_angle(input->angle_rad - PENDULUM_PI);
    return kx * input->position_m + kv * input->velocity_m_s
        - kt * error - kw * input->angular_velocity_rad_s;
}

static pendulum_output_t fault(pendulum_control_t *c, pendulum_fault_t reason)
{
    c->output.mode = PENDULUM_FAULT;
    c->output.fault = reason;
    c->output.acceleration_m_s2 = 0.0f;
    return c->output;
}

pendulum_output_t pendulum_control_step(pendulum_control_t *c, const pendulum_input_t *in)
{
    if (c == NULL) {
        pendulum_output_t invalid = { .mode = PENDULUM_FAULT, .fault = PENDULUM_FAULT_CONFIG };
        return invalid;
    }
    if (c->output.mode == PENDULUM_FAULT) return c->output;
    if (in == NULL || !isfinite(in->position_m) || !isfinite(in->velocity_m_s)
        || !isfinite(in->angle_rad) || !isfinite(in->angular_velocity_rad_s))
        return fault(c, PENDULUM_FAULT_SENSOR);

    const pendulum_config_t *p = &c->config;
    if (fabsf(in->position_m) >= p->track_half_width_m - 0.25f * p->travel_margin_m)
        return fault(c, PENDULUM_FAULT_TRAVEL);

    const float error = pendulum_wrap_angle(in->angle_rad - PENDULUM_PI);
    const float energy = 0.5f * p->effective_length_m / PENDULUM_GRAVITY
        * in->angular_velocity_rad_s * in->angular_velocity_rad_s + 1.0f - cosf(in->angle_rad);
    c->output.upright_error_rad = error;
    c->output.normalized_energy = energy;
    c->elapsed_s += p->dt_s;
    const float balance_a = pendulum_balance_acceleration(p->effective_length_m, in);

    if (c->output.mode == PENDULUM_BALANCE) {
        if (fabsf(error) > 0.55f || fabsf(in->angular_velocity_rad_s) > 3.0f) {
            c->output.mode = PENDULUM_SWING_UP;
            c->capture_elapsed_s = 0.0f;
        }
    } else {
        c->swing_elapsed_s += p->dt_s;
        if (c->swing_elapsed_s >= p->swing_timeout_s) return fault(c, PENDULUM_FAULT_TIMEOUT);
        /* Require both angle and rate to be recoverable, plus an unsaturated
         * initial balance command. Twenty samples at 1 kHz reject fleeting
         * passes near upright. The wider loss window provides hysteresis.
         */
        if (fabsf(error) < 0.15f && fabsf(in->angular_velocity_rad_s) < 1.0f
            && fabsf(balance_a) <= p->max_accel_m_s2) {
            c->capture_elapsed_s += p->dt_s;
            if (c->capture_elapsed_s >= 0.02f) {
                c->output.mode = PENDULUM_BALANCE;
                c->captures++;
            }
        } else c->capture_elapsed_s = 0.0f;
    }

    float acceleration;
    if (c->output.mode == PENDULUM_BALANCE) acceleration = balance_a;
    else {
        /* d(E/(m*g*d))/dt = -a*omega*cos(theta)/g; this term increases
         * energy when below the upright energy and removes it when above.
         */
        acceleration = p->energy_gain * (energy - 2.0f)
            * in->angular_velocity_rad_s * cosf(in->angle_rad)
            - p->swing_position_gain * in->position_m
            - p->swing_velocity_gain * in->velocity_m_s;
        /* Defined starting impulse breaks the exact downward equilibrium. */
        if (c->elapsed_s <= 0.20f && fabsf(pendulum_wrap_angle(in->angle_rad)) < 0.2f
            && fabsf(in->angular_velocity_rad_s) < 0.5f) {
            if (c->elapsed_s <= 1.5f * p->dt_s)
                c->kickoff_sign = in->position_m > 0.0f ? -1.0f : 1.0f;
            acceleration = c->kickoff_sign * fminf(0.5f, p->max_accel_m_s2);
        }
    }
    acceleration = clampf(acceleration, -p->max_accel_m_s2, p->max_accel_m_s2);

    /* Bound speed by remaining stopping distance. Apply directional envelopes
     * so the inward speed is not needlessly reduced near an endpoint.
     * Deduct one control-period travel as allowance for discretisation.
     */
    const float allowance = p->max_speed_m_s * p->dt_s;
    const float usable = p->track_half_width_m - p->travel_margin_m - allowance;
    const float vpositive = fminf(p->max_speed_m_s,
        sqrtf(fmaxf(0.0f, 2.0f * p->max_accel_m_s2 * (usable - in->position_m))));
    const float vnegative = fminf(p->max_speed_m_s,
        sqrtf(fmaxf(0.0f, 2.0f * p->max_accel_m_s2 * (usable + in->position_m))));
    acceleration = clampf(acceleration, (-vnegative - in->velocity_m_s) / p->dt_s,
        (vpositive - in->velocity_m_s) / p->dt_s);
    /* Infeasible incoming velocity must never cause an above-limit command;
     * brake at the available acceleration. The hard travel check still latches.
     */
    c->output.acceleration_m_s2 = clampf(acceleration, -p->max_accel_m_s2, p->max_accel_m_s2);
    return c->output;
}
