#include "encoder.h"
#ifndef PENDULUM_HOST_TEST
#include "main.h"
#endif
#include <math.h>
#include <stddef.h>

#define ENCODER_TWO_PI 6.2831853071795864769f

static uint32_t counts_per_turn(const Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL || encoder->res == 0u || encoder->cpr == 0u ||
        encoder->cpr > 65536u / encoder->res) {
        return 0u;
    }
    return encoder->cpr * (uint32_t)encoder->res;
}

static uint8_t parameters_valid(const Encoder_HandleTypeDef *encoder)
{
    return counts_per_turn(encoder) >= 2u &&
           isfinite(encoder->ts) && encoder->ts > 0.0f &&
           isfinite(encoder->velocity_cutoff_hz) &&
           encoder->velocity_cutoff_hz > 0.0f &&
           isfinite(encoder->max_angular_speed_rad_s) &&
           encoder->max_angular_speed_rad_s > 0.0f &&
           (encoder->direction == 1 || encoder->direction == -1);
}

void init_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr,
                  uint8_t res, float sample_time_s)
{
    if (encoder == NULL) return;
    *encoder = (Encoder_HandleTypeDef){0};
    encoder->cpr = cpr;
    encoder->res = res;
    encoder->ts = sample_time_s;
    encoder->direction = 1;
    encoder->velocity_cutoff_hz = ENCODER_DEFAULT_CUTOFF_HZ;
    encoder->max_angular_speed_rad_s = ENCODER_DEFAULT_MAX_SPEED_RAD_S;
}

void reset_Encoder(Encoder_HandleTypeDef *encoder, uint32_t cpr,
                   uint8_t res, float sample_time_s)
{
    init_Encoder(encoder, cpr, res, sample_time_s);
}

void encoder_set_direction(Encoder_HandleTypeDef *encoder, int8_t direction)
{
    if (encoder == NULL) return;
    encoder->direction = direction;
    encoder->calibrated = 0u;
    encoder->primed = 0u;
    encoder->valid = 0u;
    encoder->w = 0.0f;
}

void encoder_calibrate_down(Encoder_HandleTypeDef *encoder, uint16_t raw)
{
    if (encoder == NULL) return;
    encoder->valid = 0u;
    encoder->calibrated = 0u;
    encoder->primed = 0u;
    encoder->raw_count = raw;
    if (!parameters_valid(encoder) || raw >= counts_per_turn(encoder)) return;

    encoder->down_count = raw;
    encoder->previous_count = raw;
    encoder->angle = 0.0f;
    encoder->prev_angle = 0.0f;
    encoder->rad_angle = 0.0f;
    encoder->w = 0.0f;
    encoder->calibrated = 1u;
    /* Valid remains false until an actual sample has been processed. */
}

uint8_t encoder_process_count(Encoder_HandleTypeDef *encoder, uint16_t raw)
{
    if (encoder == NULL) return 0u;
    encoder->raw_count = raw;
    encoder->valid = 0u;
    uint32_t counts = counts_per_turn(encoder);
    if (!parameters_valid(encoder) || !encoder->calibrated || raw >= counts ||
        encoder->down_count >= counts || encoder->previous_count >= counts) {
        return 0u;
    }

    int32_t delta = (int32_t)raw - (int32_t)encoder->previous_count;
    if (delta > (int32_t)(counts / 2u)) delta -= (int32_t)counts;
    if (delta < -(int32_t)(counts / 2u)) delta += (int32_t)counts;
    const float radians_per_count = ENCODER_TWO_PI / (float)counts;
    const float raw_velocity = (float)(delta * encoder->direction) *
                               radians_per_count / encoder->ts;
    if (encoder->primed &&
        (!isfinite(raw_velocity) ||
         fabsf(raw_velocity) > encoder->max_angular_speed_rad_s)) {
        return 0u;
    }

    int32_t offset = ((int32_t)raw - (int32_t)encoder->down_count) *
                     encoder->direction;
    if (offset < 0) offset += (int32_t)counts;
    encoder->prev_angle = encoder->angle;
    encoder->rad_angle = (float)offset * radians_per_count;
    encoder->angle = (float)offset * 360.0f / (float)counts;
    if (!encoder->primed) {
        encoder->prev_angle = encoder->angle;
        encoder->w = 0.0f;
        encoder->primed = 1u;
    } else {
        /* Backward-Euler first-order low-pass: stable for every positive ts.
         * No trig/exp calls in the 1 kHz sample path. Filter delay is represented
         * in the nonlinear host simulation, not assumed to be zero. */
        float tau = 1.0f / (ENCODER_TWO_PI * encoder->velocity_cutoff_hz);
        float alpha = encoder->ts / (tau + encoder->ts);
        encoder->w += alpha * (raw_velocity - encoder->w);
    }
    encoder->previous_count = raw;
    encoder->valid = 1u;
    return 1u;
}

#ifndef PENDULUM_HOST_TEST
void update_Encoder(Encoder_HandleTypeDef *encoder)
{
    /* One CPU read per control tick. TIM3 is the configured encoder timer;
     * the unused TIM4/DMA route is intentionally absent from this estimator. */
    (void)encoder_process_count(encoder, (uint16_t)TIM3->CNT);
}
#endif
