#include "encoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* fake_hal.h is injected by the host build; firmware uses the real TIM3. */
TIM_TypeDef fake_encoder_timer;

static const float radians_per_count = 6.2831853071795864769f / 2400.0f;

static void near(float actual, float expected, float tolerance)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) < tolerance);
}

static Encoder_HandleTypeDef calibrated(uint16_t down)
{
    Encoder_HandleTypeDef encoder;
    init_Encoder(&encoder, 600u, 4u, 0.001f);
    encoder_calibrate_down(&encoder, down);
    assert(encoder.calibrated && !encoder.primed && !encoder.valid);
    return encoder;
}

static void calibration_and_startup(void)
{
    Encoder_HandleTypeDef encoder;
    init_Encoder(&encoder, 600u, 4u, 0.001f);
    assert(!encoder_process_count(&encoder, 1700u));
    encoder_calibrate_down(&encoder, 1700u);
    assert(encoder_process_count(&encoder, 1700u));
    near(encoder.rad_angle, 0.0f, 1e-6f);
    near(encoder.w, 0.0f, 1e-6f);
    /* First sample at upright must not invent a startup 3142 rad/s. */
    encoder = calibrated(1700u);
    assert(encoder_process_count(&encoder, 500u));
    near(encoder.rad_angle, 3.14159265358979323846f, 1e-6f);
    near(encoder.w, 0.0f, 1e-6f);
    near(encoder.prev_angle, 180.0f, 1e-5f);
}

static void wraps_and_direction(void)
{
    Encoder_HandleTypeDef encoder = calibrated(0u);
    assert(encoder_process_count(&encoder, 2399u));
    assert(encoder_process_count(&encoder, 0u));
    assert(encoder.w > 0.0f && encoder.w < radians_per_count / encoder.ts);
    near(encoder.rad_angle, 0.0f, 1e-6f);
    encoder = calibrated(0u);
    assert(encoder_process_count(&encoder, 0u));
    assert(encoder_process_count(&encoder, 2399u));
    assert(encoder.w < 0.0f && encoder.w > -radians_per_count / encoder.ts);
    near(encoder.rad_angle, 6.2831853071795864769f - radians_per_count, 1e-6f);

    encoder_set_direction(&encoder, -1);
    assert(!encoder.calibrated && !encoder.valid);
    assert(!encoder_process_count(&encoder, 0u));
    encoder_calibrate_down(&encoder, 0u);
    assert(encoder_process_count(&encoder, 0u));
    assert(encoder_process_count(&encoder, 1u));
    assert(encoder.w < 0.0f);
    near(encoder.angle, 359.85f, 5e-5f);
    encoder_calibrate_down(&encoder, 2399u);
    assert(encoder_process_count(&encoder, 2399u));
    assert(encoder_process_count(&encoder, 0u));
    assert(encoder.w < 0.0f);
    near(encoder.angle, 359.85f, 5e-5f);
}

static void filtering(void)
{
    Encoder_HandleTypeDef encoder = calibrated(0u);
    assert(encoder_process_count(&encoder, 0u));
    float peak = 0.0f;
    float sum = 0.0f;
    /* One count every 10 ms is a low constant speed. Check mean scale and
     * suppression of the original full 2.618 rad/s one-tick derivative. */
    for (uint16_t i = 1u; i <= 1000u; ++i) {
        assert(encoder_process_count(&encoder, i / 10u));
        peak = fmaxf(peak, fabsf(encoder.w));
        if (i > 500u) sum += encoder.w;
    }
    near(sum / 500.0f, radians_per_count / 0.010f, 0.001f);
    assert(peak < 0.5f);
    for (unsigned i = 0u; i < 1000u; ++i) {
        assert(encoder_process_count(&encoder, 100u));
    }
    near(encoder.w, 0.0f, 1e-5f);
    for (unsigned i = 0u; i < 1000u; ++i) {
        assert(encoder_process_count(&encoder, (uint16_t)(100u + i % 2u)));
        assert(fabsf(encoder.w) < 0.30f);
    }
}

static void invalid_data(void)
{
    Encoder_HandleTypeDef encoder = calibrated(0u);
    assert(encoder_process_count(&encoder, 0u));
    assert(!encoder_process_count(&encoder, 1000u));
    assert(!encoder.valid && encoder.previous_count == 0u);
    near(encoder.angle, 0.0f, 1e-6f);
    assert(encoder_process_count(&encoder, 1u));
    assert(!encoder_process_count(&encoder, 2400u));
    assert(encoder.previous_count == 1u);

    const float bad_periods[] = {0.0f, -0.001f, NAN, INFINITY};
    for (unsigned i = 0u; i < sizeof bad_periods / sizeof bad_periods[0]; ++i) {
        init_Encoder(&encoder, 600u, 4u, bad_periods[i]);
        encoder_calibrate_down(&encoder, 0u);
        assert(!encoder.calibrated && !encoder_process_count(&encoder, 0u));
    }
    init_Encoder(&encoder, 0u, 4u, 0.001f);
    encoder_calibrate_down(&encoder, 0u);
    assert(!encoder.calibrated);
    init_Encoder(&encoder, UINT32_MAX, 255u, 0.001f);
    encoder_calibrate_down(&encoder, 0u);
    assert(!encoder.calibrated);
    encoder = calibrated(0u);
    encoder.velocity_cutoff_hz = NAN;
    assert(!encoder_process_count(&encoder, 0u));
    encoder = calibrated(0u);
    encoder.max_angular_speed_rad_s = 0.0f;
    assert(!encoder_process_count(&encoder, 0u));
    encoder = calibrated(0u);
    encoder_set_direction(&encoder, 0);
    encoder_calibrate_down(&encoder, 0u);
    assert(!encoder.calibrated);
    assert(!encoder_process_count(NULL, 0u));
}

static void hardware_wrapper(void)
{
    Encoder_HandleTypeDef encoder = calibrated(723u);
    fake_encoder_timer.CNT = 723u;
    update_Encoder(&encoder);
    assert(encoder.valid);
    near(encoder.rad_angle, 0.0f, 1e-6f);
    fake_encoder_timer.CNT = 724u;
    update_Encoder(&encoder);
    assert(encoder.valid && encoder.w > 0.0f);
    near(encoder.rad_angle, radians_per_count, 1e-6f);
}

int main(void)
{
    calibration_and_startup();
    wraps_and_direction();
    filtering();
    invalid_data();
    hardware_wrapper();
    puts("PASS encoder: calibrated zero, startup, wrap/sign, filtered quantization, invalid data, TIM3 wrapper.");
    return 0;
}
