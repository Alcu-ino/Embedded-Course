#include "lqr.h"
#include <assert.h>
#include <stdio.h>

static void close_to(float actual, float expected)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) < 0.002f * fmaxf(1.0f, fabsf(expected)));
}

int main(void)
{
    Encoder_HandleTypeDef encoder = {0};
    motor_drv_t motor = {0};
    float acc;
    const float metres_per_step = 2.0f * (float)M_PI * 0.005f / 200.0f;
    encoder.rad_angle = (float)M_PI;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc, 0.0f);
    motor.position = 0.1f / metres_per_step;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc * metres_per_step, 0.25891947f);
    motor.position = -motor.position;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc * metres_per_step, -0.25891947f);
    motor.position = 0.0f;
    motor.fcurrent = 0.01f / metres_per_step;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc * metres_per_step, 0.043153245f);
    motor.fcurrent = 0.0f;
    /* Rightward bob lean about upright is NEGATIVE upright angular error;
     * physical cart must accelerate right to get under it. */
    encoder.rad_angle = (float)M_PI - 0.01f;
    lqr_controller(&encoder, &motor, &acc);
    assert(acc > 0.0f);
    const float positive = acc;
    encoder.rad_angle = -(float)M_PI + 0.01f;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc, -positive);
    encoder.rad_angle = 3.0f * (float)M_PI;
    lqr_controller(&encoder, &motor, &acc);
    close_to(acc * metres_per_step, 0.0f);
    puts("PASS balance: equilibrium, SI conversion, mirrored physical lean sign, multi-turn wrapping.");
    return 0;
}
