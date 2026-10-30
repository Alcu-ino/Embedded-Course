/* Executes real lqr.c, swingup.c, and the callback extracted from main.c.
 * HAL time, encoder acquisition, position acquisition and motor output are
 * substitutes: these tests cannot establish physical stability or timer timing.
 */
#include "harness.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const float pi = 3.14159265358979323846f;
static uint32_t tick;
static unsigned motor_calls;
TIM_TypeDef host_tim1;
Encoder_HandleTypeDef encoder;
motor_drv_t motor_drv;
swingup_t su;
volatile float acc;
volatile uint8_t control_on;
volatile uint8_t guasto;

uint32_t HAL_GetTick(void) { return tick; }
void update_Encoder(Encoder_HandleTypeDef *value) { (void)value; }
void motor_aggiorna_posizione(motor_drv_t *value) { (void)value; }

/* Record receipt of a command. Integrate the requested velocity as motor_acc
 * does, without pretending to simulate STEP pulses or physical cart motion. */
void motor_acc(float acceleration, motor_drv_t *value)
{
    ++motor_calls;
    value->fcurrent += acceleration * encoder.ts;
    if (value->fcurrent > 32000.0f) value->fcurrent = 32000.0f;
    if (value->fcurrent < -32000.0f) value->fcurrent = -32000.0f;
}

static void reset_case(float degrees, float raw_speed)
{
    memset(&encoder, 0, sizeof encoder);
    memset(&motor_drv, 0, sizeof motor_drv);
    encoder.ts = 0.001f;
    encoder.cpr = 600;
    encoder.res = 4;
    encoder.angle = degrees;
    encoder.rad_angle = degrees * pi / 180.0f;
    encoder.w = raw_speed;
    swingup_init(&su);
    acc = 0;
    control_on = 0;
    guasto = 0;
    motor_calls = 0;
    tick += 100; /* Trigger the real LQR's existing reinitialization rule. */
}

static void reproduce_controller_gap(float degrees)
{
    reset_case(degrees, 0);
    motor_drv.fcurrent = 5000.0f;
    TIM_HandleTypeDef timer = { .Instance = TIM1 };
    HAL_TIM_PeriodElapsedCallback(&timer);
    printf("%.1f deg: motor_acc calls = %u, command = %.1f, retained speed = %.1f step/s\n",
           degrees, motor_calls, (double)acc, (double)motor_drv.fcurrent);
    assert(control_on == 0);
    assert(motor_calls == 0);
    assert(acc == 0.0f);
    assert(motor_drv.fcurrent == 5000.0f);
}

static void reproduce_raw_velocity_startup(float sign)
{
    const float raw_one_count = sign * 2.0f * pi / 2400.0f / 0.001f;
    reset_case(180.0f, raw_one_count);
    float acceleration_steps = 0;
    lqr_controller(&encoder, &motor_drv, &acceleration_steps);
    printf("Upright, entry raw w = %+.6f rad/s: estimated w = %+.6f, a = %+.6f m/s^2\n",
           (double)raw_one_count, (double)lqr_w_stima,
           (double)(acceleration_steps * M_PER_STEP));
    assert(fabsf(lqr_u - sign * LQR_ACC_MAX) < 0.0001f);
}

int main(void)
{
    reproduce_controller_gap(170.0f);
    reproduce_controller_gap(190.0f);

    /* Sanity check: the real callback does invoke the controller at upright. */
    reset_case(180.0f, 0);
    TIM_HandleTypeDef timer = { .Instance = TIM1 };
    HAL_TIM_PeriodElapsedCallback(&timer);
    assert(control_on == 1);
    assert(motor_calls == 1);
    assert(fabsf(lqr_u) < 0.0001f);

    reproduce_raw_velocity_startup(1.0f);
    reproduce_raw_velocity_startup(-1.0f);
    puts("PASS: current controller gap and raw-derivative entry saturation reproduced.");
    puts("These assertions reproduce current defects; they do not validate fixes or hardware.");
    return 0;
}
