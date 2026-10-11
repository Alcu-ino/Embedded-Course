/* Lifecycle isolation tests. Real application/controller/encoder code runs;
 * the motor boundary is replaced here. Separate motor tests exercise its HAL. */
#include "application.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static application_t app;
static motor_drv_t md;
static motor_t motor;
static Encoder_HandleTypeDef encoder;
static unsigned commands, homing_calls;
static uint8_t switches;
static int homing_behavior;

void motor_stop(motor_drv_t *m)
{
    m->fcurrent = 0.0f;
    m->pwm_on = 0u;
}
void motor_homing_cancel(motor_drv_t *m) { m->homing_cancel_requested = 1u; }
uint8_t motor_read_limits(const motor_drv_t *m) { (void)m; return switches; }
int32_t motor_position_steps(const motor_drv_t *m) { return m->step_position; }
void motor_acc(float acceleration, motor_drv_t *m)
{
    assert(app.mode == APP_SWING_UP || app.mode == APP_BALANCE);
    assert(isfinite(acceleration));
    commands++;
    m->fcurrent += acceleration / motor.fs;
    m->fcurrent = fminf(motor.fmax, fmaxf(-motor.fmax, m->fcurrent));
    m->pwm_on = 1u;
}
uint32_t motor_homing(motor_drv_t *m)
{
    homing_calls++;
    if (homing_behavior == 1) {
        m->fault = MOTOR_FAULT_TIMEOUT;
        return 0u;
    }
    if (homing_behavior == 2) {
        /* Interrupt occurs before foreground publishes a successful result. */
        application_stop(&app, APP_FAULT_USER_STOP);
    }
    m->homed = 1u;
    m->step_position = 0;
    return 6000u;
}

static void initialize(uint8_t confirmed)
{
    memset(&md, 0, sizeof(md));
    memset(&motor, 0, sizeof(motor));
    motor.fs = 1000.0f;
    motor.fmax = 1000.0f;
    md.motor = &motor;
    commands = homing_calls = 0u;
    switches = 0u;
    homing_behavior = 0;
    init_Encoder(&encoder, CPR, RES, 0.001f);
    encoder_set_direction(&encoder, 1);
    application_config_t config = {
        .hardware_confirmed = confirmed,
        .metres_per_step = 0.00015707963f,
        .control = pendulum_default_config()
    };
    application_init(&app, &md, &encoder, &config);
}

static void tick(unsigned count, uint16_t raw, uint8_t button)
{
    for (unsigned i = 0; i < count; ++i) application_tick(&app, raw, button);
}

static void reach_homing(void)
{
    initialize(1u);
    assert(!encoder.calibrated); /* regression: a real boot has no down reference */
    tick(50u, 137u, 0u);
    assert(app.mode == APP_WAIT_START);
    tick(40u, 137u, 1u);
    assert(app.mode == APP_ZEROING);
    tick(1100u, 137u, 0u);
    assert(app.mode == APP_HOMING && app.homing_requested);
    assert(encoder.calibrated && encoder.down_count == 137u);
    assert(commands == 0u && homing_calls == 0u);
}

static void reach_control(void)
{
    reach_homing();
    application_service(&app);
    assert(homing_calls == 1u);
    assert(app.mode == APP_HOMING); /* service posts; tick owns state */
    tick(1u, 137u, 0u);
    assert(app.mode == APP_SETTLING);
    assert(fabsf(app.config.control.track_half_width_m - 0.4712389f) < 1e-5f);
    tick(1100u, 137u, 0u);
    assert(app.mode == APP_SWING_UP && commands > 0u);
}

static void unverified_and_startup(void)
{
    initialize(0u);
    tick(100u, 731u, 0u);
    tick(2000u, 731u, 1u);
    application_service(&app);
    assert(app.mode == APP_WAIT_CONFIGURATION && commands == 0u && homing_calls == 0u);
    assert(encoder.raw_count == 731u);
    reach_control();
}

static void faults_latch(void)
{
    reach_control();
    switches = MOTOR_LIMIT_A;
    tick(1u, 137u, 0u);
    assert(app.mode == APP_FAULT && app.fault == APP_FAULT_LIMIT && !md.pwm_on);
    unsigned before = commands;
    switches = 0u;
    tick(2000u, 137u, 0u);
    tick(100u, 137u, 1u);
    assert(commands == before && app.mode == APP_FAULT);

    reach_control();
    tick(1u, 2400u, 0u);
    assert(app.fault == APP_FAULT_ENCODER && !md.pwm_on);

    reach_control();
    md.fault = MOTOR_FAULT_TIMER;
    tick(1u, 137u, 0u);
    assert(app.fault == APP_FAULT_MOTOR && !md.pwm_on);
}

static void homing_failure_and_cancellation(void)
{
    reach_homing();
    homing_behavior = 1;
    application_service(&app);
    tick(2000u, 137u, 0u);
    assert(app.fault == APP_FAULT_HOMING && commands == 0u);

    reach_homing();
    tick(40u, 137u, 1u);
    assert(app.fault == APP_FAULT_USER_STOP && md.homing_cancel_requested);
    application_service(&app);
    assert(homing_calls == 0u && commands == 0u);

    reach_homing();
    homing_behavior = 2;
    application_service(&app);
    tick(2000u, 137u, 0u);
    assert(app.mode == APP_FAULT && app.fault == APP_FAULT_USER_STOP && commands == 0u);
}

static void stationarity_and_timing(void)
{
    initialize(1u);
    tick(40u, 137u, 1u);
    for (unsigned i = 0; i < 11000u; ++i)
        application_tick(&app, (uint16_t)((137u + i) % 2400u), 0u);
    assert(app.fault == APP_FAULT_NOT_STATIONARY && homing_calls == 0u);

    reach_homing();
    application_service(&app);
    tick(1u, 137u, 0u);
    for (unsigned i = 1u; i <= 100u; ++i) tick(1u, (uint16_t)(137u + i), 0u);
    tick(31000u, 237u, 0u);
    assert(app.fault == APP_FAULT_NOT_STATIONARY && commands == 0u);

    reach_control();
    application_check_timing(&app, 170000u, 10000u, 168000u);
    assert(app.mode != APP_FAULT);
    application_check_timing(&app, 336000u, 10000u, 168000u);
    assert(app.fault == APP_FAULT_DEADLINE && !md.pwm_on);
    reach_control();
    application_check_timing(&app, 168000u, 168000u, 168000u);
    assert(app.fault == APP_FAULT_DEADLINE && !md.pwm_on);
}

static void capture_preserves_home(void)
{
    reach_control();
    md.step_position = 100;
    /* Seed a new sample history for an isolated upright handoff test. */
    encoder.primed = 0u;
    encoder.w = 0.0f;
    tick(50u, 1337u, 0u);
    assert(app.mode == APP_BALANCE);
    assert(md.step_position == 100);
}

static void commissioning_modes(void)
{
    reach_homing();
    app.config.run_mode = APP_RUN_HOMING_ONLY;
    application_service(&app);
    tick(10000u, 137u, 0u);
    assert(app.mode == APP_HOMED_IDLE && commands == 0u && !md.pwm_on);
    switches = MOTOR_LIMIT_A;
    tick(1u, 137u, 0u);
    assert(app.fault == APP_FAULT_LIMIT);

    reach_homing();
    app.config.run_mode = APP_RUN_BALANCE_ONLY;
    application_service(&app);
    tick(10000u, 137u, 0u);
    assert(app.mode == APP_WAIT_BALANCE && commands == 0u && !md.pwm_on);
    /* Simulate a slow manual lift, without seeding a new encoder history. */
    for (unsigned i = 1u; i <= 1200u; ++i) tick(10u, (uint16_t)(137u + i), 0u);
    tick(500u, 1337u, 0u);
    assert(app.mode == APP_BALANCE && commands > 0u);
    assert(md.step_position == 0);
    /* Leaving the capture region stops; balance-only never starts a swing-up. */
    for (unsigned i = 1u; i <= 250u; ++i) tick(10u, (uint16_t)(1337u + i), 0u);
    assert(app.fault == APP_FAULT_CAPTURE_LOST && !md.pwm_on);
}

int main(void)
{
    unverified_and_startup();
    faults_latch();
    homing_failure_and_cancellation();
    stationarity_and_timing();
    capture_preserves_home();
    commissioning_modes();
    puts("PASS application: uncalibrated startup, configuration gate, homing/settling/control, staged commissioning, latched faults, cancellation, timing and preserved home.");
    return 0;
}
