/* Executes the production motor.c against virtual GPIO and a TIM2-like PWM1
 * timer. HAL_GetTick is read-only; HAL_Delay advances virtual time, including
 * compare interrupts for every completed STEP pulse. Models fixed pulse width,
 * counter rollover, immediate ARR writes, and software UG without false steps.
 * The ideal stepper misses no steps: mechanics/electrical timing need bench QA. */
#include "motor.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

GPIO_TypeDef fake_gpio_b, fake_gpio_c;
uint32_t SystemCoreClock = 180000000u;
static TIM_TypeDef timer;
static motor_drv_t *active_motor;
static uint32_t irq_mask;
static uint32_t fractional_tick;
static jmp_buf watchdog;
static struct {
    const char *name;
    int position;
    int track_steps;
    int fail_start;
    int inject_limit_bounce;
    int force_release_b;
    int running;
    int a_low;
    int b_low;
    int cw_at_b;
    int both_pressed;
    int stuck_a;
    int missing_b;
    int frozen_timer;
    int cancel;
    int delayed_poll;
    int bounce;
    uint32_t contact_at;
    unsigned contact_bit;
    int pulse_done;
    int overrun;
    unsigned emitted_steps;
    unsigned into_endstop;
    unsigned last_switches;
    uint32_t tick;
    uint32_t elapsed;
    unsigned delay_calls;
    float max_observed_speed;
} sim;

#define TRACK_STEPS (sim.track_steps)
enum { WATCHDOG_MS = 30000 };
enum { EXPECT_CENTRE, EXPECT_REJECT_WITHOUT_MOVING, EXPECT_FAULT, EXPECT_DRIVER };

/* The simulated switches live here. These fictional levels and direction
 * mappings are intentionally varied in tests; they do not identify real wiring. */
static unsigned pressed_switches(void)
{
    if (sim.both_pressed) return 3u;
    unsigned a = (sim.position <= 0 || sim.stuck_a) ? 1u : 0u;
    unsigned b = (sim.position >= TRACK_STEPS && !sim.missing_b && !sim.force_release_b) ? 2u : 0u;
    unsigned state = a | b;
    if (sim.bounce && (uint32_t)(sim.elapsed - sim.contact_at) < 7u &&
        (uint32_t)(sim.elapsed - sim.contact_at) % 2u != 0u)
        state &= ~sim.contact_bit;
    return state;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port != LIMIT_A_GPIO_Port ||
        (pin != LIMIT_A_Pin && pin != LIMIT_B_Pin)) abort();
    unsigned bit = pin == LIMIT_A_Pin ? 1u : 2u;
    int active_low = pin == LIMIT_A_Pin ? sim.a_low : sim.b_low;
    int pressed = (pressed_switches() & bit) != 0;
    return (pressed != active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~(uint32_t)pin;
}

int HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    if (sim.fail_start) return 1;
    sim.running = 1;
    timer.CR1 |= TIM_CR1_CEN;
    timer.CNT = 0u;
    timer.EGR = 0u;
    sim.pulse_done = 0;
    return HAL_OK;
}

int HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    sim.running = 0;
    timer.CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

static void pulse_complete(void)
{
    sim.emitted_steps++;
    int direction = (fake_gpio_b.ODR & (1u << 12)) ? -1 : 1;
    if (!sim.cw_at_b) direction = -direction;
    int next = sim.position + direction;
    if (next < 0 || next > TRACK_STEPS) sim.into_endstop++;
    else {
        sim.position = next;
        if (sim.position == 0 || sim.position == TRACK_STEPS) {
            sim.contact_at = sim.elapsed;
            sim.contact_bit = sim.position == 0 ? 1u : 2u;
        }
    }
    timer.SR |= TIM_FLAG_CC1;
    if (irq_mask == 0u) {
        /* HAL_TIM_IRQHandler clears CC1IF before delivering the callback. */
        timer.SR &= ~TIM_FLAG_CC1;
        motor_pulse_finished_isr(active_motor);
    }
}

static void advance_timer_ticks(uint32_t remaining)
{
    while (sim.running && (timer.CR1 & TIM_CR1_CEN) != 0u &&
           !sim.frozen_timer && remaining > 0u) {
        if (timer.EGR & TIM_EGR_UG) {
            timer.CNT = 0u;
            timer.EGR = 0u;
            timer.SR |= TIM_FLAG_UPDATE;
            sim.pulse_done = 0;
        }
        uint32_t next = sim.pulse_done ? timer.ARR + 1u : timer.CCR1;
        if (next <= timer.CNT) {
            fprintf(stderr, "Invalid timer schedule CNT=%u next=%u\n", timer.CNT, next);
            abort();
        }
        uint32_t delta = next - timer.CNT;
        if (delta > remaining) {
            timer.CNT += remaining;
            remaining = 0u;
        } else {
            timer.CNT += delta;
            remaining -= delta;
            if (sim.pulse_done) {
                timer.CNT = 0u;
                timer.SR |= TIM_FLAG_UPDATE;
                sim.pulse_done = 0;
            } else {
                sim.pulse_done = 1;
                pulse_complete();
            }
        }
    }
}

uint32_t fake_get_primask(void) { return irq_mask; }
void fake_set_primask(uint32_t mask)
{
    if (mask != 0u && irq_mask == 0u && sim.inject_limit_bounce) {
        /* Inject a compare IRQ immediately before the final restart critical
         * section. The contact trips in the ISR, then bounces released before
         * the foreground rereads GPIO. Only the latched hit can stop a restart. */
        sim.inject_limit_bounce = 0;
        timer.CNT = timer.CCR1;
        sim.pulse_done = 1;
        pulse_complete();
        sim.force_release_b = 1;
    }
    irq_mask = mask;
    if (irq_mask == 0u && sim.running && (timer.SR & TIM_FLAG_CC1) != 0u) {
        timer.SR &= ~TIM_FLAG_CC1;
        motor_pulse_finished_isr(active_motor);
    }
}
void fake_nop(void)
{
    fractional_tick += 84u;
    if (fractional_tick >= 180u) {
        fractional_tick -= 180u;
        advance_timer_ticks(1u);
    }
}

static void advance_one_ms(void)
{
    advance_timer_ticks(84000u);
    ++sim.tick;
    ++sim.elapsed;
    if (sim.elapsed >= WATCHDOG_MS) longjmp(watchdog, 1);
    if (sim.cancel && sim.elapsed >= 200u) {
        sim.cancel = 0;
        motor_homing_cancel(active_motor);
    }
    if (active_motor && fabsf(active_motor->fcurrent) > sim.max_observed_speed)
        sim.max_observed_speed = fabsf(active_motor->fcurrent);
    unsigned switches = pressed_switches();
    if (switches != sim.last_switches) {
        printf("  t=%5u ms: switches A=%u B=%u, cart=%d\n", sim.elapsed,
               switches & 1u, (switches >> 1) & 1u, sim.position);
        sim.last_switches = switches;
    }
}

uint32_t HAL_GetTick(void)
{
    return sim.tick;
}

void HAL_Delay(uint32_t milliseconds)
{
    /* Real STM32 HAL adds one tick to a requested nonzero delay. The jitter
     * scenario varies additional latency; the firmware integrates elapsed time. */
    if (milliseconds > 0u) ++milliseconds;
    if (sim.delayed_poll) milliseconds += (sim.delay_calls++ % 3u);
    while (milliseconds--) advance_one_ms();
}

static int driver_checks(motor_drv_t *md)
{
    if (strcmp(sim.name, "command_nan") == 0) {
        motor_acc(NAN, md);
        motor_acc(1000.0f, md);
        return md->fault == MOTOR_FAULT_COMMAND && !sim.running && sim.emitted_steps == 0u;
    }
    if (strcmp(sim.name, "latched_limit_bounce") == 0) {
        sim.position = TRACK_STEPS - 2;
        md->homing_active = 1u;
        md->pulse_limit_stop_mask = 3u;
        motor_acc(1000.0f, md);
        HAL_Delay(1u); /* First completed pulse stops one step short of B. */
        sim.inject_limit_bounce = 1;
        motor_acc(1000.0f, md); /* IRQ injection occurs inside this call. */
        unsigned before = sim.emitted_steps;
        motor_acc(1000.0f, md); /* A later call must honor the same latch too. */
        HAL_Delay(2u);
        int ok = !sim.running && before == 2u && sim.emitted_steps == before &&
                 sim.into_endstop == 0u && md->pulse_limit_hit == MOTOR_LIMIT_B &&
                 md->fault == MOTOR_FAULT_NONE;
        md->homing_active = 0u;
        motor_stop(md);
        return ok;
    }
    if (strcmp(sim.name, "direction_reversal") == 0) {
        motor_acc(2000.0f, md);
        motor_acc(-2000.0f, md);
        motor_acc(-2000.0f, md);
        HAL_Delay(1u);
        motor_stop(md);
        return md->emitted_steps == 2u && sim.emitted_steps == 2u &&
               motor_position_steps(md) == 0 && sim.position == TRACK_STEPS / 2 &&
               md->drv->dir == DIRECTION_CCW;
    }
    if (strcmp(sim.name, "command_bounds") == 0) {
        motor_acc(1000000.0f, md);
        int acceleration_ok = fabsf(md->fcurrent - 2.0f) < 0.0001f;
        for (unsigned i = 0u; i < 3000u; ++i) motor_acc(1000000.0f, md);
        int ok = acceleration_ok && md->fcurrent == md->motor->fmax;
        motor_stop(md);
        return ok && md->fcurrent == 0.0f && !sim.running;
    }
    if (strcmp(sim.name, "zero_acc_holds_velocity") == 0) {
        motor_acc(1000.0f, md);
        float before = md->fcurrent;
        motor_acc(0.0f, md);
        int ok = md->fcurrent == before && before > 0.0f;
        motor_stop(md);
        return ok && md->fcurrent == 0.0f;
    }
    if (strcmp(sim.name, "pending_pulse") == 0) {
        /* A compare happened with interrupts masked: stop must drain it once. */
        motor_acc(1000.0f, md);
        timer.CNT = timer.CCR1;
        sim.pulse_done = 1;
        sim.emitted_steps++;
        sim.position++;
        timer.SR |= TIM_FLAG_CC1 | TIM_FLAG_UPDATE;
        motor_stop(md);
        motor_stop(md);
        return md->emitted_steps == 1u && motor_position_steps(md) == 1 && !sim.running;
    }
    if (strcmp(sim.name, "ug_not_a_step") == 0) {
        for (unsigned i = 0u; i < 20u; ++i) {
            /* A software update while output is stopped is not a STEP. */
            timer.EGR = TIM_EGR_UG;
            timer.SR |= TIM_FLAG_UPDATE;
            motor_stop(md);
        }
        return md->emitted_steps == 0u && motor_position_steps(md) == 0;
    }
    if (strcmp(sim.name, "runtime_limit") == 0) {
        sim.position = TRACK_STEPS - 5;
        for (unsigned i = 0; i < 1000u && md->fault == MOTOR_FAULT_NONE; ++i) {
            motor_acc(1000.0f, md);
            HAL_Delay(1u);
        }
        unsigned count = sim.emitted_steps;
        motor_acc(-1000.0f, md);
        HAL_Delay(10u);
        return md->fault == MOTOR_FAULT_LIMIT && !sim.running &&
               count == sim.emitted_steps && sim.into_endstop == 0u;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    sim.name = argv[1];
    sim.track_steps = 400;
    sim.position = TRACK_STEPS / 2;
    sim.cw_at_b = 1;
    volatile int expected = EXPECT_CENTRE;
    int wrong_mapping = 0;
    int wrong_polarity = 0;
    int invalid_config = 0;
    if (strcmp(sim.name, "middle") == 0) { }
    else if (strcmp(sim.name, "at_a") == 0) sim.position = 0;
    else if (strcmp(sim.name, "at_b") == 0) sim.position = TRACK_STEPS;
    else if (strcmp(sim.name, "reversed_mapping") == 0) sim.cw_at_b = 0;
    else if (strcmp(sim.name, "reversed_at_a") == 0) { sim.cw_at_b = 0; sim.position = 0; }
    else if (strcmp(sim.name, "reversed_at_b") == 0) { sim.cw_at_b = 0; sim.position = TRACK_STEPS; }
    else if (strcmp(sim.name, "odd_track") == 0) sim.track_steps = 401;
    else if (strcmp(sim.name, "short_track") == 0) {
        sim.track_steps = 250; sim.position = 125; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "long_track") == 0) {
        sim.track_steps = 600; sim.position = 300; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "timer_start_failure") == 0) {
        sim.fail_start = 1; expected = EXPECT_REJECT_WITHOUT_MOVING;
    } else if (strcmp(sim.name, "both_pressed") == 0) {
        sim.both_pressed = 1; expected = EXPECT_REJECT_WITHOUT_MOVING;
    } else if (strcmp(sim.name, "a_stuck_pressed") == 0) {
        sim.position = 0; sim.stuck_a = 1; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "b_never_pressed") == 0) {
        sim.missing_b = 1; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "timer_stalled") == 0) {
        sim.frozen_timer = 1; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "active_low") == 0) sim.a_low = sim.b_low = 1;
    else if (strcmp(sim.name, "mixed_polarity") == 0) sim.a_low = 1;
    else if (strcmp(sim.name, "polarity_mismatch") == 0) {
        sim.a_low = sim.b_low = 1; wrong_polarity = 1; expected = EXPECT_REJECT_WITHOUT_MOVING;
    } else if (strcmp(sim.name, "wrong_mapping") == 0) {
        wrong_mapping = 1; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "invalid_config") == 0) {
        invalid_config = 1; expected = EXPECT_REJECT_WITHOUT_MOVING;
    } else if (strcmp(sim.name, "cancel") == 0) {
        sim.cancel = 1; expected = EXPECT_FAULT;
    } else if (strcmp(sim.name, "cancel_before_start") == 0) {
        expected = EXPECT_REJECT_WITHOUT_MOVING;
    } else if (strcmp(sim.name, "delayed_poll") == 0) sim.delayed_poll = 1;
    else if (strcmp(sim.name, "switch_bounce") == 0) { sim.bounce = 1; sim.delayed_poll = 1; }
    else if (strcmp(sim.name, "tick_wrap") == 0) sim.tick = UINT32_MAX - 50u;
    else if (strcmp(sim.name, "command_nan") == 0 || strcmp(sim.name, "command_bounds") == 0 ||
             strcmp(sim.name, "pending_pulse") == 0 || strcmp(sim.name, "ug_not_a_step") == 0 ||
             strcmp(sim.name, "runtime_limit") == 0 || strcmp(sim.name, "zero_acc_holds_velocity") == 0 ||
             strcmp(sim.name, "direction_reversal") == 0 || strcmp(sim.name, "latched_limit_bounce") == 0)
        expected = EXPECT_DRIVER;
    else { fprintf(stderr, "Unknown case: %s\n", sim.name); return 2; }

    TIM_HandleTypeDef htim = { .Instance = &timer };
    motor_t motor;
    drv_t drv;
    motor_drv_t md = {0};
    drv_init(&drv, DRV8825_FULL_STEP, GPIOC, 1u << 6,
             GPIOB, 1u << 15, GPIOB, 1u << 14,
             GPIOB, 1u << 12, GPIOB, 1u << 13);
    motor_init(&motor, 200, &htim, TIM_CHANNEL_1, 1000.0f, 84000000.0f, 1000.0f);
    motor_drv_init(&md, &drv, &motor);
    active_motor = &md;
    if (!motor_set_limits(&md, 1000.0f, 2000.0f)) return 2;
    motor_homing_config_t config = motor_homing_default_config();
    config.a_pressed_level = sim.a_low && !wrong_polarity ? GPIO_PIN_RESET : GPIO_PIN_SET;
    config.b_pressed_level = sim.b_low && !wrong_polarity ? GPIO_PIN_RESET : GPIO_PIN_SET;
    config.cw_limit = (sim.cw_at_b != wrong_mapping) ? MOTOR_LIMIT_B : MOTOR_LIMIT_A;
    config.max_release_steps = 40u;
    config.min_travel_steps = invalid_config ? 600u : 300u;
    config.max_travel_steps = 500u;
    config.release_timeout_ms = 1000u;
    config.phase_timeout_ms = 6000u;
    config.total_timeout_ms = 20000u;
    (void)motor_configure_homing(&md, &config);
    if (strcmp(sim.name, "cancel_before_start") == 0) motor_homing_cancel(&md);
    sim.last_switches = pressed_switches();
    printf("CASE %s: cart=%d, raw GPIO A=%d B=%d, CW toward %c\n", sim.name, sim.position,
           HAL_GPIO_ReadPin(GPIOC, LIMIT_A_Pin), HAL_GPIO_ReadPin(GPIOC, LIMIT_B_Pin),
           sim.cw_at_b ? 'B' : 'A');
    volatile uint32_t travel = 0;
    volatile int passed = 0;
    if (setjmp(watchdog) == 0) {
        if (expected == EXPECT_DRIVER) passed = driver_checks(&md);
        else travel = motor_homing(&md);
    } else sim.overrun = 1;
    if (expected != EXPECT_DRIVER) {
        passed = !sim.overrun && !sim.running;
        if (expected == EXPECT_CENTRE) {
            passed = passed && travel == (uint32_t)TRACK_STEPS &&
                     abs(sim.position - TRACK_STEPS / 2) <= 1 && sim.into_endstop == 0 &&
                     md.homed && motor_position_steps(&md) == 0 && md.fault == MOTOR_FAULT_NONE &&
                     sim.max_observed_speed <= config.speed + 0.01f &&
                     md.emitted_steps == sim.emitted_steps;
        } else {
            passed = passed && travel == 0 && md.fault != MOTOR_FAULT_NONE && !md.homed;
            if (expected == EXPECT_REJECT_WITHOUT_MOVING)
                passed = passed && sim.emitted_steps == 0;
            unsigned before = sim.emitted_steps;
            motor_acc(1000.0f, &md);
            HAL_Delay(5u);
            passed = passed && !sim.running && sim.emitted_steps == before;
        }
    }
    printf("%s %s: travel=%u cart=%d counted=%u emitted=%u fault=%d time=%u ms "
           "into_endstop=%u PWM=%s%s\n", passed ? "PASS" : "FAIL", sim.name,
           (unsigned)travel, sim.position, (unsigned)md.emitted_steps, sim.emitted_steps,
           md.fault, sim.elapsed, sim.into_endstop, sim.running ? "ON" : "OFF",
           sim.overrun ? "; watchdog expired" : "");
    return passed ? 0 : 1;
}
