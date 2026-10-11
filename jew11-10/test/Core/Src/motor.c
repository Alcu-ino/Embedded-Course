/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "motor.h"

/* At least 1 us even counting only the NOP instructions, using the actual
 * core clock. Exceeds DRV8825 DIR setup and hold requirements (650 ns). */
#define DIR_SETUP_DELAY() do { \
    uint32_t remaining = SystemCoreClock / 1000000u + 1u; \
    while (remaining--) { __NOP(); } \
} while (0)

/* ========================================================================== */
/*  DRV8825                                                                   */
/* ========================================================================== */
void drv_init(drv_t *drv, drv_microstep_t microsteps,
              GPIO_TypeDef *m0_port, uint16_t m0_pin,
              GPIO_TypeDef *m1_port, uint16_t m1_pin,
              GPIO_TypeDef *m2_port, uint16_t m2_pin,
              GPIO_TypeDef *dir_port, uint16_t dir_pin,
              GPIO_TypeDef *rst_slp_port, uint16_t rst_slp_pin)
{
    drv->m0_port = m0_port;           drv->m0_pin = m0_pin;
    drv->m1_port = m1_port;           drv->m1_pin = m1_pin;
    drv->m2_port = m2_port;           drv->m2_pin = m2_pin;
    drv->dir_port = dir_port;         drv->dir_pin = dir_pin;
    drv->rst_slp_port = rst_slp_port; drv->rst_slp_pin = rst_slp_pin;

    drv_set_microsteps(drv, microsteps);

    /* Pin e campo dir allineati fin dall'inizio: scrittura diretta,
       non tramite drv_set_direction (che salterebbe se il campo coincide) */
    drv->dir = DIRECTION_CW;
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);   /* CW = pin basso */

    HAL_GPIO_WritePin(rst_slp_port, rst_slp_pin, GPIO_PIN_SET); /* esce da reset/sleep */
    HAL_Delay(2);                                               /* t_WAKE max 1.7 ms   */
}

void drv_set_microsteps(drv_t *drv, drv_microstep_t microsteps)
{
    /* Tabella DRV8825 (M2 M1 M0): 000=1, 001=1/2, 010=1/4, 011=1/8, 100=1/16, 101=1/32 */
    uint8_t code;
    switch (microsteps) {
        case DRV8825_FULL_STEP: code = 0u; break;
        case DRV8825_HALF_STEP: code = 1u; break;
        case DRV8825_4_STEPS:   code = 2u; break;
        case DRV8825_8_STEPS:   code = 3u; break;
        case DRV8825_16_STEPS:  code = 4u; break;
        case DRV8825_32_STEPS:  code = 5u; break;
        default:                code = 0u; microsteps = DRV8825_FULL_STEP; break;
    }
    drv->microsteps = microsteps;

    HAL_GPIO_WritePin(drv->m0_port, drv->m0_pin, (code & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m1_port, drv->m1_pin, (code & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drv->m2_port, drv->m2_pin, (code & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void drv_set_direction(drv_t *drv, direction_t dir)
{
    /* Aggiorna sempre pin e campo insieme: il campo resta la copia fedele del pin */
    HAL_GPIO_WritePin(drv->dir_port, drv->dir_pin,
                      (dir == DIRECTION_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    drv->dir = dir;
}

/* ========================================================================== */
/* STEP generation and accounting                                            */
/* ========================================================================== */
#include <limits.h>
#include <string.h>

/* Keep interrupt load and the shortest period bounded. This must only be
 * increased after measuring interrupt latency with the complete application. */
#define MOTOR_MAX_IRQ_FREQUENCY 10000.0f
#define MOTOR_DEFAULT_AMAX     50000.0f

static uint32_t critical_enter(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static void critical_leave(uint32_t mask)
{
    __set_PRIMASK(mask);
}

static void count_pulse(motor_drv_t *md)
{
    int32_t increment = md->drv->dir == DIRECTION_CW ? 1 : -1;
    if ((increment > 0 && md->step_position == INT32_MAX) ||
        (increment < 0 && md->step_position == INT32_MIN)) {
        md->fault = MOTOR_FAULT_TRAVEL;
        return;
    }
    md->step_position += increment;
    md->emitted_steps++;
}

/* Stop with STEP low. If called during the 2.5 us high pulse, let that pulse
 * finish and account for its pending compare interrupt before disabling it.
 * The loop is cycle-bounded even if the timer clock has stopped. This cannot
 * establish that a mechanical step actually happened (there is no cart encoder).
 * Critical sections are shorter than a period at the enforced 10 kHz ceiling. */
static void stop_pwm(motor_drv_t *md)
{
    TIM_HandleTypeDef *htim = md->motor->htim_PWM;
    uint32_t mask = critical_enter();
    uint32_t was_enabled = htim->Instance->CR1 & TIM_CR1_CEN;
    htim->Instance->CR1 &= ~TIM_CR1_CEN;
    if (md->pwm_on) {
        if (htim->Instance->CNT < htim->Instance->CCR1 && was_enabled != 0u) {
            /* Freeze first to avoid a fresh rising edge between a LOW test and
             * stopping the channel. Resume only to finish an already HIGH pulse. */
            htim->Instance->CR1 |= TIM_CR1_CEN;
            uint32_t budget = (SystemCoreClock / 1000000u + 1u) * 3u;
            while (htim->Instance->CNT < htim->Instance->CCR1 && budget > 0u) {
                __NOP();
                --budget;
            }
            htim->Instance->CR1 &= ~TIM_CR1_CEN;
            if (htim->Instance->CNT < htim->Instance->CCR1 && md->fault == MOTOR_FAULT_NONE)
                md->fault = MOTOR_FAULT_TIMER;
        }
        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
            count_pulse(md);
        }
    }
    HAL_TIM_PWM_Stop_IT(htim, md->motor->tim_channel);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
    md->pwm_on = 0u;
    critical_leave(mask);
}

void motor_stop(motor_drv_t *md)
{
    stop_pwm(md);
    md->fcurrent = 0.0f;
    md->state = MOTOR_DRV_STOPPED;
    md->position = (float)motor_position_steps(md);
    if (md->fault != MOTOR_FAULT_NONE) md->homed = 0u;
}

static void motor_fail(motor_drv_t *md, motor_fault_t fault)
{
    if (md->fault == MOTOR_FAULT_NONE) md->fault = fault;
    md->homed = 0u;
    motor_stop(md);
}

void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *htim,
                uint32_t tim_channel, float fmax, float fclk, float fs)
{
    motor->steps_per_rev = steps_per_rev;
    motor->htim_PWM = htim;
    motor->tim_channel = tim_channel;
    motor->fclk = fclk;
    motor->fmax = fmax;
    motor->fs = fs;
    motor->amax = MOTOR_DEFAULT_AMAX;
    HAL_TIM_PWM_Stop_IT(htim, tim_channel);
    /* Immediate ARR updates prevent the first low-frequency period from
     * delaying an acceleration ramp. motor_acc handles CNT >= new ARR before
     * lowering ARR. CCR is fixed, not a 50% duty-cycle STEP signal. */
    htim->Instance->CR1 &= ~TIM_CR1_ARPE;
    __HAL_TIM_DISABLE_OCxPRELOAD(htim, tim_channel);
    __HAL_TIM_SET_COMPARE(htim, tim_channel, 0u);
    htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
}

void motor_drv_init(motor_drv_t *md, drv_t *drv, motor_t *motor)
{
    memset(md, 0, sizeof(*md));
    md->drv = drv;
    md->motor = motor;
    md->state = MOTOR_DRV_STOPPED;
    if (motor->tim_channel != TIM_CHANNEL_1 || !isfinite(motor->fclk) ||
        motor->fclk <= 0.0f || !isfinite(motor->fs) || motor->fs <= 0.0f ||
        !isfinite(motor->fmax) || motor->fmax < MOTOR_FMIN_PRATICA ||
        motor->fmax > MOTOR_MAX_IRQ_FREQUENCY) md->fault = MOTOR_FAULT_CONFIG;
}

int32_t motor_position_steps(const motor_drv_t *md)
{
    /* A naturally aligned 32-bit load is atomic on the Cortex-M4. */
    return md->step_position;
}

int motor_set_limits(motor_drv_t *md, float max_speed, float max_acceleration)
{
    if (md->pwm_on || md->homing_active || !isfinite(max_speed) ||
        !isfinite(max_acceleration) || max_speed < MOTOR_FMIN_PRATICA ||
        max_speed > MOTOR_MAX_IRQ_FREQUENCY || max_acceleration <= 0.0f)
        return 0;
    md->motor->fmax = max_speed;
    md->motor->amax = max_acceleration;
    return 1;
}

uint8_t motor_read_limits(const motor_drv_t *md)
{
    if (!md->homing_configured) return 0u;
    uint8_t state = 0u;
    if (HAL_GPIO_ReadPin(LIMIT_A_GPIO_Port, LIMIT_A_Pin) == md->homing.a_pressed_level)
        state |= MOTOR_LIMIT_A;
    if (HAL_GPIO_ReadPin(LIMIT_B_GPIO_Port, LIMIT_B_Pin) == md->homing.b_pressed_level)
        state |= MOTOR_LIMIT_B;
    return state;
}

int motor_clear_fault(motor_drv_t *md)
{
    if (md->pwm_on || md->homing_active || !md->homing_configured ||
        motor_read_limits(md) == (MOTOR_LIMIT_A | MOTOR_LIMIT_B)) return 0;
    md->fault = MOTOR_FAULT_NONE;
    md->homed = 0u;
    md->homing_cancel_requested = 0u;
    return 1;
}

void motor_homing_cancel(motor_drv_t *md)
{
    md->homing_cancel_requested = 1u;
    motor_fail(md, MOTOR_FAULT_CANCELLED);
}

void motor_pulse_finished_isr(motor_drv_t *md)
{
    if (!md->pwm_on) return;
    count_pulse(md);
    /* This callback occurs at the falling edge of the fixed-width PWM1 pulse.
     * GPIO reads and integer comparisons stop before the next scheduled STEP.
     * No floating-point calculations are required in the normal ISR path. */
    uint8_t limits = motor_read_limits(md);
    uint8_t watched = md->homing_active ? md->pulse_limit_stop_mask : 3u;
    md->pulse_limit_hit |= (uint8_t)(limits & watched);
    if (md->fault != MOTOR_FAULT_NONE || (limits & watched) != 0u ||
        (md->pulse_stop_enabled && md->emitted_steps == md->pulse_stop_at)) {
        if (!md->homing_active && limits != 0u)
            md->fault = limits == 3u ? MOTOR_FAULT_BOTH_LIMITS : MOTOR_FAULT_LIMIT;
        /* Already in the low phase; HAL stops channel + CC interrupt + counter. */
        HAL_TIM_PWM_Stop_IT(md->motor->htim_PWM, md->motor->tim_channel);
        md->pwm_on = 0u;
        md->fcurrent = 0.0f;
        md->state = MOTOR_DRV_STOPPED;
        if (md->fault != MOTOR_FAULT_NONE) md->homed = 0u;
    }
}

static void motor_acc_dt(float acc, float dt, motor_drv_t *md)
{
    if (md->fault != MOTOR_FAULT_NONE) { motor_stop(md); return; }
    if (!md->homing_configured) { motor_fail(md, MOTOR_FAULT_CONFIG); return; }
    if (!isfinite(acc)) { motor_fail(md, MOTOR_FAULT_COMMAND); return; }
    if (md->homing_active &&
        (md->pulse_limit_hit != 0u ||
         (motor_read_limits(md) & md->pulse_limit_stop_mask) != 0u ||
         (md->pulse_stop_enabled && md->emitted_steps == md->pulse_stop_at))) {
        md->pulse_limit_hit |= (uint8_t)(motor_read_limits(md) & md->pulse_limit_stop_mask);
        motor_stop(md);
        return;
    }
    if (!md->homing_active && motor_read_limits(md) != 0u) {
        motor_fail(md, motor_read_limits(md) == 3u ?
                   MOTOR_FAULT_BOTH_LIMITS : MOTOR_FAULT_LIMIT);
        return;
    }
    motor_t *motor = md->motor;
    TIM_HandleTypeDef *htim = motor->htim_PWM;
    TIM_TypeDef *tim = htim->Instance;
    if (acc > motor->amax) acc = motor->amax;
    if (acc < -motor->amax) acc = -motor->amax;
    float v = md->fcurrent + acc * dt;
    if (!isfinite(v)) { motor_fail(md, MOTOR_FAULT_COMMAND); return; }
    if (v > motor->fmax) v = motor->fmax;
    if (v < -motor->fmax) v = -motor->fmax;
    md->fcurrent = v;
    md->position = (float)motor_position_steps(md);
    float frequency = fabsf(v);
    float timer_hz = motor->fclk / ((float)tim->PSC + 1.0f);
    float minimum = timer_hz / 4294967295.0f;
    if (minimum < MOTOR_FMIN_PRATICA) minimum = MOTOR_FMIN_PRATICA;
    if (frequency < minimum) {
        stop_pwm(md);
        md->state = MOTOR_DRV_STOPPED;
        /* Preserve the sub-step/s command so an acceleration ramp can build. */
        return;
    }
    direction_t direction = v > 0.0f ? DIRECTION_CW : DIRECTION_CCW;
    if (direction != md->drv->dir) {
        stop_pwm(md);
        /* stop_pwm completed any STEP high time; delay also provides DIR hold. */
        DIR_SETUP_DELAY();
        drv_set_direction(md->drv, direction);
        DIR_SETUP_DELAY();
    }
    uint32_t pulse = (uint32_t)(STEP_PULSE_S * timer_hz) + 1u;
    uint32_t period = (uint32_t)(timer_hz / frequency);
    if (period < 2u * pulse) period = 2u * pulse;
    uint32_t mask = critical_enter();
    /* A limit/cancel interrupt may have stopped us before entering this section. */
    if (md->fault != MOTOR_FAULT_NONE ||
        (md->homing_active && (md->homing_cancel_requested || md->pulse_limit_hit != 0u ||
         (motor_read_limits(md) & md->pulse_limit_stop_mask) != 0u ||
         (md->pulse_stop_enabled && md->emitted_steps == md->pulse_stop_at)))) {
        critical_leave(mask);
        motor_stop(md);
        return;
    }
    if (!md->pwm_on) {
        __HAL_TIM_SET_AUTORELOAD(htim, period - 1u);
        __HAL_TIM_SET_COMPARE(htim, motor->tim_channel, pulse);
        tim->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
        md->pwm_on = 1u;
        if (HAL_TIM_PWM_Start_IT(htim, motor->tim_channel) != HAL_OK) {
            md->pwm_on = 0u;
            critical_leave(mask);
            motor_fail(md, MOTOR_FAULT_TIMER);
            return;
        }
    } else {
        /* ARR has no preload: a shorter period applies immediately. If CNT has
         * already passed it, restart only from the LOW phase. Software UG is
         * never counted as a step; the subsequent real compare edge is. */
        /* Masking CPU interrupts does not stop hardware CNT. Freeze CEN for
         * the read/write pair so CNT cannot cross a newly shortened ARR. */
        tim->CR1 &= ~TIM_CR1_CEN;
        if (tim->CNT >= period - 1u) {
            if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
                __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
                motor_pulse_finished_isr(md);
                if (!md->pwm_on) {
                    critical_leave(mask);
                    return;
                }
            }
            __HAL_TIM_SET_AUTORELOAD(htim, period - 1u);
            tim->EGR = TIM_EGR_UG;
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
        } else {
            __HAL_TIM_SET_AUTORELOAD(htim, period - 1u);
        }
        if (md->pwm_on) tim->CR1 |= TIM_CR1_CEN;
    }
    md->state = (acc == 0.0f) ? MOTOR_DRV_RUN :
                (v * acc >= 0.0f ? MOTOR_DRV_ACCEL : MOTOR_DRV_DECEL);
    critical_leave(mask);
}

void motor_acc(float acc, motor_drv_t *md)
{
    motor_acc_dt(acc, 1.0f / md->motor->fs, md);
}

/* ========================================================================== */
/* Bounded homing: acquire, release/relatch both ends, measure, return to center */
/* ========================================================================== */
motor_homing_config_t motor_homing_default_config(void)
{
    /* Provisional values only. The application must prohibit arming until the
     * real switch levels, direction, travel, drive and encoder are verified. */
    motor_homing_config_t config = {
        .a_pressed_level = GPIO_PIN_SET, .b_pressed_level = GPIO_PIN_SET,
        .cw_limit = MOTOR_LIMIT_B, .speed = 200.0f, .latch_speed = 100.0f,
        .acceleration = 1000.0f, .debounce_ms = 5u, .backoff_steps = 10u,
        .max_release_steps = 100u, .min_travel_steps = 200u,
        .max_travel_steps = 20000u, .release_timeout_ms = 5000u,
        .phase_timeout_ms = 120000u, .total_timeout_ms = 600000u
    };
    return config;
}

int motor_configure_homing(motor_drv_t *md, const motor_homing_config_t *c)
{
    if (c == NULL || md->pwm_on || md->homing_active ||
        (c->a_pressed_level != GPIO_PIN_SET && c->a_pressed_level != GPIO_PIN_RESET) ||
        (c->b_pressed_level != GPIO_PIN_SET && c->b_pressed_level != GPIO_PIN_RESET) ||
        (c->cw_limit != MOTOR_LIMIT_A && c->cw_limit != MOTOR_LIMIT_B) ||
        !isfinite(c->speed) || !isfinite(c->latch_speed) || !isfinite(c->acceleration) ||
        c->speed < MOTOR_FMIN_PRATICA || c->speed > md->motor->fmax ||
        c->latch_speed < MOTOR_FMIN_PRATICA || c->latch_speed > c->speed ||
        c->acceleration <= 0.0f || c->acceleration > md->motor->amax ||
        c->debounce_ms < 1u || c->debounce_ms > 100u || c->backoff_steps < 1u ||
        c->max_release_steps <= c->backoff_steps || c->min_travel_steps < 4u ||
        c->max_release_steps >= c->min_travel_steps ||
        c->max_travel_steps < c->min_travel_steps ||
        c->max_travel_steps > (uint32_t)INT32_MAX / 4u ||
        c->release_timeout_ms <= c->debounce_ms ||
        c->phase_timeout_ms < c->release_timeout_ms ||
        c->total_timeout_ms < c->phase_timeout_ms ||
        c->total_timeout_ms >= 0x80000000u) {
        motor_fail(md, MOTOR_FAULT_CONFIG);
        return 0;
    }
    md->homing = *c;
    md->homing_configured = 1u;
    md->homed = 0u;
    /* Configuration alone does not clear a previously latched fault. */
    return 1;
}

static direction_t toward(const motor_drv_t *md, uint8_t endpoint)
{
    return endpoint == md->homing.cw_limit ? DIRECTION_CW : DIRECTION_CCW;
}

static int home_check(motor_drv_t *md, uint32_t total_start,
                      uint32_t phase_start, uint32_t timeout)
{
    if (md->fault != MOTOR_FAULT_NONE) return 0;
    if (md->homing_cancel_requested) {
        motor_fail(md, MOTOR_FAULT_CANCELLED);
        return 0;
    }
    if (motor_read_limits(md) == 3u) {
        motor_fail(md, MOTOR_FAULT_BOTH_LIMITS);
        return 0;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - total_start) >= md->homing.total_timeout_ms ||
        (uint32_t)(now - phase_start) >= timeout) {
        motor_fail(md, MOTOR_FAULT_TIMEOUT);
        return 0;
    }
    return 1;
}

static void home_ramp(motor_drv_t *md, direction_t dir, float speed,
                       uint32_t *last_update)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - *last_update;
    *last_update = now;
    if (elapsed == 0u) return;
    /* HAL_Delay(1) usually waits two SysTicks. Integrate measured elapsed time,
     * not an assumed 1 ms. A long scheduling pause never causes a catch-up jump. */
    if (elapsed > 20u) elapsed = 20u;
    float dt = (float)elapsed * 0.001f;
    float target = dir == DIRECTION_CW ? speed : -speed;
    float acc = (target - md->fcurrent) / dt;
    if (acc > md->homing.acceleration) acc = md->homing.acceleration;
    if (acc < -md->homing.acceleration) acc = -md->homing.acceleration;
    motor_acc_dt(acc, dt, md);
}

/* A contact stops motion immediately. Debounce happens only while stopped;
 * a bounce restarts the same bounded approach instead of driving into a switch
 * for the entire debounce period. Departure must release within both bounds. */
static int home_seek(motor_drv_t *md, uint8_t target, uint8_t departure,
                     float speed, uint32_t total_start)
{
    motor_stop(md);
    uint32_t phase_start = HAL_GetTick();
    uint32_t last_update = phase_start;
    uint32_t first_step = md->emitted_steps;
    uint32_t release_since = phase_start;
    uint32_t hit_since = phase_start;
    uint8_t releasing = departure != 0u;
    uint8_t release_candidate = 0u;
    uint8_t hit_candidate = 0u;
    uint8_t hit_level = 0u;
    md->pulse_limit_hit = 0u;
    md->pulse_limit_stop_mask = (uint8_t)(3u & ~departure);
    md->pulse_stop_at = first_step + md->homing.max_travel_steps;
    md->pulse_stop_enabled = 1u;
    while (home_check(md, total_start, phase_start, md->homing.phase_timeout_ms)) {
        uint32_t now = HAL_GetTick();
        uint8_t limits = motor_read_limits(md);
        uint32_t steps = md->emitted_steps - first_step;
        if (steps >= md->homing.max_travel_steps && (limits & target) == 0u) {
            motor_fail(md, MOTOR_FAULT_TRAVEL); return 0;
        }
        if (releasing) {
            if (steps >= md->homing.max_release_steps ||
                (uint32_t)(now - phase_start) >= md->homing.release_timeout_ms) {
                motor_fail(md, MOTOR_FAULT_RELEASE); return 0;
            }
            if ((limits & departure) == 0u) {
                if (!release_candidate) { release_candidate = 1u; release_since = now; }
                if ((uint32_t)(now - release_since) >= md->homing.debounce_ms) {
                    releasing = 0u;
                    md->pulse_limit_stop_mask = 3u;
                }
            } else release_candidate = 0u;
        } else if (((limits | md->pulse_limit_hit) & (uint8_t)(3u & ~target)) != 0u) {
            motor_fail(md, MOTOR_FAULT_UNEXPECTED_LIMIT); return 0;
        }
        if ((limits & target) != 0u || (md->pulse_limit_hit & target) != 0u) {
            motor_stop(md);
            md->pulse_limit_hit = 0u;
            if (releasing) { motor_fail(md, MOTOR_FAULT_TRAVEL); return 0; }
            if (!hit_candidate) {
                hit_candidate = 1u;
                hit_level = (limits & target) != 0u;
                hit_since = now;
            }
        }
        if (hit_candidate) {
            /* Stay stopped through both edges of switch bounce. Resume only
             * after a stable RELEASE, accept only a stable PRESSED reading. */
            uint8_t level = (limits & target) != 0u;
            if (level != hit_level) { hit_level = level; hit_since = now; }
            if ((uint32_t)(now - hit_since) >= md->homing.debounce_ms) {
                if (level) { md->pulse_stop_enabled = 0u; return 1; }
                hit_candidate = 0u;
            }
        }
        if (!hit_candidate) home_ramp(md, toward(md, target), speed, &last_update);
        HAL_Delay(1u);
    }
    return 0;
}

static int home_backoff(motor_drv_t *md, uint8_t endpoint, uint32_t total_start)
{
    motor_stop(md);
    uint32_t phase_start = HAL_GetTick();
    uint32_t last_update = phase_start;
    uint32_t first_step = md->emitted_steps;
    uint32_t release_since = phase_start;
    uint8_t release_candidate = 0u;
    uint8_t released = 0u;
    md->pulse_limit_hit = 0u;
    md->pulse_limit_stop_mask = (uint8_t)(3u & ~endpoint);
    md->pulse_stop_at = first_step + md->homing.max_release_steps;
    md->pulse_stop_enabled = 1u;
    while (home_check(md, total_start, phase_start, md->homing.release_timeout_ms)) {
        uint32_t now = HAL_GetTick();
        uint8_t limits = motor_read_limits(md);
        uint32_t steps = md->emitted_steps - first_step;
        if ((limits & (uint8_t)(3u & ~endpoint)) != 0u || md->pulse_limit_hit != 0u) {
            motor_fail(md, MOTOR_FAULT_UNEXPECTED_LIMIT); return 0;
        }
        if (steps >= md->homing.max_release_steps) {
            motor_fail(md, MOTOR_FAULT_RELEASE); return 0;
        }
        if ((limits & endpoint) == 0u) {
            if (!release_candidate) { release_candidate = 1u; release_since = now; }
            if (!released && (uint32_t)(now - release_since) >= md->homing.debounce_ms) {
                uint32_t mask = critical_enter();
                md->pulse_stop_at = md->emitted_steps + md->homing.backoff_steps;
                md->pulse_stop_enabled = 1u;
                md->pulse_limit_stop_mask = 3u;
                critical_leave(mask);
                released = 1u;
            }
        } else {
            if (released) { motor_fail(md, MOTOR_FAULT_UNEXPECTED_LIMIT); return 0; }
            release_candidate = 0u;
        }
        if (released && md->emitted_steps == md->pulse_stop_at) {
            motor_stop(md);
            md->pulse_stop_enabled = 0u;
            return 1;
        }
        home_ramp(md, toward(md, (uint8_t)(3u & ~endpoint)), md->homing.latch_speed, &last_update);
        HAL_Delay(1u);
    }
    return 0;
}

static int home_center(motor_drv_t *md, uint8_t departure,
                       uint32_t distance, uint32_t total_start)
{
    motor_stop(md);
    uint32_t phase_start = HAL_GetTick();
    uint32_t last_update = phase_start;
    uint32_t first_step = md->emitted_steps;
    uint32_t release_since = phase_start;
    uint8_t release_candidate = 0u;
    uint8_t released = 0u;
    md->pulse_limit_hit = 0u;
    md->pulse_stop_at = first_step + distance;
    md->pulse_stop_enabled = 1u;
    md->pulse_limit_stop_mask = (uint8_t)(3u & ~departure);
    while (home_check(md, total_start, phase_start, md->homing.phase_timeout_ms)) {
        uint32_t now = HAL_GetTick();
        uint8_t limits = motor_read_limits(md);
        uint32_t steps = md->emitted_steps - first_step;
        if ((limits & (uint8_t)(3u & ~departure)) != 0u ||
            (released && (limits & departure) != 0u) || md->pulse_limit_hit != 0u) {
            motor_fail(md, MOTOR_FAULT_UNEXPECTED_LIMIT); return 0;
        }
        if (!released) {
            if (steps >= md->homing.max_release_steps ||
                (uint32_t)(now - phase_start) >= md->homing.release_timeout_ms) {
                motor_fail(md, MOTOR_FAULT_RELEASE); return 0;
            }
            if ((limits & departure) == 0u) {
                if (!release_candidate) { release_candidate = 1u; release_since = now; }
                if ((uint32_t)(now - release_since) >= md->homing.debounce_ms) {
                    released = 1u;
                    md->pulse_limit_stop_mask = 3u;
                }
            } else release_candidate = 0u;
        }
        if (steps == distance) {
            motor_stop(md);
            md->pulse_stop_enabled = 0u;
            if (!released) { motor_fail(md, MOTOR_FAULT_RELEASE); return 0; }
            return 1;
        }
        home_ramp(md, toward(md, (uint8_t)(3u & ~departure)), md->homing.speed, &last_update);
        HAL_Delay(1u);
    }
    return 0;
}

uint32_t motor_homing(motor_drv_t *md)
{
    motor_stop(md);
    md->homed = 0u;
    md->travel_steps = 0u;
    if (!md->homing_configured) { motor_fail(md, MOTOR_FAULT_CONFIG); return 0u; }
    if (md->fault != MOTOR_FAULT_NONE) return 0u;
    if (md->homing_cancel_requested) { motor_fail(md, MOTOR_FAULT_CANCELLED); return 0u; }
    md->homing_active = 1u;
    uint32_t start = HAL_GetTick();
    uint8_t first = motor_read_limits(md);
    uint32_t travel = 0u;
    if (first == 3u) { motor_fail(md, MOTOR_FAULT_BOTH_LIMITS); goto done; }
    if (first == 0u) first = MOTOR_LIMIT_A;
    if (!home_seek(md, first, 0u, md->homing.speed, start)) goto done;
    if (!home_backoff(md, first, start)) goto done;
    if (!home_seek(md, first, 0u, md->homing.latch_speed, start)) goto done;
    int32_t first_position = motor_position_steps(md);
    uint8_t second = (uint8_t)(3u & ~first);
    if (!home_seek(md, second, first, md->homing.speed, start)) goto done;
    if (!home_backoff(md, second, start)) goto done;
    if (!home_seek(md, second, 0u, md->homing.latch_speed, start)) goto done;
    int64_t span = (int64_t)motor_position_steps(md) - first_position;
    if (span < 0) span = -span;
    if (span < md->homing.min_travel_steps || span > md->homing.max_travel_steps) {
        motor_fail(md, MOTOR_FAULT_TRAVEL); goto done;
    }
    travel = (uint32_t)span;
    if (!home_center(md, second, travel / 2u, start)) { travel = 0u; goto done; }
    /* No pulses can be pending now. Preserve this center through mode changes. */
    md->step_position = 0;
    md->position = 0.0f;
    md->travel_steps = travel;
    md->homed = 1u;
done:
    motor_stop(md);
    md->pulse_stop_enabled = 0u;
    md->pulse_limit_stop_mask = 3u;
    md->homing_active = 0u;
    return md->fault == MOTOR_FAULT_NONE && md->homed ? travel : 0u;
}
