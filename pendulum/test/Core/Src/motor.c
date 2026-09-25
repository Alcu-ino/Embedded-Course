/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "motor.h"

/* Ritardo per t_SETUP di DIR (DRV8825 >= 650 ns).
   A 168 MHz ogni iterazione costa ~5-8 cicli: 60 iterazioni = ~2-3 us. */
#define DIR_SETUP_DELAY() do { for (volatile int _i = 0; _i < 60; _i++) { __NOP(); } } while (0)

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
/*  Motore                                                                    */
/* ========================================================================== */
void motor_init(motor_t *motor, uint16_t steps_per_rev, TIM_HandleTypeDef *htim,
                uint32_t tim_channel, float fmax, float fclk, float fs)
{
    TIM_TypeDef *tim = htim->Instance;

    motor->steps_per_rev = steps_per_rev;
    motor->htim_PWM      = htim;
    motor->tim_channel   = tim_channel;
    motor->fclk          = fclk;
    motor->fmax          = fmax;
    motor->fs            = fs;

    /* Preload di ARR e CCR, una volta sola: i nuovi valori diventano attivi
       solo all'update event, quindi CNT non puo' mai trovarsi oltre ARR
       (senza: conteggio fino a 2^32 = ~51 s senza passi). */
    tim->CR1 |= TIM_CR1_ARPE;
    __HAL_TIM_ENABLE_OCxPRELOAD(htim, tim_channel);

    /* Uscita ferma e registri in uno stato noto */
    HAL_TIM_PWM_Stop(htim, tim_channel);
    __HAL_TIM_SET_COMPARE(htim, tim_channel, 0u);
    tim->EGR = TIM_EGR_UG;
}

void motor_drv_init(motor_drv_t *motor_drv, drv_t *drv, motor_t *motor)
{
    motor_drv->drv      = drv;
    motor_drv->motor    = motor;
    motor_drv->state    = MOTOR_DRV_STOPPED;
    motor_drv->fcurrent = 0.0f;
    motor_drv->pwm_on   = 0u;
}

/* Da chiamare a frequenza fs (ISR di TIM3). acc in step/s^2. */
void motor_acc(float acc, motor_drv_t *motor_drv)
{
    TIM_HandleTypeDef *htim = motor_drv->motor->htim_PWM;
    TIM_TypeDef       *tim  = htim->Instance;
    uint32_t psc = tim->PSC;
    uint32_t ch  = motor_drv->motor->tim_channel;

    float fmax = motor_drv->motor->fmax;
    float fclk = motor_drv->motor->fclk;
    float fs   = motor_drv->motor->fs;

    /* 1. Delta time */
    float dt = (fs > 0.0f) ? (1.0f / fs) : 0.005f;

    /* 2. Integrazione velocita' */
    float v = motor_drv->fcurrent + (acc * dt);
    if (!isfinite(v)) v = 0.0f;

    /* 3. Saturazione */
    if (v >  fmax) v =  fmax;
    if (v < -fmax) v = -fmax;

    /* 4. Direzione: scritta solo al cambio */
    uint8_t dir_changed = 0u;
    if (v > 0.0f && motor_drv->drv->dir != DIRECTION_CW) {
        drv_set_direction(motor_drv->drv, DIRECTION_CW);
        dir_changed = 1u;
    } else if (v < 0.0f && motor_drv->drv->dir != DIRECTION_CCW) {
        drv_set_direction(motor_drv->drv, DIRECTION_CCW);
        dir_changed = 1u;
    }

    motor_drv->fcurrent = v;
    float F = fabsf(v);

    /* 5. Frequenza minima: limite del timer o soglia pratica (periodi <= 200 ms).
          Con ARRMAX a 32 bit prevale sempre la soglia pratica, quindi
          ticks resta molto sotto max_ticks e il cast a uint32_t e' sicuro. */
    const float tick_hz   = fclk / ((float)psc + 1.0f);
    const float max_ticks = (float)ARRMAX + 1.0f;
    float fmin = tick_hz / max_ticks;
    if (fmin < MOTOR_FMIN_PRATICA) fmin = MOTOR_FMIN_PRATICA;

    if (F < fmin) {
        motor_drv->state = MOTOR_DRV_STOPPED;
        if (motor_drv->pwm_on) {
            HAL_TIM_PWM_Stop(htim, ch);
            motor_drv->pwm_on = 0u;
        }
        return;
    }

    if (v == fmax || v == -fmax) {
        motor_drv->state = MOTOR_DRV_RUN;                 /* in saturazione */
    } else {
        motor_drv->state = (v * acc >= 0.0f) ? MOTOR_DRV_ACCEL : MOTOR_DRV_DECEL;
    }

    /* 6. Periodo e impulso.
          Impulso alto a durata fissa; il periodo contiene sempre almeno
          un alto e un basso di durata 'pulse', quindi CCR vale sempre 'pulse'.
          Frequenza massima effettiva: tick_hz / (2*pulse) (~199 kHz a 84 MHz). */
    uint32_t pulse     = (uint32_t)(STEP_PULSE_S * tick_hz) + 1u;
    uint32_t min_ticks = 2u * pulse;

    float ticks = tick_hz / F;
    if (ticks < (float)min_ticks) ticks = (float)min_ticks;

    uint32_t arr = (uint32_t)(ticks - 1.0f);
    uint32_t ccr = pulse;

    /* 7. Registri in preload: attivi al prossimo update event */
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, ch, ccr);

    /* 8. Avvio / anticipo del passo */
    if (dir_changed) {
        /* Se la direzione è cambiata, arresta temporaneamente il PWM per garantire
           livello BASSO su STEP e rispettare t_HOLD e t_SETUP del DRV8825 */
        if (motor_drv->pwm_on) {
            HAL_TIM_PWM_Stop(htim, ch);
            motor_drv->pwm_on = 0u;
        }
        DIR_SETUP_DELAY();              /* Garantisce t_SETUP >= 650 ns */
        tim->EGR = TIM_EGR_UG;          /* Carica subito i nuovi ARR/CCR e azzera CNT */
        HAL_TIM_PWM_Start(htim, ch);    /* Avvia il PWM in modo pulito */
        motor_drv->pwm_on = 1u;
    } else if (motor_drv->pwm_on == 0u) {
        /* Avvio da fermo senza cambio direzione */
        tim->EGR = TIM_EGR_UG;
        HAL_TIM_PWM_Start(htim, ch);
        motor_drv->pwm_on = 1u;
    } else {
        /* Motore già in moto nella stessa direzione: anticipo sicuro se accelera */
        uint32_t cnt = tim->CNT;
        if (cnt >= arr && cnt >= 2u * pulse) {
            tim->EGR = TIM_EGR_UG;
        }
    }
}