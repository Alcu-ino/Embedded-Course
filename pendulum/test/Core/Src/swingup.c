/*
 * swingup.c
 *
 *  Swing-up a iniezione di energia (Astrom-Furuta):
 *
 *      acc = sat( -K * (E_rif - E) * sign(w * cos(theta)), +/- acc_max )
 *
 *  Energia in joule (non normalizzata), angolo misurato dal basso:
 *
 *      E = 0.5 * J * w^2 + m*g*l * (1 - cos(theta))
 *
 *      E = 0        pendolo fermo in basso
 *      E = 2*m*g*l  pendolo fermo in alto  -> E_rif
 *
 *  K moltiplica direttamente (E_rif - E) in joule: la scala 1/(m*g*l)
 *  e' inclusa in K.
 *
 *  Uso (ISR di TIM3, a frequenza fs):
 *
 *      update_Encoder(&encoder);
 *      if (swingup_update(&su, &encoder, &motor_drv) == SWINGUP_CATTURA) {
 *          float acc_lqr = ...;              // LQR sull'errore su.errore
 *          motor_acc(acc_lqr, &motor_drv);
 *      }
 */
#include "swingup.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SWINGUP_CATTURA_RAD  (SWINGUP_CATTURA_DEG * (float)M_PI / 180.0f)

void swingup_init(swingup_t *su)
{
    su->segno   = 1.0f;   /* segno iniziale: da' la prima spinta partendo da fermo */
    su->energia = 0.0f;
    su->errore  = (float)M_PI;
    su->acc     = 0.0f;
}

swingup_stato_t swingup_update(swingup_t *su,
                               const Encoder_HandleTypeDef *encoder,
                               motor_drv_t *motor_drv)
{
    float th = encoder->rad_angle;   /* [rad], 0 = basso, pi = alto */
    float w  = encoder->w;           /* [rad/s]                     */
    float c  = cosf(th);

    /* 1. Energia del pendolo [J] */
    float E = 0.5f * SWINGUP_J * w * w + SWINGUP_MGL * (1.0f - c);
    su->energia = E;

    /* 2. Scostamento dalla verticale alta, riportato in (-pi, pi]:
          vale da entrambi i lati (165-195 gradi) e dopo piu' giri */
    float e = th - (float)M_PI;
    e = atan2f(sinf(e), cosf(e));
    su->errore = e;

    if (fabsf(e) < SWINGUP_CATTURA_RAD) {
        su->acc = 0.0f;
        return SWINGUP_CATTURA;      /* motor_acc() la chiama l'LQR */
    }

    /* 3. Segno di w*cos(theta) con zona morta: dentro la zona morta resta
          l'ultimo segno valido. Evita le commutazioni sul rumore e fa
          partire il pendolo da fermo (w = 0) senza un impulso dedicato. */
    float s = w * c;
    if      (s >  SWINGUP_EPS) su->segno =  1.0f;
    else if (s < -SWINGUP_EPS) su->segno = -1.0f;

    /* 4. Legge di Astrom-Furuta */
    float acc = -SWINGUP_SEGNO * SWINGUP_K * (SWINGUP_E_RIF - E) * su->segno;

    /* 5. Richiamo del carrello verso il centro (spento se KX = KV = 0).
          position in step (0 = centro dopo l'homing), fcurrent in step/s. */
    acc -= SWINGUP_KX * motor_drv->position + SWINGUP_KV * motor_drv->fcurrent;

    /* 6. Saturazione */
    if (acc >  SWINGUP_ACC_MAX) acc =  SWINGUP_ACC_MAX;
    if (acc < -SWINGUP_ACC_MAX) acc = -SWINGUP_ACC_MAX;
    su->acc = acc;

    /* 7. Comando al motore [step/s^2] */
    motor_acc(acc, motor_drv);

    return SWINGUP_IN_CORSO;
}