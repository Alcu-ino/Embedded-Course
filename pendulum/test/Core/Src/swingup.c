/*
 * swingup.c
 *
 *  Swing-up a iniezione di energia (Astrom-Furuta):
 *
 *      acc = sat( -K * (E_rif - E) * sign(w * cos(theta)) - KX*p - KV*v , +/- acc_max )
 *
 *  Energia in joule (non normalizzata), angolo misurato dal basso:
 *
 *      E = 0.5 * J * w^2 + m*g*l * (1 - cos(theta))
 *
 *      E = 0        pendolo fermo in basso
 *      E = 2*m*g*l  pendolo fermo in alto  -> E_rif
 *
 *  Perche' funziona: la potenza che il carrello trasferisce al pendolo e'
 *  proporzionale a -acc * w * cos(theta). Scegliendo il segno di acc opposto
 *  a quello di w*cos(theta), l'energia cresce finche' E < E_rif e cala se E > E_rif.
 */
#include "swingup.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SWINGUP_CATTURA_RAD  (SWINGUP_CATTURA_DEG * (float)M_PI / 180.0f)

void swingup_init(swingup_t *su)
{
    su->segno   = 1.0f;          /* segno iniziale: da' la prima spinta partendo da fermo */
    su->w_f     = 0.0f;
    su->energia = 0.0f;
    su->errore  = (float)M_PI;
    su->acc_ms2 = 0.0f;
    su->acc     = 0.0f;
}

swingup_stato_t swingup_update(swingup_t *su,
                               const Encoder_HandleTypeDef *encoder,
                               motor_drv_t *motor_drv)
{
    /* ---------------------------------------------------------------------- */
    /* 1. Misure. w viene filtrata: la derivata dell'encoder a 1 kHz ha quanti */
    /*    di 2.6 rad/s, troppo grossi per decidere il segno della spinta.      */
    /* ---------------------------------------------------------------------- */
    const float th = encoder->rad_angle;               /* [rad] 0 = basso, pi = alto */
    su->w_f += SWINGUP_ALFA * (encoder->w - su->w_f);
    const float w = su->w_f;                           /* [rad/s] filtrata */
    const float c = cosf(th);

    /* ---------------------------------------------------------------------- */
    /* 2. Energia del pendolo [J]: cinetica + potenziale (zero in basso).       */
    /* ---------------------------------------------------------------------- */
    const float E = 0.5f * SWINGUP_J * w * w + SWINGUP_MGL * (1.0f - c);
    su->energia = E;

    /* ---------------------------------------------------------------------- */
    /* 3. Scostamento dalla verticale alta in (-pi, pi].                        */
    /*    Nella finestra di cattura lo swing-up si ritira: comanda l'LQR.      */
    /* ---------------------------------------------------------------------- */
    float e = th - (float)M_PI;
    e = atan2f(sinf(e), cosf(e));
    su->errore = e;

    if (fabsf(e) < SWINGUP_CATTURA_RAD) {
        su->acc_ms2 = 0.0f;
        su->acc     = 0.0f;
        return SWINGUP_CATTURA;                        /* motor_acc() la chiama l'LQR */
    }

    /* ---------------------------------------------------------------------- */
    /* 4. Segno di w*cos(theta) con zona morta: dentro la zona morta resta      */
    /*    l'ultimo segno valido. Da fermo (w = 0) vale il segno iniziale +1,   */
    /*    quindi il pendolo riceve la prima spinta senza codice dedicato.      */
    /* ---------------------------------------------------------------------- */
    const float s = w * c;
    if      (s >  SWINGUP_EPS) su->segno =  1.0f;
    else if (s < -SWINGUP_EPS) su->segno = -1.0f;

    /* ---------------------------------------------------------------------- */
    /* 5. Legge di Astrom-Furuta [m/s^2]                                        */
    /* ---------------------------------------------------------------------- */
    float acc = -SWINGUP_SEGNO * SWINGUP_K * (SWINGUP_E_RIF - E) * su->segno;

    /* ---------------------------------------------------------------------- */
    /* 6. Richiamo del carrello verso il centro.                                */
    /*    position (passi contati da TIM5) e fcurrent sono in impulsi:         */
    /*    convertiti in m e m/s.                                               */
    /* ---------------------------------------------------------------------- */
    acc -= SWINGUP_KX * motor_drv->position * M_PER_STEP
         + SWINGUP_KV * motor_drv->fcurrent * M_PER_STEP;

    /* ---------------------------------------------------------------------- */
    /* 7. Saturazione dell'accelerazione [m/s^2]                                */
    /* ---------------------------------------------------------------------- */
    if (acc >  SWINGUP_ACC_MAX) acc =  SWINGUP_ACC_MAX;
    if (acc < -SWINGUP_ACC_MAX) acc = -SWINGUP_ACC_MAX;
    su->acc_ms2 = acc;

    /* ---------------------------------------------------------------------- */
    /* 8. Comando al motore: m/s^2 -> impulsi/s^2                               */
    /* ---------------------------------------------------------------------- */
    su->acc = acc / (M_PER_STEP);
    motor_acc(su->acc, motor_drv);

    return SWINGUP_IN_CORSO;
}
