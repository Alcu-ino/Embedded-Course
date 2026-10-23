/*
 * lqr.c
 *
 *  Created on: 4 oct 2026
 *      Author: vito
 *
 *  LQR con stimatore di [phi, w] sul modello linearizzato.
 *
 *  Perche' lo stimatore: l'encoder ha 2400 impulsi/giro (2.6 mrad). Derivando
 *  su 1 ms, encoder->w vale 0 oppure +/-2.6 rad/s e niente in mezzo: con
 *  K4 ~ 70 ogni impulso darebbe un colpo di ~180 m/s^2 al carrello.
 *  Lo stimatore predice angolo e velocita' con il modello e li corregge con
 *  l'angolo misurato: la velocita' stimata e' continua e quasi senza ritardo.
 */
#include "lqr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile float lqr_phi_stima = 0.0f;
volatile float lqr_w_stima   = 0.0f;
volatile float lqr_u         = 0.0f;

/* Guadagni in un vettore per il prodotto K*x */
static const float K[4] = { LQR_K1, LQR_K2, LQR_K3, LQR_K4 };

/* Stato interno dello stimatore (persiste tra una chiamata e l'altra) */
static float    phi_h  = 0.0f;   /* angolo stimato [rad]                       */
static float    w_h    = 0.0f;   /* velocita' angolare stimata [rad/s]         */
static float    v_prec = 0.0f;   /* fcurrent alla chiamata precedente [imp/s]  */
static uint32_t t_prec = 0u;     /* HAL_GetTick() alla chiamata precedente     */
static uint8_t  avviato = 0u;    /* 0 finche' non c'e' stata una prima chiamata */

/* Riporta un angolo in (-pi, pi] */
static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc)
{
    const float ts = encoder->ts;                 /* [s] periodo di controllo = 1/Fs */

    /* ---------------------------------------------------------------------- */
    /* 1. Angolo misurato rispetto alla verticale alta.                        */
    /*    rad_angle va da 0 (basso) a 2*pi; in alto vale pi.                   */
    /*    wrap_pi lo porta in (-pi, pi], LQR_VERSO_ANGOLO lo allinea al verso  */
    /*    del carrello usato nel modello.                                      */
    /* ---------------------------------------------------------------------- */
    const float phi_mis = LQR_VERSO_ANGOLO * wrap_pi(encoder->rad_angle - (float)M_PI);

    /* ---------------------------------------------------------------------- */
    /* 2. (Ri)aggancio: se l'LQR non e' stato chiamato negli ultimi ms, il      */
    /*    pendolo e' appena entrato nella finestra 165-195 gradi.              */
    /*    Lo stimatore parte dalla misura; per la velocita' si usa la derivata */
    /*    grezza dell'encoder (rumorosa, ma lo stimatore la corregge in ~0.1 s).*/
    /* ---------------------------------------------------------------------- */
    const uint32_t t_ora = HAL_GetTick();
    if (!avviato || (t_ora - t_prec) > LQR_RIAGGANCIO_MS) {
        phi_h   = phi_mis;
        w_h     = LQR_VERSO_ANGOLO * encoder->w;
        v_prec  = motor_drv->fcurrent;
        avviato = 1u;
    }
    t_prec = t_ora;

    /* ---------------------------------------------------------------------- */
    /* 3. Accelerazione REALMENTE applicata al carrello nell'ultimo periodo    */
    /*    [m/s^2]. Si ricava dalla variazione di velocita' prodotta da         */
    /*    motor_acc(): tiene conto della saturazione di velocita' (fmax),      */
    /*    cosi' il modello dello stimatore vede quello che succede davvero.    */
    /* ---------------------------------------------------------------------- */
    const float u_applicata = (motor_drv->fcurrent - v_prec) / ts * M_PER_STEP;
    v_prec = motor_drv->fcurrent;

    /* ---------------------------------------------------------------------- */
    /* 4. Stimatore - PREDIZIONE con il modello linearizzato (Eulero):          */
    /*        phi(k+1) = phi(k) + ts * w(k)                                    */
    /*        w(k+1)   = w(k)   + ts * (A21*phi(k) - B2*u(k))                  */
    /* ---------------------------------------------------------------------- */
    const float phi_p = phi_h + ts * w_h;
    const float w_p   = w_h   + ts * (LQR_A21 * phi_h - LQR_B2 * u_applicata);

    /* ---------------------------------------------------------------------- */
    /* 5. Stimatore - CORREZIONE con la misura.                                 */
    /*    e = errore tra angolo misurato e predetto.                           */
    /*    Guadagni L1, L2 scelti per avere i poli dell'errore di stima in      */
    /*    s^2 + 2*zeta*w0*s + w0^2 (il termine A21 in L2 compensa              */
    /*    l'instabilita' del modello aperto).                                  */
    /* ---------------------------------------------------------------------- */
    const float L1 = 2.0f * OSS_ZETA * OSS_W0 * ts;
    const float L2 = (OSS_W0 * OSS_W0 + LQR_A21) * ts;
    const float e  = wrap_pi(phi_mis - phi_p);

    phi_h = phi_p + L1 * e;
    w_h   = w_p   + L2 * e;

    /* ---------------------------------------------------------------------- */
    /* 6. Stato in unita' SI.                                                  */
    /*    position e fcurrent sono in impulsi (contati da TIM5 / comandati):   */
    /*    si convertono in metri con M_PER_STEP.                               */
    /* ---------------------------------------------------------------------- */
    const float x[4] = {
        motor_drv->position * M_PER_STEP,   /* p   [m]     */
        motor_drv->fcurrent * M_PER_STEP,   /* v   [m/s]   */
        phi_h,                              /* phi [rad]   */
        w_h                                 /* w   [rad/s] */
    };

    /* ---------------------------------------------------------------------- */
    /* 7. Legge di controllo u = -K x  [m/s^2]                                 */
    /* ---------------------------------------------------------------------- */
    float u = 0.0f;
    for (int i = 0; i < 4; i++) {
        u -= K[i] * x[i];
    }

    /* ---------------------------------------------------------------------- */
    /* 8. Saturazione dell'accelerazione: motor_acc limita solo la velocita'.  */
    /* ---------------------------------------------------------------------- */
    if (u >  LQR_ACC_MAX) u =  LQR_ACC_MAX;
    if (u < -LQR_ACC_MAX) u = -LQR_ACC_MAX;

    /* ---------------------------------------------------------------------- */
    /* 9. Uscita: m/s^2 -> impulsi/s^2 (UNA sola divisione, niente *32:         */
    /*    il /32 dei microstep e' gia' dentro M_PER_STEP).                     */
    /* ---------------------------------------------------------------------- */
    *acc = u / M_PER_STEP;

    /* Debug */
    lqr_phi_stima = phi_h;
    lqr_w_stima   = w_h;
    lqr_u         = u;
}
