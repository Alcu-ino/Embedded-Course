/*
 * pid.c
 *
 *  PID in cascata (posizione -> angolo -> accelerazione) con stimatore di
 *  [phi, w] sul modello linearizzato. Vedi pid.h per la struttura e per
 *  come i guadagni sono ricavati dall'LQR.
 */
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile float pid_phi_stima = 0.0f;
volatile float pid_w_stima   = 0.0f;
volatile float pid_phi_rif   = 0.0f;
volatile float pid_u         = 0.0f;

/* Stato interno (persiste tra una chiamata e l'altra) */
static float    phi_h   = 0.0f;   /* angolo stimato [rad]                       */
static float    w_h     = 0.0f;   /* velocita' angolare stimata [rad/s]         */
static float    v_prec  = 0.0f;   /* fcurrent alla chiamata precedente [imp/s]  */
static float    int_x   = 0.0f;   /* KI_X * integrale(p)       [rad]            */
static float    int_phi = 0.0f;   /* KI_PHI * integrale(e_phi) [m/s^2]          */
static uint32_t t_prec  = 0u;     /* HAL_GetTick() alla chiamata precedente     */
static uint8_t  avviato = 0u;

static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

static float satura(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

void pid_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc)
{
    const float ts = encoder->ts;                 /* [s] periodo di controllo */

    /* ---------------------------------------------------------------------- */
    /* 1. Angolo misurato rispetto alla verticale alta, in (-pi, pi],          */
    /*    con il verso dell'encoder tarato per l'LQR.                          */
    /* ---------------------------------------------------------------------- */
    const float phi_mis = LQR_VERSO_ANGOLO * wrap_pi(encoder->rad_angle - (float)M_PI);

    /* ---------------------------------------------------------------------- */
    /* 2. (Ri)aggancio: se il PID non e' stato chiamato negli ultimi ms, il    */
    /*    pendolo e' appena entrato nella finestra 165-195 gradi.              */
    /*    Lo stimatore riparte dalla misura e gli INTEGRALI SI AZZERANO:       */
    /*    quanto accumulato in un tentativo precedente non vale piu'.          */
    /* ---------------------------------------------------------------------- */
    const uint32_t t_ora = HAL_GetTick();
    if (!avviato || (t_ora - t_prec) > LQR_RIAGGANCIO_MS) {
        phi_h   = phi_mis;
        w_h     = LQR_VERSO_ANGOLO * encoder->w;
        v_prec  = motor_drv->fcurrent;
        int_x   = 0.0f;
        int_phi = 0.0f;
        avviato = 1u;
    }
    t_prec = t_ora;

    /* ---------------------------------------------------------------------- */
    /* 3. Stimatore (identico a lqr.c): predizione con il modello usando        */
    /*    l'accelerazione davvero applicata, poi correzione con la misura.     */
    /* ---------------------------------------------------------------------- */
    const float u_applicata = (motor_drv->fcurrent - v_prec) / ts * M_PER_STEP;
    v_prec = motor_drv->fcurrent;

    const float phi_p = phi_h + ts * w_h;
    const float w_p   = w_h   + ts * (LQR_A21 * phi_h - LQR_B2 * u_applicata);

    const float L1 = 2.0f * OSS_ZETA * OSS_W0 * ts;
    const float L2 = (OSS_W0 * OSS_W0 + LQR_A21) * ts;
    const float e_oss = wrap_pi(phi_mis - phi_p);

    phi_h = phi_p + L1 * e_oss;
    w_h   = w_p   + L2 * e_oss;

    /* ---------------------------------------------------------------------- */
    /* 4. Posizione e velocita' del carrello in unita' SI                       */
    /* ---------------------------------------------------------------------- */
    const float p = motor_drv->position * M_PER_STEP;   /* [m]   */
    const float v = motor_drv->fcurrent * M_PER_STEP;   /* [m/s] */

    /* ---------------------------------------------------------------------- */
    /* 5. ANELLO ESTERNO - PID sulla posizione (riferimento p = 0, il centro). */
    /*    L'uscita e' l'inclinazione da dare al pendolo: per riportare il      */
    /*    carrello verso il centro il pendolo deve pendere verso il centro.    */
    /*    Il derivativo usa v misurata (non la derivata di p): niente rumore.  */
    /* ---------------------------------------------------------------------- */
    int_x += PID_KI_X * p * ts;
    int_x  = satura(int_x, PID_INT_X_MAX);                    /* anti-windup */

    float phi_rif = -(PID_KP_X * p + int_x + PID_KD_X * v);
    phi_rif = satura(phi_rif, PID_PHI_RIF_MAX);

    /* ---------------------------------------------------------------------- */
    /* 6. ANELLO INTERNO - PID sull'angolo.                                     */
    /*    Proporzionale e integrale sull'errore (phi - phi_rif).               */
    /*    Derivativo sulla SOLA misura w (non su d(errore)/dt): quando         */
    /*    l'anello esterno sposta phi_rif non arriva un colpo al carrello.     */
    /*    Il segno e' "positivo" perche' il pendolo e' instabile: se cade      */
    /*    verso +phi il carrello deve accelerare verso +phi per sottenderlo.   */
    /* ---------------------------------------------------------------------- */
    const float e_phi = phi_h - phi_rif;

    float u = PID_KP_PHI * e_phi + int_phi + PID_KD_PHI * w_h;

    /* ---------------------------------------------------------------------- */
    /* 7. Saturazione dell'accelerazione e anti-windup dell'integrale interno: */
    /*    si integra solo se il comando non e' saturo, oppure se l'errore lo   */
    /*    farebbe uscire dalla saturazione.                                    */
    /* ---------------------------------------------------------------------- */
    const float u_sat = satura(u, PID_ACC_MAX);
    if (u == u_sat || (u > 0.0f) != (e_phi > 0.0f)) {
        int_phi += PID_KI_PHI * e_phi * ts;
        int_phi  = satura(int_phi, PID_INT_PHI_MAX);
    }
    u = u_sat;

    /* ---------------------------------------------------------------------- */
    /* 8. Uscita: m/s^2 -> impulsi/s^2 per motor_acc()                          */
    /* ---------------------------------------------------------------------- */
    *acc = u / M_PER_STEP;

    /* Debug */
    pid_phi_stima = phi_h;
    pid_w_stima   = w_h;
    pid_phi_rif   = phi_rif;
    pid_u         = u;
}
