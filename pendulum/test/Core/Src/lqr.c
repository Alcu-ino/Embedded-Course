/*
 * lqr.c
 *
 *  Created on: 4 oct 2026
 *      Author: vito
 *
 *  LQR con azione integrale e stimatore di [phi, w] sul modello linearizzato.
 *
 *  Perche' lo stimatore: l'encoder ha 2400 impulsi/giro (2.6 mrad). Derivando
 *  su 4 ms, encoder->w vale 0 oppure +/-0.65 rad/s e niente in mezzo: con
 *  K4 ~ 32 ogni impulso darebbe un colpo di ~21 m/s^2 al carrello.
 *  Lo stimatore predice angolo e velocita' con il modello e li corregge con
 *  l'angolo misurato: la velocita' stimata e' continua e quasi senza ritardo.
 *
 *  Perche' l'integratore: z = integrale della posizione del carrello
 *  (riferimento = 0, centro corsa). Un offset fisso dell'encoder farebbe
 *  fermare il carrello fuori centro (K3/K1 = 6.7 m/rad, cioe' ~1.8 cm per
 *  conteggio); z cresce finche' il carrello non torna al centro e a regime
 *  vale K3*offset + K5*z = 0. Quindi z misura anche l'offset dell'encoder:
 *  vedi lqr_offset_conteggi.
 *  I guadagni K1..K5 vengono da lqr() sul sistema aumentato [A 0; C 0],
 *  C = [1 0 0 0], convertiti in accelerazione.
 */
#include "lqr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile float lqr_phi_stima       = 0.0f;
volatile float lqr_w_stima         = 0.0f;
volatile float lqr_u               = 0.0f;
volatile float lqr_z               = 0.0f;
volatile float lqr_offset_conteggi = 0.0f;

/* Guadagni in un vettore per il prodotto K*x (K5 = guadagno integrale) */
static const float K[5] = { LQR_K1, LQR_K2, LQR_K3, LQR_K4, LQR_K5 };

/* Stato interno dello stimatore (persiste tra una chiamata e l'altra) */
static float    phi_h  = 0.0f;   /* angolo stimato [rad]                       */
static float    w_h    = 0.0f;   /* velocita' angolare stimata [rad/s]         */
static float    v_prec = 0.0f;   /* fcurrent alla chiamata precedente [imp/s]  */
static uint8_t  avviato = 0u;    /* 0 = alla prossima chiamata si (ri)aggancia */

/* Stato dell'integratore (persiste tra una chiamata e l'altra) */
static float    z_int  = 0.0f;   /* integrale della posizione carrello [m*s]   */

/* Riporta un angolo in (-pi, pi] */
static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

/* Da chiamare quando l'LQR viene abbandonato (pendolo fuori finestra):
   alla prossima chiamata stimatore e integratore ripartono da capo. */
void lqr_reset(void)
{
    avviato = 0u;
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
    const float phi_mis = (LQR_VERSO_ANGOLO * wrap_pi(encoder->rad_angle - (float)M_PI));

    /* ---------------------------------------------------------------------- */
    /* 2. (Ri)aggancio: prima chiamata dopo lqr_reset(), cioe' il pendolo e'   */
    /*    appena entrato nella finestra 170-190 gradi.                         */
    /*    Lo stimatore parte dalla misura; per la velocita' si usa la derivata */
    /*    grezza dell'encoder (rumorosa, ma lo stimatore la corregge in ~0.1 s).*/
    /*    L'integratore riparte da zero: il valore vecchio non ha piu' senso.  */
    /* ---------------------------------------------------------------------- */
    if (!avviato) {
        phi_h   = phi_mis;
        w_h     = LQR_VERSO_ANGOLO * encoder->w;
        v_prec  = motor_drv->fcurrent;
        z_int   = 0.0f;
        avviato = 1u;
    }

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
    /* quanto angolare dell'encoder [rad] */
    const float q_enc = 2.0f * (float)M_PI / (float)(encoder->cpr * encoder->res);
    const float dz    = LQR_DZ_CONTEGGI * q_enc;

    float e = wrap_pi(phi_mis - phi_p);
    if (fabsf(e) <= dz) e = 0.0f;              /* dentro il quanto: si fida del modello  */
    else                e -= copysignf(dz, e); /* fuori: corregge solo l'eccedenza,      */
                                               /* senza gradino all'uscita dalla zona    */
    phi_h = phi_p + L1 * e;
    w_h   = w_p   + L2 * e;

    /* ---------------------------------------------------------------------- */
    /* 6. Stato aumentato in unita' SI (tutti i riferimenti sono nulli).       */
    /*    position e fcurrent sono in impulsi (contati da TIM5 / comandati):   */
    /*    si convertono in metri con M_PER_STEP.                               */
    /* ---------------------------------------------------------------------- */
    const float x[5] = {
        motor_drv->position * M_PER_STEP,   /* p   [m]     rif. 0 (centro)  */
        motor_drv->fcurrent * M_PER_STEP,   /* v   [m/s]   rif. 0           */
        phi_h,                              /* phi [rad]   rif. 0 (upright) */
        w_h,                                /* w   [rad/s] rif. 0           */
        z_int                               /* z   [m*s]   integrale di p   */
    };

    /* ---------------------------------------------------------------------- */
    /* 7. Legge di controllo u = -K x  [m/s^2]  (K = [Kp Ki], x aumentato)     */
    /* ---------------------------------------------------------------------- */
    float u = 0.0f;
    for (int i = 0; i < 5; i++) {
        u -= K[i] * x[i];
    }

    /* ---------------------------------------------------------------------- */
    /* 8. Saturazione dell'accelerazione: motor_acc limita solo la velocita'.  */
    /* ---------------------------------------------------------------------- */
    uint8_t saturo = 0u;
    if (u >  LQR_ACC_MAX) { u =  LQR_ACC_MAX; saturo = 1u; }
    if (u < -LQR_ACC_MAX) { u = -LQR_ACC_MAX; saturo = 1u; }

    /* ---------------------------------------------------------------------- */
    /* 9. Aggiornamento dell'integratore (Eulero):  z(k+1) = z(k) + ts * p(k)  */
    /*    Anti-windup: non integra mentre il comando e' saturo e |z| e'        */
    /*    comunque limitato a LQR_Z_MAX.                                       */
    /* ---------------------------------------------------------------------- */
    if (!saturo) {
        z_int += ts * x[0];
    }
    if (z_int >  LQR_Z_MAX) z_int =  LQR_Z_MAX;
    if (z_int < -LQR_Z_MAX) z_int = -LQR_Z_MAX;

    /* ---------------------------------------------------------------------- */
    /* 10. Uscita: m/s^2 -> impulsi/s^2 (UNA sola divisione, niente *32:        */
    /*     il /32 dei microstep e' gia' dentro M_PER_STEP).                    */
    /* ---------------------------------------------------------------------- */
    *acc = u / M_PER_STEP;

    /* ---------------------------------------------------------------------- */
    /* Debug.                                                                  */
    /* lqr_offset_conteggi: offset dell'encoder stimato dall'integratore.      */
    /* A regime K3*offset + K5*z = 0  ->  offset = -K5*z/K3. Ha senso solo     */
    /* dopo che il carrello si e' assestato al centro (decine di secondi).     */
    /* ---------------------------------------------------------------------- */
    lqr_phi_stima       = phi_h;
    lqr_w_stima         = w_h;
    lqr_u               = u;
    lqr_z               = z_int;
    lqr_offset_conteggi = (-LQR_K5 * z_int / LQR_K3) / q_enc;
}