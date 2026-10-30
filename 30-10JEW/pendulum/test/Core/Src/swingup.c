/*
 * swingup.c
 *
 *  Swing-up a iniezione di energia in forma "a velocita'":
 *
 *      E     = 0.5 * J * w^2 + m*g*l * (1 - cos(theta))        [J]
 *      v_rif = SEGNO * sat(KE * (E_rif - E) * sin(theta), V_SU) - KX * p
 *      acc   = sat((v_rif - v) / TAU, +/- ACC_MAX)
 *
 *  Perche' in velocita' e non in accelerazione (Astrom-Furuta classica):
 *  lo stepper ha una velocita' massima (fmax). Con la legge classica la spinta
 *  satura in un verso, il carrello arriva a fmax in poche decine di ms e poi
 *  viaggia a velocita' costante, che non cede energia al pendolo.
 *  Qui la velocita' segue sin(theta): l'accelerazione e' proporzionale a
 *  w*cos(theta), quindi la potenza ceduta al pendolo e' sempre positiva
 *  finche' E < E_rif, e il carrello resta entro +/- V_SU.
 *  L'energia ceduta per mezza oscillazione e' al massimo 2*m*l*V_SU*w_max:
 *  piu' alta e' fmax, piu' rapido e' lo swing-up.
 */
#include "swingup.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SWINGUP_CATTURA_RAD  (SWINGUP_CATTURA_DEG * (float)M_PI / 180.0f)

void swingup_init(swingup_t *su)
{
    su->w_f     = 0.0f;
    su->energia = 0.0f;
    su->errore  = (float)M_PI;
    su->v_rif   = 0.0f;
    su->acc_ms2 = 0.0f;
    su->acc     = 0.0f;
    su->kick    = SWINGUP_KICK_S;
}

static float sat(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

swingup_stato_t swingup_update(swingup_t *su,
                               const Encoder_HandleTypeDef *encoder,
                               motor_drv_t *motor_drv)
{
    const float ts = encoder->ts;                      /* [s] periodo di controllo */

    /* ---------------------------------------------------------------------- */
    /* 1. Misure. w filtrata: la derivata dell'encoder a 1 kHz ha quanti di     */
    /*    2.6 rad/s, troppo grossi per l'energia cinetica.                     */
    /* ---------------------------------------------------------------------- */
    const float th = encoder->rad_angle;               /* [rad] 0 = basso, pi = alto */
    su->w_f += SWINGUP_ALFA * (encoder->w - su->w_f);
    const float w = su->w_f;

    /* ---------------------------------------------------------------------- */
    /* 2. Energia del pendolo [J]: cinetica + potenziale (zero in basso).       */
    /* ---------------------------------------------------------------------- */
    const float E = 0.5f * SWINGUP_J * w * w + SWINGUP_MGL * (1.0f - cosf(th));
    su->energia = E;

    /* ---------------------------------------------------------------------- */
    /* 3. Scostamento dalla verticale alta in (-pi, pi].                        */
    /*    Nella finestra di cattura lo swing-up si ritira: comanda l'LQR.      */
    /* ---------------------------------------------------------------------- */
    float e = th - (float)M_PI;
    e = atan2f(sinf(e), cosf(e));
    su->errore = e;

    if (fabsf(e) < SWINGUP_CATTURA_RAD) {
        su->v_rif   = 0.0f;
        su->acc_ms2 = 0.0f;
        su->acc     = 0.0f;
        return SWINGUP_CATTURA;                        /* motor_acc() la chiama l'LQR */
    }

    /* ---------------------------------------------------------------------- */
    /* 4. Velocita' massima dello swing-up [m/s], dalla fmax del motore.        */
    /* ---------------------------------------------------------------------- */
    const float v_su = SWINGUP_V_FRAZ * motor_drv->motor->fmax * M_PER_STEP;

    /* ---------------------------------------------------------------------- */
    /* 5. Velocita' di riferimento [m/s].                                       */
    /*    Il segno di sin(theta) mette la velocita' in fase con l'angolo: il    */
    /*    carrello inverte quando il pendolo passa dal basso, dove w e' massima,*/
    /*    e li' l'accelerazione cede il massimo di energia.                    */
    /*    Lontano dal bersaglio KE*(E_rif-E) e' grande e la velocita' satura a */
    /*    +/- V_SU (onda quadra); vicino a E_rif si riduce con continuita'.    */
    /* ---------------------------------------------------------------------- */
    float v_rif = SWINGUP_SEGNO * sat(SWINGUP_KE * (SWINGUP_E_RIF - E) * sinf(th), v_su);

    /* Spinta iniziale: da fermo sin(theta) = 0 */
    if (su->kick > 0.0f) {
        v_rif   += v_su;
        su->kick -= ts;
    }

    /* Richiamo verso il centro (position in impulsi -> m) */
    v_rif -= SWINGUP_KX * motor_drv->position * M_PER_STEP;
    su->v_rif = v_rif;

    /* ---------------------------------------------------------------------- */
    /* 6. Inseguimento della velocita' [m/s^2], saturato.                       */
    /* ---------------------------------------------------------------------- */
    const float v = motor_drv->fcurrent * M_PER_STEP;  /* [m/s] */
    const float acc = sat((v_rif - v) / SWINGUP_TAU, SWINGUP_ACC_MAX);
    su->acc_ms2 = acc;

    /* ---------------------------------------------------------------------- */
    /* 7. Comando al motore: m/s^2 -> impulsi/s^2                               */
    /* ---------------------------------------------------------------------- */
    su->acc = acc / M_PER_STEP;
    motor_acc(su->acc, motor_drv);

    return SWINGUP_IN_CORSO;
}
