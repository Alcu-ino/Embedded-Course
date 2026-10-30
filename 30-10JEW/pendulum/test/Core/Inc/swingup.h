/*
 * swingup.h
 *
 *  Swing-up a iniezione di energia per pendolo su carrello, in forma
 *  "a velocita'": adatta a un carrello con velocita' massima limitata
 *  (stepper, fmax).
 *  Angolo dal basso (encoder.c): 0 = giu', pi = su.
 *  I calcoli sono in unita' SI; la conversione in impulsi/s^2 avviene solo
 *  alla chiamata di motor_acc().
 */
#ifndef SWINGUP_H
#define SWINGUP_H

#include "encoder.h"
#include "motor.h"
#include "lqr.h"          /* M_PER_STEP, LQR_VERSO_ANGOLO */

/* ------------------------------------------------------------------------ */
/*  Parametri fisici (gli STESSI del modello MATLAB e di lqr.h)              */
/* ------------------------------------------------------------------------ */
/* Pendolo equivalente: asta uniforme 0.65 m sopra il perno + 0.35 m sotto
   -> L = (2/3)(0.65^3 + 0.35^3)/(0.65^2 - 0.35^2) = 0.7056 m.
   Si usa il pendolo a massa concentrata m, L con lo stesso rapporto J/(m*g*l):
   l'energia NORMALIZZATA E/(m*g*l) e' identica a quella del pendolo vero,
   quindi il valore di m cambia solo la scala in joule (assorbita da KE). */
#define SWINGUP_M           0.10f                 /* [kg]                            */
#define SWINGUP_L           0.5778f               /* [m] lunghezza equivalente       */
#define SWINGUP_G           9.81f                 /* [m/s^2]                         */
#define SWINGUP_J           (SWINGUP_M * SWINGUP_L * SWINGUP_L)
#define SWINGUP_MGL         (SWINGUP_M * SWINGUP_G * SWINGUP_L)

/* Energia di riferimento [J]. Fermo in alto = 2*m*g*l. Si mira un po' sopra
   per vincere l'attrito: il pendolo arriva in finestra con ~1-2 rad/s.
   NON alzarla di molto: con 10*MGL (versione precedente) la spinta era sempre
   satura e il pendolo arrivava in alto troppo veloce per l'LQR. */
#define SWINGUP_E_RIF       (2.2f * SWINGUP_MGL)

/* ------------------------------------------------------------------------ */
/*  Taratura                                                                */
/* ------------------------------------------------------------------------ */
/* Velocita' di riferimento del carrello:
       v_rif = SEGNO * sat(KE * (E_rif - E) * sin(theta), V_SU) - KX * p
   Con v proporzionale a sin(theta) l'accelerazione e' proporzionale a
   w*cos(theta): la potenza ceduta al pendolo e' sempre >= 0 e la velocita'
   resta limitata, invece di saturare in un verso come con la legge in
   accelerazione. */
#define SWINGUP_KE          20.0f    /* [(m/s)/J]                                */

/* Frazione di fmax usata dallo swing-up (margine per il richiamo KX).
   V_SU = SWINGUP_V_FRAZ * fmax * M_PER_STEP, letta da motor_drv a runtime. */
#define SWINGUP_V_FRAZ      0.9f

/* Richiamo del carrello verso il centro [1/s]: velocita' in m/s per metro
   di scostamento (0 = centro dopo l'homing). */
#define SWINGUP_KX          1.0f

/* Inseguimento della velocita' di riferimento: acc = (v_rif - v) / TAU [s] */
#define SWINGUP_TAU         0.02f

/* Accelerazione massima [m/s^2] (sotto il limite a cui lo stepper perde passi) */
#define SWINGUP_ACC_MAX     3.0f

/* Verso della spinta: segue il verso dell'encoder fissato per l'LQR.
   Verificato in simulazione MuJoCo con LQR_VERSO_ANGOLO = +1.
   Se in prova l'ampiezza CALA invece di crescere, il verso dell'encoder
   o del carrello e' invertito: sistemare LQR_VERSO_ANGOLO, non questo. */
#define SWINGUP_SEGNO       (LQR_VERSO_ANGOLO)

/* Spinta iniziale: da fermo sin(theta) = 0 e la legge non farebbe nulla.
   Per KICK_S secondi dopo swingup_init() si aggiunge +V_SU. */
#define SWINGUP_KICK_S      0.15f

/* Filtro passa-basso su w (solo per il calcolo dell'energia):
   w_f += ALFA * (w - w_f). 0.1 a 1 kHz = costante di tempo ~10 ms. */
#define SWINGUP_ALFA        0.1f

/* Finestra di cattura attorno alla verticale alta: 180 +/- 15 gradi.
   DEVE coincidere con la finestra in cui il main chiama l'LQR (165-195):
   se e' piu' larga, tra le due finestre nessuno chiama motor_acc() e il
   carrello prosegue alla velocita' dell'ultimo comando. */
#define SWINGUP_CATTURA_DEG 15.0f

/* ------------------------------------------------------------------------ */
/*  Tipi                                                                    */
/* ------------------------------------------------------------------------ */
typedef enum {
    SWINGUP_IN_CORSO = 0,   /* swing-up attivo: motor_acc() gia' chiamata      */
    SWINGUP_CATTURA  = 1    /* pendolo nella finestra: motor_acc() NON chiamata,
                               tocca all'LQR                                   */
} swingup_stato_t;

typedef struct {
    float    w_f;      /* velocita' angolare filtrata [rad/s]            */
    float    energia;  /* energia attuale [J] (debug)                    */
    float    errore;   /* scostamento dalla verticale alta [rad]         */
    float    v_rif;    /* velocita' di riferimento [m/s] (debug)         */
    float    acc_ms2;  /* ultima accelerazione comandata [m/s^2] (debug) */
    float    acc;      /* ultima accelerazione comandata [impulsi/s^2]   */
    float    kick;     /* tempo rimanente di spinta iniziale [s]        */
} swingup_t;

/* ------------------------------------------------------------------------ */
/*  Funzioni                                                                */
/* ------------------------------------------------------------------------ */
void swingup_init(swingup_t *su);

/* A frequenza Fs (ISR di TIM1), DOPO update_Encoder() e motor_aggiorna_posizione(). */
swingup_stato_t swingup_update(swingup_t *su,
                               const Encoder_HandleTypeDef *encoder,
                               motor_drv_t *motor_drv);

#endif /* SWINGUP_H */
