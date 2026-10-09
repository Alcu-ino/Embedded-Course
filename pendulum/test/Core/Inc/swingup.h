/*
 * swingup.h
 *
 *  Swing-up a iniezione di energia (Astrom-Furuta) per pendolo su carrello.
 *  Usa encoder.c (angolo dal basso: 0 = giu', 180 gradi = su) e motor.c
 *  (comando in accelerazione, step/s^2).
 */
#ifndef SWINGUP_H
#define SWINGUP_H

#include "encoder.h"
#include "motor.h"

/* ------------------------------------------------------------------------ */
/*  Parametri                                                               */
/* ------------------------------------------------------------------------ */

/* Guadagno: moltiplica (E_rif - E_attuale), con l'energia in joule
   (NON normalizzata: la scala 1/(m*g*l) e' gia' dentro K).
   Il risultato va diretto a motor_acc(), quindi K e' in (step/s^2) / J. */
#define SWINGUP_K           800.0f

/* DA INSERIRE: parametri fisici del pendolo (gli stessi usati in MATLAB).
   I valori qui sotto sono SEGNAPOSTO. */
#define SWINGUP_M           0.10f      /* [kg]     massa del pendolo                  */
#define SWINGUP_L           0.7056f      /* [m]      distanza perno - baricentro        */
#define SWINGUP_J           0.0f     /* [kg*m^2] inerzia rispetto al PERNO
                                          (asta uniforme lunga Lasta: m*Lasta^2/3)    */
#define SWINGUP_G           9.81f      /* [m/s^2]                                     */

/* m*g*l [J]: energia potenziale del pendolo orizzontale */
#define SWINGUP_MGL         (SWINGUP_M * SWINGUP_G * SWINGUP_L)

/* Energia di riferimento [J]: pendolo fermo in alto = 2*m*g*l.
   Si puo' alzare di poco (es. 2.05f * SWINGUP_MGL) per compensare l'attrito. */
#define SWINGUP_E_RIF       (2.0f * SWINGUP_MGL)

/* DA TARARE: saturazione dell'accelerazione [step/s^2].
   Sotto il limite a cui lo stepper perde passi. */
#define SWINGUP_ACC_MAX     2000.0f

/* Verso della spinta: +1 oppure -1.
   Dipende da come sono orientati asse del carrello e encoder.
   Se l'ampiezza delle oscillazioni CALA invece di crescere, mettere -1. */
#define SWINGUP_SEGNO       (1.0f)

/* Zona morta su w*cos(theta) [rad/s]: sotto questa soglia il segno
   non viene aggiornato (resta l'ultimo valido), per non commutare sul rumore. */
#define SWINGUP_EPS         0.05f

/* Finestra di cattura attorno alla verticale alta: 180 +/- 15 gradi
   (cioe' 165 - 195 gradi). Dentro la finestra subentra l'LQR. */
#define SWINGUP_CATTURA_DEG 15.0f

/* Richiamo del carrello verso il centro (0 = disattivato).
   KX in 1/s^2 (per step di posizione), KV in 1/s (per step/s di velocita').
   Senza richiamo lo swing-up ignora la posizione: attenzione ai finecorsa. */
#define SWINGUP_KX          0.0f
#define SWINGUP_KV          0.0f

/* ------------------------------------------------------------------------ */
/*  Tipi                                                                    */
/* ------------------------------------------------------------------------ */
typedef enum {
    SWINGUP_IN_CORSO = 0,   /* swing-up attivo: motor_acc() gia' chiamata      */
    SWINGUP_CATTURA  = 1    /* pendolo nella finestra: motor_acc() NON chiamata,
                               tocca all'LQR                                   */
} swingup_stato_t;

typedef struct {
    float segno;    /* ultimo segno valido di w*cos(theta): +1 / -1 */
    float energia;  /* energia attuale [J] (debug)                  */
    float errore;   /* scostamento dalla verticale alta [rad], in (-pi, pi] */
    float acc;      /* ultima accelerazione comandata [step/s^2]    */
} swingup_t;

/* ------------------------------------------------------------------------ */
/*  Funzioni                                                                */
/* ------------------------------------------------------------------------ */
void swingup_init(swingup_t *su);

/* Da chiamare a frequenza fs (ISR di TIM3), DOPO update_Encoder(). */
swingup_stato_t swingup_update(swingup_t *su,
                               const Encoder_HandleTypeDef *encoder,
                               motor_drv_t *motor_drv);

#endif /* SWINGUP_H */