/*
 * swingup.h
 *
 *  Swing-up a iniezione di energia (Astrom-Furuta) per pendolo su carrello.
 *  Angolo dal basso (encoder.c): 0 = giu', pi = su.
 *  I calcoli sono in unita' SI (m/s^2); la conversione in impulsi/s^2
 *  avviene solo alla chiamata di motor_acc().
 */
#ifndef SWINGUP_H
#define SWINGUP_H

#include "encoder.h"
#include "motor.h"
#include "lqr.h"          /* M_PER_STEP: un'unica definizione per tutto il progetto */

/* ------------------------------------------------------------------------ */
/*  Parametri fisici (gli STESSI del modello MATLAB e di lqr.h)              */
/* ------------------------------------------------------------------------ */
#define SWINGUP_M           0.10f                 /* [kg] massa del pendolo          */
#define SWINGUP_L           0.7056f               /* [m]  perno - baricentro         */
#define SWINGUP_G           9.81f                 /* [m/s^2]                         */
/* [kg*m^2] inerzia rispetto al PERNO. Massa concentrata: m*L^2.
   Asta uniforme lunga Lasta: m*Lasta^2/3 (e SWINGUP_L = Lasta/2). */
#define SWINGUP_J           (SWINGUP_M * SWINGUP_L * SWINGUP_L)

/* m*g*l [J]: energia potenziale con il pendolo orizzontale */
#define SWINGUP_MGL         (SWINGUP_M * SWINGUP_G * SWINGUP_L)

/* Energia di riferimento [J]. Pendolo fermo in alto = 2*m*g*l, ma la spinta
   K*(E_rif - E) tende a zero avvicinandosi al bersaglio e, con l'attrito,
   l'energia si ferma all'80-90%: si mira il 10% piu' in alto (2.2 = 2 * 1.1).
   L'eccesso non e' un problema: nella finestra di cattura subentra l'LQR. */
#define SWINGUP_E_RIF       (10.0f * SWINGUP_MGL)

/* ------------------------------------------------------------------------ */
/*  Taratura                                                                */
/* ------------------------------------------------------------------------ */
/* Guadagno [(m/s^2)/J]: moltiplica (E_rif - E) in joule.
   Da fermo E = 0: acc = K * E_rif = 4 * 1.52 J ~ 6 m/s^2 -> saturata a ACC_MAX.
   Valori piu' bassi (1.5) non bastano a vincere l'attrito vicino all'alto. */
#define SWINGUP_K           100.0f

/* Accelerazione massima [m/s^2] (sotto il limite a cui lo stepper perde passi) */
#define SWINGUP_ACC_MAX     3.0f

/* Verso della spinta. NON e' un parametro libero: dipende dal verso
   dell'encoder gia' fissato per l'LQR.
   Bilancio di energia: dE/dt = m*l*acc*w*cos(theta) * LQR_VERSO_ANGOLO
   (con acc = accelerazione del carrello verso +position). Perche' l'energia
   cresca, la spinta deve avere segno -LQR_VERSO_ANGOLO * sign(w*cos(theta)):
   la formula al punto 5 di swingup.c ha gia' il '-', quindi SEGNO = -VERSO.
   Basta tarare LQR_VERSO_ANGOLO con la prova a mano e questo segue da solo. */
#define SWINGUP_SEGNO       (-LQR_VERSO_ANGOLO)

/* Filtro passa-basso su w: w_f += ALFA * (w - w_f).
   0.1 a 1 kHz = costante di tempo ~10 ms. Serve perche' encoder->w
   salta tra 0 e +/-2.6 rad/s e il segno di w*cos(theta) commuterebbe sul rumore. */
#define SWINGUP_ALFA        0.1f

/* Zona morta su w*cos(theta) [rad/s] (applicata alla w filtrata) */
#define SWINGUP_EPS         0.4f

/* Finestra di cattura attorno alla verticale alta: 180 +/- 5 gradi
   (deve coincidere con ANGOLO_MIN / ANGOLO_MAX del main). */
#define SWINGUP_CATTURA_DEG 20.0f

/* Richiamo del carrello verso il centro (0 = centro dopo l'homing).
   KX [1/s^2] su posizione in m, KV [1/s] su velocita' in m/s.
   Senza richiamo in simulazione il carrello deriva di metri.
   KV frena la deriva di velocita' (il carrello resta "centrato" in velocita'),
   KX lo riporta verso il centro. Con 2 e 2: escursione circa +/-35 cm. */
#define SWINGUP_KX          2.0f
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
    float segno;    /* ultimo segno valido di w*cos(theta): +1 / -1   */
    float w_f;      /* velocita' angolare filtrata [rad/s]            */
    float energia;  /* energia attuale [J] (debug)                    */
    float errore;   /* scostamento dalla verticale alta [rad]         */
    float acc_ms2;  /* ultima accelerazione comandata [m/s^2] (debug) */
    float acc;      /* ultima accelerazione comandata [impulsi/s^2]   */
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
