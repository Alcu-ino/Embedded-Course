/*
 * pid.h
 *
 *  PID in cascata per pendolo inverso su carrello, comando in accelerazione.
 *  Sostituisce lqr_controller() con la STESSA firma: nel main cambia solo
 *  il nome della funzione chiamata.
 *
 *  Struttura (due anelli):
 *
 *    ANELLO ESTERNO (carrello, PID)  -> decide di quanto inclinare il pendolo
 *        phi_rif = -( KP_X*p + KI_X*integrale(p) + KD_X*v )      [rad]
 *
 *    ANELLO INTERNO (angolo, PID)    -> decide l'accelerazione del carrello
 *        u = KP_PHI*(phi - phi_rif) + KI_PHI*integrale(phi - phi_rif) + KD_PHI*w
 *                                                                  [m/s^2]
 *
 *  Perche' due anelli: un PID sul solo angolo tiene su il pendolo ma non vede
 *  il carrello, che deriva a velocita' costante fino al finecorsa.
 *  Per riportare il carrello al centro bisogna prima inclinare il pendolo
 *  verso il centro: e' questo che fa l'anello esterno.
 *
 *  Guadagni ricavati dall'LQR: con KI_X = KI_PHI = 0 il PID da' ESATTAMENTE
 *  lo stesso comando dell'LQR, perche'
 *      u_LQR = 10*p + 38.216*v + 235.67*phi + 69.65*w
 *            = 235.67*( phi + (10/235.67)*p + (38.216/235.67)*v ) + 69.65*w
 *  cioe' KP_PHI = -K3, KD_PHI = -K4, KP_X = K1/K3, KD_X = K2/K3.
 *  In piu' rispetto all'LQR c'e' l'integrale sulla posizione (KI_X), che
 *  annulla la deriva del carrello dovuta a uno zero dell'encoder non
 *  perfettamente verticale (con l'LQR 1 grado di errore = 41 cm di scarto).
 *
 *  Stato in unita' SI (come lqr.c):
 *      p [m], v [m/s], phi [rad] dalla verticale alta, w [rad/s]
 *  phi e w arrivano dallo stesso stimatore di lqr.c (encoder->w da solo e'
 *  troppo quantizzato per il termine derivativo).
 */
#ifndef PID_H
#define PID_H

#include "encoder.h"
#include "motor.h"
#include "lqr.h"   /* M_PER_STEP, LQR_VERSO_ANGOLO, modello e poli dello stimatore:
                      un'unica definizione per LQR, PID e swing-up */

/* ------------------------------------------------------------------------ */
/*  Anello INTERNO: angolo                                                  */
/* ------------------------------------------------------------------------ */
#define PID_KP_PHI     ( -LQR_K3 )          /* [(m/s^2)/rad]      = 235.67 */
#define PID_KD_PHI     ( -LQR_K4 )          /* [(m/s^2)/(rad/s)]  =  69.65 */
#ifndef PID_KI_PHI
#define PID_KI_PHI     0.0f                 /* [(m/s^2)/(rad*s)]           */
#endif
/* KI_PHI resta a 0: l'errore costante dell'encoder NON si toglie integrando
   l'angolo (a regime il pendolo vero e' verticale e la misura vale proprio
   l'errore). Lo toglie l'integrale sulla posizione, qui sotto. */

/* ------------------------------------------------------------------------ */
/*  Anello ESTERNO: posizione del carrello                                  */
/* ------------------------------------------------------------------------ */
#define PID_KP_X       ( LQR_K1 / LQR_K3 )  /* [rad/m]       = 0.0424 */
#define PID_KD_X       ( LQR_K2 / LQR_K3 )  /* [rad/(m/s)]   = 0.1622 */
#ifndef PID_KI_X
#define PID_KI_X       0.03f                /* [rad/(m*s)]  (tarato in simulazione) */
#endif

/* ------------------------------------------------------------------------ */
/*  Limiti                                                                  */
/* ------------------------------------------------------------------------ */
/* Inclinazione massima chiesta dall'anello esterno [rad] (5 gradi).
   Oltre, il carrello dovrebbe accelerare troppo per riprendere il pendolo. */
#ifndef PID_PHI_RIF_MAX
#define PID_PHI_RIF_MAX     (5.0f * 0.0174533f)
#endif

/* Accelerazione massima comandata [m/s^2] (come l'LQR) */
#define PID_ACC_MAX         LQR_ACC_MAX

/* Massimo contributo degli integrali (anti-windup) */
#define PID_INT_X_MAX       (4.0f * 0.0174533f)   /* [rad]   sull'inclinazione */
#define PID_INT_PHI_MAX     1.0f                   /* [m/s^2] sull'accelerazione */

/* Variabili di debug (Live Expressions di CubeIDE) */
extern volatile float pid_phi_stima;   /* [rad]   */
extern volatile float pid_w_stima;     /* [rad/s] */
extern volatile float pid_phi_rif;     /* [rad]   inclinazione chiesta dall'anello esterno */
extern volatile float pid_u;           /* [m/s^2] ultimo comando */

/* Da chiamare a frequenza Fs nella ISR, DOPO update_Encoder() e
   motor_aggiorna_posizione(), al posto di lqr_controller().
   In uscita: acc in impulsi/s^2 per motor_acc(). */
void pid_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc);

#endif /* PID_H */
