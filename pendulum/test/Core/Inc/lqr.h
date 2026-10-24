/*
 * lqr.h
 *
 *  LQR per pendolo inverso su carrello, comando in accelerazione.
 *
 *  Stato (unita' SI, le stesse del modello MATLAB):
 *      x[0] = p   [m]      posizione carrello (0 = centro corsa, dall'homing)
 *      x[1] = v   [m/s]    velocita' carrello
 *      x[2] = phi [rad]    scostamento dalla verticale alta
 *      x[3] = w   [rad/s]  velocita' angolare
 *
 *  Legge di controllo:  u = -K x   [m/s^2]
 *  Uscita per motor_acc():  u / M_PER_STEP   [impulsi/s^2]
 */
#ifndef LQR_H
#define LQR_H

#include <math.h>
#include "encoder.h"
#include "motor.h"

/* ------------------------------------------------------------------------ */
/*  Meccanica                                                               */
/* ------------------------------------------------------------------------ */
/* [m] spostamento del carrello per UN impulso su STEP:
       (angolo di un passo intero [rad]) * raggio puleggia / microstep
       = (2*pi/200) * 0.005 / 32 = 4.91e-6 m
   Le PARENTESI esterne sono indispensabili: senza, a/M_PER_STEP diventa
   a/0.0314*0.005/32 (errore di un fattore ~1e6 nel vecchio codice). */
#define M_PER_STEP   ((0.0314159f * 0.005f) / 32.0f)

/* ------------------------------------------------------------------------ */
/*  Guadagni (da MATLAB: lqr sul modello con u = accelerazione carrello)    */
/* ------------------------------------------------------------------------ */
#define LQR_K1   (-10.0000f)    /* su p   [m]     */
#define LQR_K2   (-38.2160f)    /* su v   [m/s]   */
#define LQR_K3   (-235.6668f)   /* su phi [rad]   */
#define LQR_K4   (-69.6459f)    /* su w   [rad/s] */

/* ------------------------------------------------------------------------ */
/*  Taratura                                                                */
/* ------------------------------------------------------------------------ */
/* Verso dell'encoder rispetto al carrello: +1 oppure -1.
   PROVA: carrello fermo, pendolo tenuto in alto a mano e inclinato verso il
   finecorsa toccato PER PRIMO nell'homing (verso in cui position cresce):
   encoder.angle deve SALIRE sopra 180. Se scende, mettere -1.0f.
   (Si corregge la misura, non i guadagni K: quelli sono giusti per il modello.) */
#define LQR_VERSO_ANGOLO   (+1.0f)

/* Accelerazione massima comandata [m/s^2].
   motor_acc() limita solo la VELOCITA': senza questo limite l'LQR puo'
   chiedere decine di m/s^2 e lo stepper perde passi. */
#define LQR_ACC_MAX        8.0f

/* ------------------------------------------------------------------------ */
/*  Modello linearizzato per lo stimatore (stessi parametri di MATLAB)      */
/* ------------------------------------------------------------------------ */
/*      phi'' = A21 * phi - B2 * u
   Massa concentrata a distanza l dal perno:  A21 = g/l,  B2 = 1/l.
   Asta uniforme (baricentro lc, inerzia Jp al perno): A21 = m*g*lc/Jp, B2 = m*lc/Jp. */
#define LQR_G              9.81f
#define LQR_L              0.7056f
#define LQR_A21            (LQR_G / LQR_L)
#define LQR_B2             (1.0f / LQR_L)

/* Poli dello stimatore: s^2 + 2*zeta*w0*s + w0^2.
   w0 piu' alto = stima piu' pronta ma piu' rumorosa. */
#define OSS_W0             40.0f     /* [rad/s] */
#define OSS_ZETA           0.8f

/* Se tra due chiamate passano piu' di questi ms, l'LQR era spento:
   lo stimatore riparte dalla misura (ingresso nella finestra 165-195). */
#define LQR_RIAGGANCIO_MS  3u

/* Variabili di debug (leggibili nel Live Expressions di CubeIDE) */
extern volatile float lqr_phi_stima;   /* [rad]   */
extern volatile float lqr_w_stima;     /* [rad/s] */
extern volatile float lqr_u;           /* [m/s^2] ultimo comando */

/* Da chiamare a frequenza Fs (ISR di TIM1), DOPO update_Encoder()
   e motor_aggiorna_posizione(). In uscita: acc in impulsi/s^2 per motor_acc(). */
void lqr_controller(Encoder_HandleTypeDef *encoder, motor_drv_t *motor_drv, float *acc);

#endif /* LQR_H */
