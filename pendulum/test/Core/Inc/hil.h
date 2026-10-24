/*
 * hil.h
 *
 *  Hardware-in-the-loop su USART2 (ST-LINK Virtual COM Port della Nucleo F446RE,
 *  PA2 = TX, PA3 = RX): il controllore gira sulla scheda, il pendolo e il
 *  carrello sono simulati in MuJoCo (hil_mujoco.py).
 *
 *  Tutto NON bloccante:
 *    - RX: DMA1 Stream5 / Channel 4 in modo circolare. Nessun interrupt: i byte
 *          arrivati vengono letti e decodificati nell'ISR di TIM1 (hil_aggiorna_encoder).
 *    - TX: interrupt TXE di USART2 su un buffer di una trama (~0.22 ms a 921600 baud).
 *          (DMA1 Stream6, l'unico stream per USART2_TX, e' gia' usato da TIM4_UP.)
 *  Si usano direttamente i registri: il modulo HAL_UART non serve (e' disabilitato
 *  in stm32f4xx_hal_conf.h) e non va rigenerato nulla da CubeMX.
 *
 *  Sincronizzazione: la scheda resta il "clock" del sistema. A ogni tick di TIM1
 *  manda una trama di stato (accelerazione, velocita', posizione del carrello);
 *  il simulatore avanza di UN passo di controllo per ogni trama ricevuta e
 *  risponde con l'angolo. Il tempo simulato coincide quindi con i tick della
 *  scheda, indipendentemente dai ritardi del PC.
 *
 *  ---------------------------------------------------------------------------
 *  PROTOCOLLO (little endian, CRC-8 poly 0x07 sui byte tra intestazione e CRC)
 *
 *  PC -> scheda, 8 byte, "angolo":
 *      [0]  0xAA
 *      [1]  0x55
 *      [2]  seq     uint16   numero del passo di simulazione
 *      [4]  cnt     uint16   angolo in conteggi encoder 0..CPR*RES-1
 *                            (stessa scala di TIM3->CNT: 0 = giu', 1200 = su)
 *      [6]  flags   uint8    bit0 RESET, bit1 FINECORSA
 *      [7]  crc     uint8
 *
 *  scheda -> PC, 20 byte, "stato":
 *      [0]  0xAA
 *      [1]  0x55
 *      [2]  seq     uint16   tick della scheda
 *      [4]  ack     uint16   seq dell'ultimo angolo usato dal controllore
 *      [6]  acc     float32  accelerazione comandata al carrello [m/s^2]
 *      [10] v       float32  velocita' del carrello (fcurrent)   [m/s]
 *      [14] p       float32  posizione del carrello (TIM5)       [m]
 *      [18] stato   uint8    bit0 LQR attivo, bit1 guasto, bit2 in attesa del
 *                            simulatore, bit3 reset eseguito in questo tick
 *      [19] crc     uint8
 *  ---------------------------------------------------------------------------
 */
#ifndef HIL_H
#define HIL_H

#include "main.h"
#include "encoder.h"
#include "motor.h"
#include <stdint.h>
#include <stdbool.h>

/* 1 = angolo dal simulatore via USART2, 0 = encoder reale (firmware originale).
   Si puo' anche definire dal compilatore (-DHIL_ATTIVO=0). */
#ifndef HIL_ATTIVO
#define HIL_ATTIVO          1
#endif

/* Baud rate (deve coincidere con --baud di hil_mujoco.py).
   Il BRR si calcola a runtime da HAL_RCC_GetPCLK1Freq(). */
#define HIL_BAUD            921600u

/* Tick consecutivi senza un angolo nuovo oltre i quali il collegamento e'
   considerato perso: carrello fermo, si aspetta un nuovo RESET dal PC. */
#define HIL_TIMEOUT_TICK    100u

/* Buffer circolare del DMA di ricezione [byte] */
#define HIL_RX_BUF          256u

/* Priorita' NVIC dell'interrupt di USART2 (TX) */
#define HIL_IRQ_PRIO        1u

/* Protocollo */
#define HIL_SYNC0           0xAAu
#define HIL_SYNC1           0x55u
#define HIL_LEN_ANGOLO      8u
#define HIL_LEN_STATO       20u

#define HIL_FLAG_RESET      0x01u
#define HIL_FLAG_FINECORSA  0x02u

#define HIL_ST_LQR          0x01u
#define HIL_ST_GUASTO       0x02u
#define HIL_ST_ATTESA       0x04u
#define HIL_ST_RESET        0x08u

typedef enum {
    HIL_OK      = 0,    /* angolo valido nell'encoder: il controllo puo' girare   */
    HIL_ATTESA  = 1,    /* simulatore non collegato o muto: tenere fermo il carrello */
    HIL_RESET   = 2     /* il PC ha chiesto il reset: azzerare lo stato del controllo */
} hil_esito_t;

/* Diagnostica (leggibile nel Live Expressions di CubeIDE) */
typedef struct {
    uint32_t trame_ok;        /* trame angolo valide ricevute                     */
    uint32_t trame_crc_err;   /* trame scartate per CRC errato                    */
    uint32_t tick_senza_dati; /* tick in cui e' stato riusato l'angolo precedente */
    uint32_t tx_saltate;      /* trame di stato non inviate (TX ancora occupata)  */
    uint32_t reset;           /* reset ricevuti                                   */
    uint16_t ultimo_seq;      /* seq dell'ultimo angolo usato                     */
    uint16_t ultimo_cnt;      /* cnt dell'ultimo angolo usato                     */
} hil_diag_t;

extern volatile hil_diag_t hil_diag;

/* Configura USART2 + DMA (da chiamare una volta, prima di avviare TIM1). */
void hil_init(uint32_t baud);

/* Al posto di update_Encoder(): legge i byte arrivati (senza attendere),
   e se c'e' un angolo nuovo aggiorna angle, rad_angle, prev_angle e w
   esattamente come farebbe l'encoder. Senza dati nuovi l'angolo resta quello
   del tick precedente (e anche w, invece di azzerarsi). */
hil_esito_t hil_aggiorna_encoder(Encoder_HandleTypeDef *encoder);

/* true una sola volta, sul fronte di salita del finecorsa simulato. */
bool hil_finecorsa(void);

/* Accoda la trama di stato e ritorna subito (la trasmissione avviene
   nell'interrupt di USART2). acc_impulsi in impulsi/s^2, come la variabile acc
   del main: la conversione in m/s^2 la fa la funzione. */
void hil_invia_stato(const motor_drv_t *motor_drv, float acc_impulsi,
                     uint8_t control_on, uint8_t guasto, hil_esito_t esito);

#endif /* HIL_H */
