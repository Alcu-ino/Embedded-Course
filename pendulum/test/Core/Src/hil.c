/*
 * hil.c
 *
 *  Hardware-in-the-loop su USART2: vedi hil.h per protocollo e architettura.
 *
 *  Risorse usate (nessuna in conflitto con il firmware esistente):
 *    USART2 (PA2/PA3, gia' configurati AF7 da MX_GPIO_Init)
 *    DMA1 Stream5 Channel 4 (USART2_RX)      - Stream6 resta a TIM4_UP
 *    USART2_IRQn (solo TXE)                  - USART2_IRQHandler e' definito qui
 */
#include "hil.h"
#include "lqr.h"          /* M_PER_STEP */
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile hil_diag_t hil_diag;

/* ------------------------------------------------------------------------ */
/*  Ricezione (DMA circolare + parser)                                      */
/* ------------------------------------------------------------------------ */
static volatile uint8_t rx_buf[HIL_RX_BUF];
static uint16_t rx_rd = 0u;                      /* prossimo byte da leggere   */
static uint8_t  rx_frame[HIL_LEN_ANGOLO];
static uint8_t  rx_idx = 0u;

/* Ultima trama valida decodificata (non ancora consumata) */
static uint8_t  rx_nuova    = 0u;
static uint16_t rx_seq      = 0u;
static uint16_t rx_cnt      = 0u;
static uint8_t  rx_flags    = 0u;   /* flag dell'ultima trama               */
static uint8_t  rx_reset    = 0u;   /* RESET visto in una qualsiasi trama   */

/* Stato del collegamento */
static uint8_t  collegato        = 0u;
static uint16_t seq_usato        = 0u;
static uint32_t tick_senza_nuovi = 0u;
static uint8_t  finecorsa_prec   = 0u;
static uint8_t  finecorsa_evento = 0u;

/* ------------------------------------------------------------------------ */
/*  Trasmissione (interrupt TXE)                                            */
/* ------------------------------------------------------------------------ */
static volatile uint8_t tx_buf[HIL_LEN_STATO];
static volatile uint8_t tx_idx = 0u;
static volatile uint8_t tx_len = 0u;            /* 0 = trasmettitore libero    */
static uint16_t tx_seq = 0u;

/* ------------------------------------------------------------------------ */
static uint8_t crc8(const volatile uint8_t *d, uint32_t n)
{
    uint8_t c = 0u;
    for (uint32_t i = 0; i < n; i++) {
        c ^= d[i];
        for (uint8_t b = 0; b < 8u; b++) {
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
        }
    }
    return c;
}

static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

/* ========================================================================== */
void hil_init(uint32_t baud)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 = TX, PA3 = RX (gia' impostati da MX_GPIO_Init; pull-up su RX
       per non leggere rumore a linea scollegata) */
    g.Pin       = USART_TX_Pin | USART_RX_Pin;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    /* --- USART2: 8N1, oversampling 16 --- */
    USART2->CR1 = 0u;
    USART2->CR2 = 0u;
    USART2->CR3 = 0u;
    const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    USART2->BRR = (pclk + baud / 2u) / baud;    /* BRR = USARTDIV*16 = fpclk/baud */

    /* --- DMA1 Stream5 Ch4: USART2->DR -> rx_buf, circolare, byte --- */
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) { }
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->PAR  = (uint32_t)&USART2->DR;
    DMA1_Stream5->M0AR = (uint32_t)rx_buf;
    DMA1_Stream5->NDTR = HIL_RX_BUF;
    DMA1_Stream5->FCR  = 0u;                                  /* modo diretto   */
    DMA1_Stream5->CR   = (4u << DMA_SxCR_CHSEL_Pos)           /* Channel 4      */
                       | DMA_SxCR_PL_0                        /* priorita' media*/
                       | DMA_SxCR_MINC
                       | DMA_SxCR_CIRC;                       /* periph->mem, 8 bit */
    DMA1_Stream5->CR  |= DMA_SxCR_EN;
    rx_rd = 0u;

    USART2->CR3 = USART_CR3_DMAR;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* Interrupt solo per la trasmissione (TXEIE abilitato da hil_invia_stato) */
    HAL_NVIC_SetPriority(USART2_IRQn, HIL_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    memset((void *)&hil_diag, 0, sizeof(hil_diag));
    collegato = 0u;
}

/* -------------------------------------------------------------------------- */
/*  Parser: consuma tutti i byte scritti dal DMA dall'ultima chiamata.         */
/*  Tiene l'ULTIMA trama valida; RESET viene ricordato anche se seguito da     */
/*  altre trame nello stesso tick.                                             */
/* -------------------------------------------------------------------------- */
static void rx_poll(void)
{
    uint16_t wr = (uint16_t)(HIL_RX_BUF - DMA1_Stream5->NDTR);
    if (wr >= HIL_RX_BUF) wr = 0u;

    while (rx_rd != wr) {
        const uint8_t b = rx_buf[rx_rd];
        rx_rd = (uint16_t)((rx_rd + 1u) % HIL_RX_BUF);

        if (rx_idx == 0u) {
            if (b == HIL_SYNC0) rx_frame[rx_idx++] = b;
        } else if (rx_idx == 1u) {
            if (b == HIL_SYNC1)      rx_frame[rx_idx++] = b;
            else if (b == HIL_SYNC0) rx_idx = 1u;           /* AA AA 55 ...  */
            else                     rx_idx = 0u;
        } else {
            rx_frame[rx_idx++] = b;
            if (rx_idx == HIL_LEN_ANGOLO) {
                rx_idx = 0u;
                if (crc8(&rx_frame[2], HIL_LEN_ANGOLO - 3u) == rx_frame[HIL_LEN_ANGOLO - 1u]) {
                    rx_seq   = (uint16_t)(rx_frame[2] | (rx_frame[3] << 8));
                    rx_cnt   = (uint16_t)(rx_frame[4] | (rx_frame[5] << 8));
                    rx_flags = rx_frame[6];
                    if (rx_flags & HIL_FLAG_RESET) rx_reset = 1u;
                    rx_nuova = 1u;
                    hil_diag.trame_ok++;
                } else {
                    hil_diag.trame_crc_err++;
                }
            }
        }
    }
}

/* Converte i conteggi nello stesso modo di update_Encoder() */
static void imposta_angolo(Encoder_HandleTypeDef *enc, uint16_t cnt)
{
    const uint32_t giro = enc->cpr * enc->res;
    if (cnt >= giro) cnt = (uint16_t)(cnt % giro);
    const float k = 360.0f / (float)giro;
    enc->angle     = (float)cnt * k;
    enc->rad_angle = enc->angle * (float)M_PI / 180.0f;
}

/* ========================================================================== */
hil_esito_t hil_aggiorna_encoder(Encoder_HandleTypeDef *encoder)
{
    rx_poll();

    /* ---- RESET dal PC: riparte tutto da questo angolo, velocita' nulla ---- */
    if (rx_reset) {
        rx_reset = 0u;
        rx_nuova = 0u;
        imposta_angolo(encoder, rx_cnt);
        encoder->prev_angle = encoder->angle;
        encoder->w          = 0.0f;
        seq_usato        = rx_seq;
        tick_senza_nuovi = 0u;
        finecorsa_prec   = 0u;
        finecorsa_evento = 0u;
        collegato        = 1u;
        hil_diag.reset++;
        hil_diag.ultimo_seq = rx_seq;
        hil_diag.ultimo_cnt = rx_cnt;
        return HIL_RESET;
    }

    if (!collegato) {
        rx_nuova = 0u;                     /* trame senza RESET: ignorate      */
        return HIL_ATTESA;
    }

    const int16_t dseq = (int16_t)(rx_seq - seq_usato);
    if (rx_nuova && dseq > 0) {
        rx_nuova = 0u;

        /* l'angolo usato nel controllo precedente diventa il vecchio */
        encoder->prev_angle = encoder->angle;
        imposta_angolo(encoder, rx_cnt);

        /* velocita' sul tempo SIMULATO trascorso (dseq passi): se un angolo
           arriva in ritardo non nasce un picco doppio su w */
        float delta = (encoder->angle - encoder->prev_angle) * (float)M_PI / 180.0f;
        delta = wrap_pi(delta);
        encoder->w = delta / ((float)dseq * encoder->ts);

        seq_usato        = rx_seq;
        tick_senza_nuovi = 0u;
        hil_diag.ultimo_seq = rx_seq;
        hil_diag.ultimo_cnt = rx_cnt;

        /* finecorsa simulato: evento sul fronte di salita */
        const uint8_t fc = (rx_flags & HIL_FLAG_FINECORSA) ? 1u : 0u;
        if (fc && !finecorsa_prec) finecorsa_evento = 1u;
        finecorsa_prec = fc;
    } else {
        /* nessun angolo nuovo: si tiene l'ultimo (angle e w invariati) */
        rx_nuova = 0u;
        hil_diag.tick_senza_dati++;
        if (++tick_senza_nuovi > HIL_TIMEOUT_TICK) {
            collegato = 0u;
            return HIL_ATTESA;
        }
    }
    return HIL_OK;
}

bool hil_finecorsa(void)
{
    if (finecorsa_evento) {
        finecorsa_evento = 0u;
        return true;
    }
    return false;
}

/* ========================================================================== */
void hil_invia_stato(const motor_drv_t *motor_drv, float acc_impulsi,
                     uint8_t control_on, uint8_t guasto, hil_esito_t esito)
{
    if (tx_len != 0u) {                 /* trama precedente ancora in uscita */
        hil_diag.tx_saltate++;
        return;
    }

    const float acc = acc_impulsi * M_PER_STEP;
    const float v   = motor_drv->fcurrent * M_PER_STEP;
    const float p   = (float)motor_drv->position * M_PER_STEP;

    uint8_t st = 0u;
    if (control_on)            st |= HIL_ST_LQR;
    if (guasto)                st |= HIL_ST_GUASTO;
    if (esito == HIL_ATTESA)   st |= HIL_ST_ATTESA;
    if (esito == HIL_RESET)    st |= HIL_ST_RESET;

    uint8_t f[HIL_LEN_STATO];
    f[0] = HIL_SYNC0;
    f[1] = HIL_SYNC1;
    f[2] = (uint8_t)(tx_seq & 0xFFu);
    f[3] = (uint8_t)(tx_seq >> 8);
    f[4] = (uint8_t)(seq_usato & 0xFFu);
    f[5] = (uint8_t)(seq_usato >> 8);
    memcpy(&f[6],  &acc, 4);            /* Cortex-M4: little endian come il PC */
    memcpy(&f[10], &v,   4);
    memcpy(&f[14], &p,   4);
    f[18] = st;
    f[19] = crc8(&f[2], HIL_LEN_STATO - 3u);
    tx_seq++;

    for (uint32_t i = 0; i < HIL_LEN_STATO; i++) tx_buf[i] = f[i];
    tx_idx = 0u;
    tx_len = HIL_LEN_STATO;
    USART2->CR1 |= USART_CR1_TXEIE;     /* il resto lo fa l'interrupt */
}

/* -------------------------------------------------------------------------- */
/*  Interrupt USART2: un byte per TXE. Non c'e' un USART2_IRQHandler in        */
/*  stm32f4xx_it.c (USART2 non e' attivo in CubeMX); se in futuro lo si        */
/*  abilita da CubeMX, togliere questa funzione o quella generata.             */
/* -------------------------------------------------------------------------- */
void USART2_IRQHandler(void)
{
    const uint32_t sr = USART2->SR;

    if ((USART2->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_idx < tx_len) {
            USART2->DR = tx_buf[tx_idx++];
        }
        if (tx_idx >= tx_len) {
            USART2->CR1 &= ~USART_CR1_TXEIE;
            tx_len = 0u;                /* l'ultimo byte e' nel registro di shift */
        }
    }
}
