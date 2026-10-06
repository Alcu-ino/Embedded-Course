# Report delle frequenze

## Esito

La frequenza del controllo dichiarata (`Fs = 1000 Hz`) coincide con quella effettiva di TIM3. Anche `Fclk = 84 MHz`, passato al generatore PWM di TIM2, e' coerente con il clock effettivo del timer.

La differenza da valutare e' TIM4: genera una richiesta DMA a 100 kHz, cioe' 100 volte per ogni iterazione del controllo a 1 kHz. Questo puo' essere intenzionale come campionamento piu' rapido dell'encoder; va cambiato solo se si desidera che DMA campioni a 1 kHz.

**Correzione rispetto alla precedente analisi:** avevo calcolato SYSCLK usando l'uscita PLLP. In `SystemClock_Config()` la sorgente selezionata e' invece `RCC_SYSCLKSOURCE_PLLRCLK`; di conseguenza SYSCLK e' 168 MHz, non 84 MHz. Le conclusioni precedenti secondo cui TIM3 fosse a 500 Hz e `Fclk` fosse errato non erano corrette.

## Calcolo del clock

In `SystemClock_Config()` sono impostati HSI a 16 MHz, `PLLM = 16`, `PLLN = 336`, `PLLP = 4`, `PLLR = 2` e la sorgente di sistema `PLLRCLK`.

```text
Ingresso VCO = 16 MHz / 16 = 1 MHz
VCO          = 1 MHz * 336 = 336 MHz
SYSCLK       = VCO / PLLR = 336 MHz / 2 = 168 MHz
HCLK         = SYSCLK / 1 = 168 MHz
PCLK1        = HCLK / 4 = 42 MHz
```

Quando il prescaler APB e' diverso da 1, il clock dei timer su quel bus e' il doppio di PCLK. TIM2, TIM3 e TIM4 sono sul bus APB1:

```text
Clock timer APB1 = 2 * PCLK1 = 84 MHz
```

L'uscita PLLP e' 84 MHz, ma non e' quella selezionata come SYSCLK.

## Frequenze effettive

| Uso | Configurazione | Calcolo | Risultato | Valutazione |
|---|---|---|---:|---|
| TIM3, controllo | PSC=83, ARR=999 | 84 MHz / (84 * 1000) | 1 kHz | Coincide con `Fs`; periodo 1 ms |
| TIM2, tick PWM | PSC=0 | 84 MHz / (0 + 1) | 84 MHz | Coincide con `Fclk` |
| TIM4, DMA encoder | PSC=0, ARR=839 | 84 MHz / (1 * 840) | 100 kHz | Campiona ogni 10 us; 100x il controllo |

Per TIM3 e TIM4, il periodo del contatore e' `ARR + 1`; per questo nei calcoli compare 1000 e 840, non 999 e 839.

## Impatto sul software

- `init_Encoder(..., 1.0f/Fs)` imposta `encoder.ts` a 1 ms. TIM3 richiama `update_Encoder()` ogni 1 ms, quindi il tempo usato per calcolare la velocita' dell'encoder e' coerente.
- `motor_init(..., Fclk, Fs)` passa 84 MHz e 1 kHz. TIM2 ha clock di conteggio effettivo a 84 MHz e `motor_acc()` usa un passo temporale di 1 ms: entrambi i valori sono coerenti con le configurazioni correnti.
- TIM4 produce un trasferimento DMA ogni 10 us nel buffer circolare da due elementi. Questo e' piu' rapido del controllo, ma non cambia il periodo di `update_Encoder()`: il callback di controllo continua a girare a 1 kHz.

## Cosa correggere (solo se necessario)

**Se TIM4 deve campionare a 100 kHz:** non occorre correggere i parametri di frequenza descritti sopra. Mantieni PSC=0 e ARR=839.

**Se TIM4 deve campionare a 1 kHz:** modifica in `MX_TIM4_Init()` `Prescaler` a 83 e `Period` a 999. Con clock timer a 84 MHz, la frequenza risultante e' 84 MHz / (84 * 1000) = 1 kHz. Considera che in questo caso il campionamento DMA avviene alla stessa frequenza dell'ISR di controllo.

Non modificare `Fs`, i parametri di TIM3 o `Fclk` per risolvere questa differenza: con l'attuale configurazione PLL sono gia' allineati. Se il clock PLL viene cambiato in seguito, ricalcola anche i clock APB e il raddoppio applicato ai timer.

## Riferimenti nel progetto

- `Src/main.c`: definizioni `Fclk` e `Fs`, inizializzazione di encoder/motore, configurazione del clock e parametri di TIM3/TIM4.
- `Src/motor.c`: `motor_acc()` usa `Fclk` e `Fs` per calcolare tick PWM e intervallo d'integrazione.
- `Src/encoder.c`: `update_Encoder()` divide la variazione angolare per `encoder.ts`.

Questo report e' un'analisi statica delle configurazioni sorgente; non e' stata eseguita una misura sulla scheda.