/* Author: Abbadessa Lorenzo, Cafagno Vito, Dargenio Ferdinando, Fornarelli Alessandro  
 * This source is part of the Tatarus MCU assignment for the 
 * Embedded Control course at POLIBA. 
 */
#include "lib-periph.h"
#define SUCCESS            0
#define ERR_TOO_LONG_WAIT  1
#define INVALID_PIN        2

uint8_t foo_periph_init(volatile periphReg* perifReg, uint8_t tx_pin, uint8_t rx_pin)
{
    uint32_t count = 0;

    while(perifReg->CTRL.BUSY == 1){
        count++;
        if(count > 10000000){
            return ERR_TOO_LONG_WAIT;
        }
    }
    
    if(((rx_pin>=32) && (rx_pin<=143))&&((tx_pin>=32) && (tx_pin<=143))&& rx_pin!=tx_pin){
        perifReg->CTRL.RX = tx_pin;
        perifReg->CTRL.TX = rx_pin;
        return 0;
    }
    
    perifReg->CTRL.READ = 0;
    perifReg->CTRL.WRITE = 0;
    
    return INVALID_PIN;
}

void foo_periph_tx_data(volatile periphReg* perifReg, uint32_t data){
    perifReg->CTRL.READ = 0;
    perifReg->TXDATA &= 0;
    perifReg->TXDATA |= data; 
    perifReg->CTRL.WRITE = 1;
}

uint32_t foo_periph_rx_data(volatile periphReg* perifReg){
    perifReg->CTRL.WRITE = 0;
    perifReg->CTRL.READ = 1;
    return  perifReg->RXDATA;
}