/* Author: Abadessa Lorenzo, Cafagno Vito, Dargenio Ferdinando, Fornarelli Marco  
 * This source is part of the Tatarus MCU assignment for the 
 * Embedded Control course at POLIBA. 
 */

#include <stdio.h>
#include <stdint.h>

#ifndef __LIBPERIPH_H__
#define __LIBPERIPH_H__

typedef struct{
    struct{
        uint32_t WRITE      :1;
        uint32_t READ       :1;
        uint32_t BUSY       :1;
        uint32_t TX         :8;
        uint32_t RX         :8;
        uint32_t RESERVED   :13;
    }__attribute__((packed)) CTRL;
    uint32_t RXDATA;
    uint32_t TXDATA;
}__attribute__((packed)) periphReg;

uint8_t foo_periph_init(volatile periphReg* perifReg, uint8_t tx_pin, uint8_t rx_pin);
void foo_periph_tx_data(volatile periphReg* perifReg, uint32_t data);
uint32_t foo_periph_rx_data(volatile periphReg* perifReg);


#endif
