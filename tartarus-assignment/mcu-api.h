#ifndef __MCUAPI_H__
#define __MCUAPI_H__

extern void *regs;
void mcu_one_iteration();

#define FOO_PERIPH (regs)
//ADDR. IN MEMORY POINTING A BASE ADDRESS DELLA PERIFERICA
#define MCU_STEP mcu_one_iteration() 
//SIMULA CLOCK
#endif
