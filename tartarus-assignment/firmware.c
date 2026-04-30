/* Author: Abadessa Lorenzo, Cafagno Vito, Dargenio Ferdinando, Fornarelli Marco  
 * This source is part of the Tatarus MCU assignment for the 
 * Embedded Control course at POLIBA. 
 */

#include "mcu-api.h"
#include "lib-periph.h"
#include "buffer.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MY_REG ((volatile periphReg *) FOO_PERIPH)

/* Recall FOO_PERIPH is the address to the memory mapped registers of the peripheral */
/* Of course you can implement functions to be called in the main if you need */

void main() 
{
	Buffer* buffer;
	MCU_STEP;
	int rc = -1;
	MCU_STEP;
	bool ready = false;
	MCU_STEP;
	uint32_t read = 0;
	MCU_STEP;
	uint64_t sum = 0;
	uint32_t mea = 0;
	MCU_STEP;
	printf("Hello Tartarus MCU world!\n");
	MCU_STEP; /* This simulates that you have performed on tick. It's just an example */

	rc = foo_periph_init(MY_REG,63,65);
	MCU_STEP;
	
	if(rc == 1){
		MCU_STEP;
		printf("HAVE WAITED FOR TOO LONG PERIPHERAL");
	}
	else if (rc == 2)
	{	
		MCU_STEP;
		MCU_STEP;
		printf("PIN CHOSEN NOT IN RANGE 32-143");
	}
	else if (rc == 0){
		MCU_STEP;
		MCU_STEP;
		MCU_STEP;
		printf("PERIPH READY... STARTING PROGRAM");
		MCU_STEP;
		ready = true;
	}
	else{
		MCU_STEP;
		MCU_STEP;
		MCU_STEP;
		printf("UNKNOWN ERROR");
	}
	MCU_STEP;
	buffer = initialize(10);
	MCU_STEP;
	while(ready){
		MCU_STEP;
		
		while(MY_REG->CTRL.BUSY == 1){
			MCU_STEP;
			printf("ATTENDO DISPONIBILITA PER READ\n");
			MCU_STEP;
		}

		read = foo_periph_rx_data(MY_REG);
		printf("READ : %u\n", read);
		MCU_STEP;
		if(add(buffer, read) == 1){
			MCU_STEP;
			sum = 0;
			for(size_t i = 0; i < buffer->size; i++)
			{
				MCU_STEP;
				sum += (buffer->data[i]);
				MCU_STEP;
			}
			MCU_STEP;
			mea = sum/(buffer->capacity);
			MCU_STEP;
			printf("MEAN : %u\n",mea);
			while(MY_REG->CTRL.BUSY == 1){
				MCU_STEP;
				printf("ATTENDO DISPONIBILITA PER WRITE DELLA MEAN\n");
				MCU_STEP;
			}
			foo_periph_tx_data(MY_REG, mea);
			MCU_STEP;
			reset(buffer);
			MCU_STEP;
		}
		MCU_STEP;
	}
}

