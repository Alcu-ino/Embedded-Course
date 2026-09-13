/*
 * motor.c
 *
 *  Created on: 13 set 2026
 *      Author: vito
 */
#include "debug.h"

int __io_putchar(int ch){
ITM_SendChar(ch);
return ch;
}
