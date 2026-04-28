#include "stm32f446xx.h"

#define DOUT 	0b01
#define LED_PIN 5
#define PP 0b0
#define LOW 0b00
#define PUP 0b01

void RCC_init()
{
	uint32_t tmp;
	tmp = RCC->CR;

	tmp |= ((uint32_t)RCC_CR_HSION);
	RCC->CR = tmp;

    /* wait until HSI is ready */
   	while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
   	/* Reset SWS bits and assign HSI as system clock source */
  	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;
    /* Wait till HSI is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS_HSI) != RCC_CFGR_SWS_HSI) {;} /* on one line! */
}

void GPIO_clock_init(uint32_t mask) {
    RCC->AHB1ENR |= mask;
}

void GPIO_init(GPIO_TypeDef* GPIOx, uint8_t pin)
{
	GPIOx->MODER &= ~(0b11<<(2*pin));
	GPIOx->MODER |= (DOUT<<(2*pin));

	GPIOx->OTYPER &= ~(0b1<<pin);
	GPIOx->OTYPER |= (PP << pin);

	GPIOx->OSPEEDR &= ~(0b11<<(2*pin));
	GPIOx->OSPEEDR |= (LOW<< (2*pin));

	GPIOx->PUPDR &= ~(0b11<<(2*pin));
	GPIOx->PUPDR |= (PUP<<(2*pin));
}

void toggle_led(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t counter)
{
	if(counter == 1){
		GPIOx->BSRR = (1<<pin);
	}else{
		GPIOx->BSRR = (1<<(16+pin));
	}
}

void delay_msecs(uint32_t msec)
{
	uint32_t volatile i;
	for (i = 0; i < 1000*msec ; i++) {
		asm("nop");
	}
}

int main(void)
{
	uint8_t counter = 0;
	RCC_init();
	GPIO_clock_init(RCC_AHB1ENR_GPIOAEN);
	GPIO_init(GPIOA,LED_PIN);
	while(1){
		counter ^= 1;
		toggle_led(GPIOA,LED_PIN,counter);
		delay_msecs(1000);
	}
}
