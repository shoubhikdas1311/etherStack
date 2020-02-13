/*
 * GPIO.c
 *
 *  Created on: Apr 24, 2019
 *      Author: SYS5
 */

#include "SD_GPIO.h"

void gpio_init()
{	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	GPIOA_PDDR = GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
	GPIOA_PSOR=	GPIO_PSOR_PTSO(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
	
	PORTA_PCR10 = PORT_PCR_MUX(1);
	PORTA_PCR11 = PORT_PCR_MUX(1);
	PORTA_PCR28 = PORT_PCR_MUX(1);
	PORTA_PCR29 = PORT_PCR_MUX(1);		
}
void LEDs_On(void)
{
#ifdef CPU_MK60N512VMD100
  GPIOA_PDDR = (1<<10)|(1<<11)|(1<<28)|(1<<29);
#else
  GPIOB_PDDR = (1<<11);
  GPIOC_PDDR = ((1<<7)|(1<<8)|(1<<9));
#endif
}

void LED_Dir_Out(void)
{
#ifdef CPU_MK60N512VMD100
  GPIOA_PDOR &= ~((1<<10)|(1<<11)|(1<<28)|(1<<29));
#else
  GPIOB_PDOR &= ~(1<<11);
  GPIOC_PDOR &= ~((1<<7)|(1<<8)|(1<<9));
#endif
}