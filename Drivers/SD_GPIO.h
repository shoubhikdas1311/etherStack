/*
 * GPIO.h
 *
 *  Created on: Apr 24, 2019
 *      Author: SYS5
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "MK60N512.h"
#include "SD_UART.h"
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(n) (1 << (n & GPIO_PIN_MASK))

#define LED_BLUE_ON				GPIOA_PCOR |= GPIO_PCOR_PTCO(GPIO_PIN(10));
#define LED_RED_ON				GPIOA_PCOR |= GPIO_PCOR_PTCO(GPIO_PIN(11));
#define LED_YELLOW_ON			GPIOA_PCOR |= GPIO_PCOR_PTCO(GPIO_PIN(28));
#define LED_GREEN_ON			GPIOA_PCOR |= GPIO_PCOR_PTCO(GPIO_PIN(29));

#define LED_BLUE_OFF			GPIOA_PSOR |= GPIO_PSOR_PTSO(GPIO_PIN(10));
#define LED_RED_OFF				GPIOA_PSOR |= GPIO_PSOR_PTSO(GPIO_PIN(11));
#define LED_YELLOW_OFF			GPIOA_PSOR |= GPIO_PSOR_PTSO(GPIO_PIN(28));
#define LED_GREEN_OFF			GPIOA_PSOR |= GPIO_PSOR_PTSO(GPIO_PIN(29));


#define ENABLE_GPIO_CLOCKS 		(SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define LED_BLUE_EN 			(PORTA_PCR10 = PORT_PCR_MUX(1))
#define LED_RED_EN 				(PORTA_PCR11 = PORT_PCR_MUX(1))
#define LED_YELLOW_EN 			(PORTA_PCR28 = PORT_PCR_MUX(1))
#define LED_GREEN_EN 			(PORTA_PCR29 = PORT_PCR_MUX(1))

#define LED_BLUE_TOGGLE 		(GPIOA_PTOR |= (1<<10))
#define LED_RED_TOGGLE 			(GPIOA_PTOR |= (1<<11))
#define LED_YELLOW_TOGGLE 		(GPIOA_PTOR |= (1<<28))
#define LED_GREEN_TOGGLE 		(GPIOA_PTOR |= (1<<29))

void gpio_init(void);

#endif /* GPIO_H_ */