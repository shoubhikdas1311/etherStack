/*
 * main.h
 *
 *  Created on: Apr 18, 2019
 *      Author: SYS6_NET
 */

#ifndef MAIN_H_
#define MAIN_H_
#define JTAG_UART	UART5_BASE_PTR

#include "Drivers/SD_UART.h"
#include "Drivers/SD_Delay.h"
#include "Drivers/SD_GPIO.h"
#include "Drivers/Sys.h"
#include "Drivers/SD_WDOG.h"
#include "Drivers/packet_Handler.h"
//#include "f2a.h"
#define MK60DZ10	

void DAC12_VreferenceRamp(void);
extern uint8_t wDog_isr_usr();
void * base_Sz(char *ptr, char *indx, int base_sz, int act_bit);
int intToStr(int x, char str[], int d) ;

#endif /* MAIN_H_ */
