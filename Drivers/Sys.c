/*
 * Sys.c
 *
 *  Created on: Nov 19, 2019
 *      Author: SD
 */

#include "Sys.h"
extern uint8_t flg, flg_prev;
extern uint32_t jiffies, TimeSlice;
void jiffiesInit()
{
	SysTick_CSR_REG(SysTick_BASE_PTR) = 0;											// disable SysTick during setup
	SysTick_CVR_REG(SysTick_BASE_PTR) = 0;											// any write to current clears it
	SCB_SHPR3 = ((SCB_SHPR3 & 0x00FFFFFF)| 0xE0000000); 							// priority 7
	//SysTick_CSR_REG(SysTick_BASE_PTR) |= SysTick_CSR_CLKSOURCE_MASK;
}
void launchJiffies()
{
	SysTick_CSR_REG(SysTick_BASE_PTR) |= (
											SysTick_CSR_CLKSOURCE_MASK	|												//SysTick EXCEPTION
											SysTick_CSR_TICKINT_MASK	|												//pROCESSOR CLK
											SysTick_CSR_ENABLE_MASK														//Enable counter
										); // enable, core clock and interrupt arm
	SysTick_RVR_REG(SysTick_BASE_PTR) = TimeSlice - 1; // reload value
	
}

void SysTick_Handler()
{
	jiffies++;
	flg ^= 0x01;
	LED_BLUE_TOGGLE;
}
