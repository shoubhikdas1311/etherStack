
/*
 * SD_PIT.c
 *
 *  Created on: May 27, 2019
 *      Author: SYS5
 */
#include "SD_PIT.h"
void PIT_init(int PITx, uint32_t delay_us)
{
	uint32_t delVal;
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT_MCR = 0X00;
	//PIT_MCR_REG(PIT_base) &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK);
	delVal = (uint32_t)delayAdapter(delay_us, 50);
	PIT_LDVAL(PITx) = delVal;
	PIT_TCTRL(PITx) |= PIT_TCTRL_TIE_MASK;
	enable_irq((68+PITx));
	set_irq_priority((68+PITx),(1+PITx));  
	PIT_TCTRL(PITx) |= PIT_TCTRL_TEN_MASK;
}
//*********************************************************************************************
int SD_PIT_Get_Current_value(int PITx)
{
	return PIT_CVAL(PITx);
}
uint32_t delayAdapter(uint32_t delay_us, uint32_t ph_clk_mhz)
{
	uint32_t retVal = delay_us * ph_clk_mhz;
	return retVal;
}
void PIT0_IRQHandler(void)
{
	PIT0_CBR();
	PIT_TFLG(0) |= PIT_TFLG_TIF_MASK;
}
void PIT1_IRQHandler(void)
{
	PIT1_CBR();
	PIT_TFLG(1) |= PIT_TFLG_TIF_MASK;
}
void PIT0_CBR(void)
{
	LED_YELLOW_ON;
	LED_GREEN_ON;
	LED_BLUE_OFF;
	LED_RED_OFF;
}
void PIT1_CBR(void)
{
	LED_YELLOW_OFF;
	LED_GREEN_OFF;
	LED_BLUE_ON;
	LED_RED_ON;
}
