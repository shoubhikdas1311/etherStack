/*
 * SDADC.c
 *
 *  Created on: May 8, 2019
 *      Author: SYS5
 */
#include "SD_ADC.h"


void ADC_init(ADC_MemMapPtr ADCx)
{
	if (ADCx == ADC0_BASE_PTR)
	{
		SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK);
		SIM_SOPT7 |= (SIM_SOPT7_ADC0TRGSEL(0xE) | SIM_SOPT7_ADC0ALTTRGEN_MASK);		
	}
	else if (ADCx == ADC1_BASE_PTR)
	{
		SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK);
		SIM_SOPT7 |= (SIM_SOPT7_ADC1TRGSEL(0xE) | SIM_SOPT7_ADC1ALTTRGEN_MASK);				
	}
}
void ADC_selfCal(ADC_MemMapPtr ADCx)
{
	int16_t calVal = 0X00;
	
	ADC_SC3_REG(ADCx) |= (
							ADC_SC3_AVGE_MASK	|
							ADC_SC3_AVGS(0x11)	|
							ADC_SC3_CAL_MASK	|
							ADC_SC3_ADCO_MASK
							);
	
	calVal += ADC_CLP0_REG(ADCx);
	calVal += ADC_CLP1_REG(ADCx);
	calVal += ADC_CLP2_REG(ADCx);
	calVal += ADC_CLP3_REG(ADCx);
	calVal += ADC_CLP4_REG(ADCx);
	calVal += ADC_CLPS_REG(ADCx);
	
	calVal = calVal/2;
	
	calVal |= 0x8000;
	
	ADC_PG_REG(ADCx) = calVal;
	
	calVal = 0X00;
			
	calVal += ADC_CLM0_REG(ADCx);
	calVal += ADC_CLM1_REG(ADCx);
	calVal += ADC_CLM2_REG(ADCx);
	calVal += ADC_CLM3_REG(ADCx);
	calVal += ADC_CLM4_REG(ADCx);
	calVal += ADC_CLMS_REG(ADCx);
	
	calVal = calVal/2;
	
	calVal |= 0x8000;
	
	ADC_MG_REG(ADCx) = calVal;
	ADC_SC3_REG(ADCx) &= ~ADC_SC3_CAL_MASK; 
}
void ADC_config(ADC_MemMapPtr ADCx)
{
	ADC_CFG1_REG(ADCx) |= (
							ADC_CFG1_ADLPC_MASK		|
							ADC_CFG1_MODE((0X11))
							);
	ADC_CFG2_REG(ADCx) |= (
							ADC_CFG2_MUXSEL_MASK	|		//0 ADxxa channels are selected. 1 ADxxb channels are selected.
							ADC_CFG2_ADHSC_MASK		|		//0 Normal conversion sequence selected. 1 High speed conversion sequence selected (2 additional ADCK cycles to total conversion time).
							ADC_CFG2_ADLSTS(0x3)
							/*
							 * 00 Default longest sample time (20 extra ADCK cycles; 24 ADCK cycles total). 
							 * 01 12 extra ADCK cycles; 16 ADCK cycles total sample time. 
							 * 10 6 extra ADCK cycles; 10 ADCK cycles total sample time. 
							 * 11 2 extra ADCK cycles; 6 ADCK cycles total sample time.							 * 
							 */
							);
	ADC_SC2_REG(ADCx) = 0x00;
	ADC_SC1_REG(ADCx,A) = (
							ADC_SC1_AIEN_MASK		|
							ADC_SC1_DIFF_MASK		|
							ADC_SC1_ADCH(0x1C)
							);
}
volatile int16_t ADC_read(ADC_MemMapPtr ADCx)
{
	volatile int16_t res;
	if(!(ADC_SC1_REG(ADCx, A) & ADC_SC1_COCO_MASK)){}
	res = (int16_t)ADC_R_REG(ADCx, A);
	return res;
}
