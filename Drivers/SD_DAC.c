/*
 * SD_DAC.c
 *
 *  Created on: May 9, 2019
 *      Author: SYS5
 */
#include "SD_DAC.h"

void DAC_init(DAC_MemMapPtr DACx)
{
	if(DACx == DAC0_BASE_PTR)
		SIM_SCGC2 = SIM_SCGC2_DAC0_MASK;
	else if(DACx == DAC1_BASE_PTR)
		SIM_SCGC2 = SIM_SCGC2_DAC1_MASK;
	DAC_C0_REG(DACx) |= DAC_C0_DACEN_MASK;
}
void VREF_init(DAC_MemMapPtr DACx)
{
	SIM_SCGC4 |= SIM_SCGC4_VREF_MASK ;
	VREF_SC |= (
				VREF_SC_VREFEN_MASK		|
				VREF_SC_MODE_LV(0x01)
				);
	while(!(VREF_SC & VREF_SC_VREFST_MASK)){}
}
void DACx_init(DAC_MemMapPtr DACx, uint8_t Vref)
{ 
	DAC_C0_REG(DACx) |= (
							DAC_C0_DACEN_MASK		|
							DAC_C0_DACRFS_MASK		|
							DAC_C0_DACSWTRG_MASK	|
							Vref
						);
	VREF_init(DACx);
	DAC_C1_REG(DACx) |= (
							DAC_C1_DACBFMD(1)		|
							DAC_C1_DACBFWM(0)		|
							DAC_C1_DACBFEN_MASK
						);
	DAC_C2_REG(DACx) |= (
							DAC_C2_DACBFRP(0)		|
							DAC_C2_DACBFUP(0xF)
						);
}
int16_t DACx_Buff_write(DAC_MemMapPtr DACx, uint8_t DAC_indx, int16_t IpData)
{
	int16_t data,datl,dath;
	// initialize all 16 words buffer with a the value buffval
	/*datl = (IpData & 0xFF);
	 dath = ((IpData & 0xF00) >>4); */
	DAC_DATL_REG(DACx, DAC_indx)  = datl = (IpData & 0x0ff); 
	DAC_DATH_REG(DACx, DAC_indx)  = dath = ((IpData & 0xf00) >> 8); 
	/*if((DAC_DATH_REG(DACx, DAC_indx) & 0x8))
	{
		data =(int16_t)( DAC_DATL_REG(DACx, DAC_indx)|( DAC_DATH_REG(DACx, DAC_indx)<<8));
	}
	else*/
		data =(int16_t)( DAC_DATL_REG(DACx, DAC_indx)|( DAC_DATH_REG(DACx, DAC_indx)<<8) );		

	//if(temp != buffval){ while(1){} }
	if ((IpData & 0x0fff) != (data & 0x0fff)) 
	{
		printf("IpData = %d",IpData);
		printf("datl = %d\n", (int)datl);
		printf("dath = %d\n", (int)dath);                
		printf("Error in return value %d\n", data);		
	}
	else 
	{
		return data;
	}
}

void DACx_register_reset (DAC_MemMapPtr dacx_base_ptr)
{
  unsigned char dacbuff_index = 0 ;   

for (dacbuff_index=0; dacbuff_index<16;dacbuff_index++){
	DACx_Buff_write( dacx_base_ptr, dacbuff_index, 0); 
  }
   
     DAC_SR_REG(dacx_base_ptr) = DACx_SR_RESET ;
     DAC_C0_REG(dacx_base_ptr) = DACx_C0_RESET ;
     DAC_C1_REG(dacx_base_ptr) = DACx_C1_RESET;
     DAC_C2_REG(dacx_base_ptr) = DACx_C2_RESET;
} //end of DACx_register_reset_por_values
