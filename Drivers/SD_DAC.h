/*
 * SD_DAC.h
 *
 *  Created on: May 9, 2019
 *      Author: SYS5
 */

#ifndef SD_DAC_H_
#define SD_DAC_H_

#include "MK60N512.h"
#include "../common/common.h"

#define DACREF_1 0X00
#define DACREF_2 0X01
 

/* DAXx registers reset values*/
#define DACx_DAT_RESET 0
#define DACx_SR_RESET 2
#define DACx_C0_RESET 0
#define DACx_C1_RESET 0
#define DACx_C2_RESET 15 //0x0f
#define ERROR -1

void DAC_init(DAC_MemMapPtr DACx);
void VREF_init(DAC_MemMapPtr DACx);
void DACx_init(DAC_MemMapPtr DACx, uint8_t Vref);
int16_t DACx_Buff_write(DAC_MemMapPtr DACx, uint8_t DAC_indx, int16_t IpData);
void DACx_register_reset (DAC_MemMapPtr dacx_base_ptr);

#endif /* SD_DAC_H_ */
