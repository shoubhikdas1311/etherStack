/*
 * SD_PIT.h
 *
 *  Created on: May 27, 2019
 *      Author: SYS5
 */

#ifndef SD_PIT_H_
#define SD_PIT_H_

#include "MK60N512.h"
#include "SD_GPIO.h"

void PIT_init(int PITx, uint32_t delay_us);
int SD_PIT_Get_Current_value(int PITx);
uint32_t delayAdapter(uint32_t delay_us, uint32_t ph_clk_mhz);
void PIT0_CBR(void);
void PIT1_CBR(void);

#endif /* SD_PIT_H_ */
