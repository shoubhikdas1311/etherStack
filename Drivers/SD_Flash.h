/*
 * SDFlash.h
 *
 *  Created on: May 4, 2019
 *      Author: SYS5
 */

#ifndef SDFLASH_H_
#define SDFLASH_H_

#include "MK60N512.h"
#define CHECK_START 	while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK)){}	\
						if(FTFL_FSTAT & (FTFL_FSTAT_ACCERR_MASK | FTFL_FSTAT_FPVIOL_MASK))FTFL_FSTAT = (FTFL_FSTAT_ACCERR_MASK | FTFL_FSTAT_FPVIOL_MASK)// return if address is outside of second programm flash block

#define CHECK_END		FTFL_FSTAT = FTFL_FSTAT_CCIF_MASK;\
						while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK)){}	\
						(FTFL_FSTAT & (FTFL_FSTAT_ACCERR_MASK | FTFL_FSTAT_FPVIOL_MASK | FTFL_FSTAT_MGSTAT0_MASK)) ? res = 0: res = -1;

void flash_init(void);
void Test_Program_Flash_RAM_Sector(uint32_t sectoraddress);
int8_t Program_Flash_Sector(uint32_t sect_add, uint16_t phrases);
int8_t Program_Longword(uint32_t flash_add, uint8_t long_word);
int8_t Erase_Flash_Block(int32_t flash_add);
int8_t Erase_Flash_Sector(uint32_t sect_add);
#endif /* SDFLASH_H_ */
