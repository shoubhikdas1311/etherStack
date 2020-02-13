/*
 * SDFlash.c
 *
 *  Created on: May 4, 2019
 *      Author: SYS5
 */

#include "SD_Flash.h"

void flash_init(void)
{
	SIM_SCGC6 |= SIM_SCGC6_FTFL_MASK;
}
/**
* *****Test_Program_Flash_RAM_Sector***//**
* This function programs a flash sector with dummy data.
* Each flash sector is 2kB in size. This function checks the given address to be a sector border.
* The data is written to the flash programming accelleration RAM. Then the flash sector is written as a whole by hardware logic.
* This is faster than writing in 4 byte chunks.
* The hardware logic actually writes in 8 byte chunks (phrase) and one can specify how many phrases to copy.
* The RAM can hold data for one whole sector (2kB).
* @param sectoraddress This uint32_t address needs to be aligned to a sector border.
*/
void Test_Program_Flash_RAM_Sector(uint32_t sectoraddress){
	uint32_t writeaddress = 0x14000000;
	uint32_t writeaddress2 = sectoraddress;
	if( (sectoraddress & 0x000007FF) == 0 )
	{																			// return if address is not a valid 2kB sector borderreturn;
		if((sectoraddress >= 0x00040000) && (sectoraddress <= 0x0007FFFF))
		{					// return if address is outside of second programm flash block
			for( writeaddress = 0x14000000; writeaddress < 0x14000800; writeaddress += 4 )
			{
				(*((uint32_t*)writeaddress)) = writeaddress2;
				writeaddress2 += 4;
			}
			Program_Flash_Sector(sectoraddress, 256);																// copy 256 * 8 = 2048
		}
	}
}
/********Program_Flash_Sector***********//**
* This function programs the flash with the given number of phrases (8byte). The given address is not checked again, but it 
* gets truncated to be phrase aligned -> (flashaddress & 0x000000F8). The function waits for the operation to finish.
* @param sectoraddress The hardware will start writing data from RAM at this address.
* @param phrasenumber The number of phrases (8byte) to write from RAM to FLASH.
********************************************/
int8_t Program_Flash_Sector(uint32_t sect_add, uint16_t phrases)
{
	int8_t res = -1;
	CHECK_START;
	FTFL_FCCOB0 = 0X0B;
	FTFL_FCCOB1 = (uint8_t)((sect_add & 0x00FF0000) >> 16);
	FTFL_FCCOB2 = (uint8_t)((sect_add & 0x0000FF00) >> 8);
	FTFL_FCCOB3 = (uint8_t)((sect_add & 0x000000FF));
	FTFL_FCCOB4 = (uint8_t)((phrases) & 0xFF00) >> 8;
	FTFL_FCCOB5 = (uint8_t)((phrases) & 0x00FF);
	CHECK_END;
	return res;
}
/**
* *******Program_Flash_Longword********//**
* This function programs the flash with one long word. The given address is not checked again, but it gets truncated to be 
* long word aligned -> (flashaddress & 0x000000FC). The function waits for the operation to finish.
* @param flashaddress The uint32_t address that will be written.
* @param longword The uint32_t long word of data that will be written.
*/
int8_t Program_Longword(uint32_t flash_add, uint8_t long_word)
{
	int8_t res = -1;
	CHECK_START;
	FTFL_FCCOB0 = 0X06;
	FTFL_FCCOB1 = (uint8_t)((long_word & 0x00FF0000) >> 16);
	FTFL_FCCOB2 = (uint8_t)((long_word & 0x0000FF00) >> 8);
	FTFL_FCCOB3 = (uint8_t)((long_word & 0x000000FF));
	FTFL_FCCOB4 = (uint8_t)((long_word & 0xFF000000) >> 24);
	FTFL_FCCOB5 = (uint8_t)((long_word & 0x00FF0000) >> 16);
	FTFL_FCCOB6 = (uint8_t)((long_word & 0x0000FF00) >> 8);
	FTFL_FCCOB7 = (uint8_t)((long_word & 0x000000FF));
	CHECK_END;
	return res;
}

/**
* ******Erase_Flash_Block*************//**
* This function erases the given sector of the flash. The function waits for the operation to finish.
* Each flash sector is 2kB in size. This function checks the given address to be a sector border.
* @param sectoraddress The address of the sector, that will be erased, needs to be aligned to a sector border.
*/
int8_t Erase_Flash_Block(int32_t flash_add)
{
	int8_t res = -1;
	if( (flash_add & 0x000007FF) == 0 )										// Chech whether sector border or not
	{																			// return if address is not a valid 2kB sector border
		if((flash_add >= 0x00040000) && (flash_add <= 0x0007FFFF))
		{
			CHECK_START;
			FTFL_FCCOB0 = 0x08;
			FTFL_FCCOB1 = (flash_add & 0x00FF0000) >> 24;
			FTFL_FCCOB2 = (flash_add & 0x00FF0000) >> 16;
			FTFL_FCCOB3 = (flash_add & 0x00FF0000) >> 8;																		// write 1 to clear flag (executes command)
			CHECK_END;
			return res;
		}
		return res;
	}
	return res;
}

/**
* ******Erase_Flash_Sector*************//**
* This function erases the given sector of the flash. The function waits for the operation to finish.
* Each flash sector is 2kB in size. This function checks the given address to be a sector border.
* @param sectoraddress The address of the sector, that will be erased, needs to be aligned to a sector border.
*/
int8_t Erase_Flash_Sector(uint32_t sect_add)
{
	int8_t res = -1;
	if( (sect_add & 0x000007FF) == 0 )										// Chech whether sector border or not
	{																			// return if address is not a valid 2kB sector border
		if((sect_add >= 0x00040000) && (sect_add <= 0x0007FFFF))
		{
			CHECK_START;
			FTFL_FCCOB0 = 0x08;
			FTFL_FCCOB1 = (sect_add & 0x00FF0000) >> 24;
			FTFL_FCCOB2 = (sect_add & 0x00FF0000) >> 16;
			FTFL_FCCOB3 = (sect_add & 0x00FF0000) >> 8;
			CHECK_END;
			return res;
		}
		return res;
	}
	return res;
}
