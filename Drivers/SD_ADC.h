/*
 * SD_ADC.h
 *
 *  Created on: May 7, 2019
 *      Author: SYS5
 */

#ifndef SD_ADC_H_
#define SD_ADC_H_
#include "MK60N512.h"


#define A                 0x0
#define B                 0x1
/*
typedef struct adc_cfg {
  uint8_t  CONFIG1; 
  uint8_t  CONFIG2; 
  uint16_t COMPARE1; 
  uint16_t COMPARE2; 
  uint8_t  STATUS2;
  uint8_t  STATUS3; 
  uint8_t  STATUS1A; 
  uint8_t  STATUS1B;
  uint32_t PGA;
  } *tADC_ConfigPtr, tADC_Config ;  
  

#define CAL_BLK_NUMREC 18 

typedef struct adc_cal {
 
uint16_t  OFS;
uint16_t  PG;
uint16_t  MG;
uint8_t   CLPD;
uint8_t   CLPS;
uint16_t  CLP4;
uint16_t  CLP3;
uint8_t   CLP2;
uint8_t   CLP1;
uint8_t   CLP0;
uint8_t   dummy;
uint8_t   CLMD;
uint8_t   CLMS;
uint16_t  CLM4;
uint16_t  CLM3;
uint8_t   CLM2;
uint8_t   CLM1;
uint8_t   CLM0;
} tADC_Cal_Blk ;  
*/
void ADC_init(ADC_MemMapPtr ADCx);
volatile int16_t ADC_read(ADC_MemMapPtr ADCx);
void ADC_selfCal(ADC_MemMapPtr ADCx);
void ADC_config(ADC_MemMapPtr ADCx);

#endif /* SD_ADC_H_ */
