/****************************************************************************************************/
/**
Copyright (c) 2008 Freescale Semiconductor
Freescale Confidential Proprietary
\file       DMA.h
\brief
\author     Freescale Semiconductor
\author     Guadalajara Applications Laboratory RTAC Americas
\author     B22385
\version    0.1
\date       May 30, 2012
*/
/****************************************************************************************************/
/*                                                                                                  */
/* All software, source code, included documentation, and any implied know-how are property of      */
/* Freescale Semiconductor and therefore considered CONFIDENTIAL INFORMATION.                       */
/* This confidential information is disclosed FOR DEMONSTRATION PURPOSES ONLY.                      */
/*                                                                                                  */
/* All Confidential Information remains the property of Freescale Semiconductor and will not be     */
/* copied or reproduced without the express written permission of the Discloser, except for copies  */
/* that are absolutely necessary in order to fulfill the Purpose.                                   */
/*                                                                                                  */
/* Services performed by FREESCALE in this matter are performed AS IS and without any warranty.     */
/* CUSTOMER retains the final decision relative to the total design and functionality of the end    */
/* product.                                                                                         */
/* FREESCALE neither guarantees nor will be held liable by CUSTOMER for the success of this project.*/
/*                                                                                                  */
/* FREESCALE disclaims all warranties, express, implied or statutory including, but not limited to, */
/* implied warranty of merchantability or fitness for a particular purpose on any hardware,         */
/* software ore advise supplied to the project by FREESCALE, and or any product resulting from      */
/* FREESCALE services.                                                                              */
/* In no event shall FREESCALE be liable for incidental or consequential damages arising out of     */
/* this agreement. CUSTOMER agrees to hold FREESCALE harmless against any and all claims demands or */
/* actions by anyone on account of any damage,or injury, whether commercial, contractual, or        */
/* tortuous, rising directly or indirectly as a result of the advise or assistance supplied CUSTOMER*/
/* in connectionwith product, services or goods supplied under this Agreement.                      */
/*                                                                                                  */
/****************************************************************************************************/

/*****************************************************************************************************
* Module definition against multiple inclusion
*****************************************************************************************************/

#ifndef DMA_H_
#define DMA_H_
/*****************************************************************************************************
* Include files
*****************************************************************************************************/
//#include "../../Projecttypes.h"
#include "MK60N512.h"
#include "../common/common.h"
#include "SD_UART.h"
/*****************************************************************************************************
* Declaration of project wide TYPES
*****************************************************************************************************/
#define ERROR									0
#define OK										1
typedef struct
{
	uint32_t u32saddr;
	int16_t s16soff;
	uint16_t u16tcdAttr;
	uint32_t u32nbytes;
	uint32_t u32slast;
	uint32_t u32daddr;
	int16_t s16doff;
	uint16_t u16citer;
	uint32_t u32dlast_sga;
	uint16_t u16csr;
	uint16_t u16biter;
	uint32_t u32channelno;
	uint32_t u32loopcount;
    uint32_t u32link;
    uint32_t u32TriggerSource;
}stcd_DMA;

typedef enum
{
	DMA_TRIGGER_DISABLE = 0,
	DMA_TRIGGER_RESERVED,
	DMA_TRIGGER_UART0_RX,
	DMA_TRIGGER_UART0_TX,
	DMA_TRIGGER_UART1_RX,
	DMA_TRIGGER_UART1_TX,
	DMA_TRIGGER_UART2_RX,
	DMA_TRIGGER_UART2_TX,
	DMA_TRIGGER_UART3_RX,
	DMA_TRIGGER_UART3_TX,
	DMA_TRIGGER_UART4_RX,
	DMA_TRIGGER_UART4_TX,
	DMA_TRIGGER_UART5_RX,
	DMA_TRIGGER_UART5_TX,
	DMA_TRIGGER_I2S0_RX,
	DMA_TRIGGER_I2S0_TX,
	DMA_TRIGGER_SPI0_TX,
	DMA_TRIGGER_SPI0_RX,
	DMA_TRIGGER_SPI1_TX,
	DMA_TRIGGER_SPI1_RX,
	DMA_TRIGGER_SPI2_TX,
	DMA_TRIGGER_SPI2_RX,
	DMA_TRIGGER_I2C0,
	DMA_TRIGGER_I2C1,
	DMA_TRIGGER_FTM0_CHANNEL_0,
	DMA_TRIGGER_FTM0_CHANNEL_1,
	DMA_TRIGGER_FTM0_CHANNEL_2,
	DMA_TRIGGER_FTM0_CHANNEL_3,
	DMA_TRIGGER_FTM0_CHANNEL_4,
	DMA_TRIGGER_FTM0_CHANNEL_5,
	DMA_TRIGGER_FTM0_CHANNEL_6,
	DMA_TRIGGER_FTM0_CHANNEL_7,
	DMA_TRIGGER_FTM1_CHANNEL_0,
	DMA_TRIGGER_FTM1_CHANNEL_1,
	DMA_TRIGGER_FTM2_CHANNEL_0,
	DMA_TRIGGER_FTM2_CHANNEL_1,
	DMA_TRIGGER_EEE1588_TIMER_0,
	DMA_TRIGGER_EEE1588_TIMER_1,
	DMA_TRIGGER_EEE1588_TIMER_2,
	DMA_TRIGGER_EEE1588_TIMER_3,
	DMA_TRIGGER_ADC0,
	DMA_TRIGGER_ADC1,
	DMA_TRIGGER_CMP0,
	DMA_TRIGGER_CMP1,
	DMA_TRIGGER_CMP2,
	DMA_TRIGGER_DAC0,
	DMA_TRIGGER_DAC1,
	DMA_TRIGGER_CMT,
	DMA_TRIGGER_PDB,
	DMA_TRIGGER_PORT_A,
	DMA_TRIGGER_PORT_B,
	DMA_TRIGGER_PORT_C,
	DMA_TRIGGER_PORT_D,
	DMA_TRIGGER_PORT_E,
	DMA_TRIGGER_ALWAYS_EN_0,
	DMA_TRIGGER_ALWAYS_EN_1,
	DMA_TRIGGER_ALWAYS_EN_2,
	DMA_TRIGGER_ALWAYS_EN_3,
	DMA_TRIGGER_ALWAYS_EN_4,
	DMA_TRIGGER_ALWAYS_EN_5,
	DMA_TRIGGER_ALWAYS_EN_6,
	DMA_TRIGGER_ALWAYS_EN_7,
	DMA_TRIGGER_ALWAYS_EN_8,
	DMA_TRIGGER_ALWAYS_EN_9,
	DMA_TRIGGER_UNSUPPORT_TRIGGER
}eDMATriggerSource;

typedef enum
{
	DMA_CHANNEL_0 = 0,
	DMA_CHANNEL_1,
	DMA_CHANNEL_2,
	DMA_CHANNEL_3,
	DMA_CHANNEL_4,
	DMA_CHANNEL_5,
	DMA_CHANNEL_6,
	DMA_CHANNEL_7,
	DMA_CHANNEL_8,
	DMA_CHANNEL_9,
	DMA_CHANNEL_10,
	DMA_CHANNEL_11,
	DMA_CHANNEL_12,
	DMA_CHANNEL_13,
	DMA_CHANNEL_14,
	DMA_CHANNEL_15,
}eDMAChannels;
/*****************************************************************************************************
* Definition of project wide VARIABLES
*****************************************************************************************************/

/*****************************************************************************************************
* Definition of project wide MACROS / #DEFINE-CONSTANTS
*****************************************************************************************************/
#define DMA_ENABLE_REQ(dmach)	(DMA_ERQ |= (1<<dmach))

#define DMA_LINK_MINOR_TRUE			(DMA_CITER_ELINKYES_ELINK_MASK)
#define DMA_LINK_MINOR_CHANNEL(x)	(DMA_CITER_ELINKYES_LINKCH(x))
/*****************************************************************************************************
* Declaration of project wide FUNCTIONS
*****************************************************************************************************/

/*****************************************************************************************************
* Declaration of module wide FUNCTIONs - NOT for use in other modules
*****************************************************************************************************/

uint32_t u32fnDMA_ChannelInit(stcd_DMA * psDMAInit, void(* vfnDMAChCallback)(void));
/*extern void vfnDMAChannel0_ISR (void);
extern void vfnDMAChannel1_ISR (void);*/

#endif /*DMA_H_*/

