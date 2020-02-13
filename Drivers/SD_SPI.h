/*
 * spi.h
 *
 *  Created on: Feb 29, 2016
 *      Author: SD
 */

#ifndef SPI_H_
#define SPI_H_

#include "../common/common.h"

/************************@group: SPI */

/**Sub: SPI Base addresses*/
#define 	SPI0 	SPI0_BASE_PTR
#define 	SPI1 	SPI1_BASE_PTR
#define 	SPI2 	SPI2_BASE_PTR

/**Sub: SPI Board specific pins control registers*/
#define	SPI0_SIN_PIN_REG 		PORTC_PCR7
#define 	SPI0_SOUT_PIN_REG 	     PORTC_PCR6
#define 	SPI0_SCLK_PIN_REG 	     PORTC_PCR5
#define	SPI1_SIN_PIN_REG 	
#define 	SPI1_SOUT_PIN_REG 	
#define 	SPI1_SCLK_PIN_REG 	
#define	SPI2_SIN_PIN_REG 	
#define 	SPI2_SOUT_PIN_REG 	
#define 	SPI2_SCLK_PIN_REG 	

/**Sub: SPI Board specific pins Alternate functions*/

#define		SPI0_SIN_PIN_ALT 		0x02
#define 	SPI0_SOUT_PIN_ALT		0x02
#define 	SPI0_SCLK_PIN_ALT 	     0x02
#define		SPI1_SIN_PIN_ALT 	
#define 	SPI1_SOUT_PIN_ALT	
#define 	SPI1_SCLK_PIN_ALT 	
#define		SPI2_SIN_PIN_ALT 	
#define 	SPI2_SOUT_PIN_ALT	
#define 	SPI2_SCLK_PIN_ALT 	
/*******@End group:SPI*/




//==================================================================================================
//Driver referenced from k60 forms
//==================================================================================================



#define SPI_ENABLE_MASK			           (0xFFFFBFFEU)
#define SPI_DISABLE_MASK			           (1U)

#define SPI_CLR_RXFIFO		           (1U)
#define SPI_CLR_TXFIFO		           (1U)

#define SPI_DISABLE_RXFIFO	           (1U)
#define SPI_DISABLE_TXFIFO  	           (1U)

#define SPI_ENABLE_RXFIFO	           (0U)
#define SPI_ENABLE_TXFIFO  	           (0U)

#define SPI_PERI_CHIP_SEL_INACTV_LOW     (0U)
#define SPI_PERI_CHIP_SEL_INACTV_HIGH    (1U)

#define SPI_RECV_ENABLE                  (1U)
#define SPI_RECV_DISABLE                 (0U)

#define SPI_MASTER_MODE		           (1U)
#define SPI_SLAVE_MODE			      (0U)

#define FRAME_SIZE8				  	 (7U)
#define FRAME_SIZE16				 (15U)

#define LSB_FIRST	           		 (1U)
#define MSB_FIRST	           		 (0U)

#define CLK_PHASE0           			 (0U)
#define CLK_PHASE1            		 (1U)

#define CLK_POLARITY0         		 (0U)
#define CLK_POLARITY1         		 (1U)

#define PCSSCK_PRESCALER_1			 (0U)
#define PCSSCK_PRESCALER_3			 (1U)
#define PCSSCK_PRESCALER_5			 (2U)
#define PCSSCK_PRESCALER_7			 (3U)

#define PASC_PRESCALER_1				 (0U)
#define PASC_PRESCALER_3				 (1U)
#define PASC_PRESCALER_5				 (2U)
#define PASC_PRESCALER_7				 (3U)

#define PDT_PRESCALER_1				 (0U)
#define PDT_PRESCALER_3				 (1U)
#define PDT_PRESCALER_5				 (2U)
#define PDT_PRESCALER_7				 (3U)

#define ASC_SCALER_1				 (1U)
#define ASC_SCALER_2				 (2U)
#define ASC_SCALER_3				 (3U)
#define ASC_SCALER_4				 (4U)
#define ASC_SCALER_5				 (5U)
#define ASC_SCALER_6				 (6U)
 
#define DT_SCALER_1					 (1U)
#define DT_SCALER_2					 (2U)
#define DT_SCALER_3					 (3U)
#define DT_SCALER_4					 (4U)
#define DT_SCALER_5					 (5U)
#define DT_SCALER_6					 (6U)
 
#define PCS_PRESCALER_2				 (0U)
#define PCS_PRESCALER_4				 (1U)
#define PCS_PRESCALER_8				 (2U)
#define PCS_PRESCALER_16				 (3U)
#define PCS_PRESCALER_32				 (4U)
#define PCS_PRESCALER_64				 (5U)


#define SPI_DATA_RECEIVED			 (0x20000U)
#define SPI_TX_READY				 (0x2000000U)

#define TX_FIFO_EMPTY				 (0x2000000U)
#define RX_FIFO_EMPTY				 (0x20000U)




/*
*******************************************************************************
*                         GLOBAL VARIABLE DECLARATION
*******************************************************************************
*/

typedef struct
{
	uint8_t *data_buff_ptr;
	uint8_t data_len;
	uint8_t data_buff_status;

}spi_tx_data_t;

typedef enum
{
	BAUDRATE_500KHZ,
	BAUDRATE_250KHZ,
	BAUDRATE_125KHZ
}SPI_BAUDRATE;


/********************************************************************/

int spi_init (SPI_MemMapPtr );
void spi_putbyte(SPI_MemMapPtr ,char );
char spi_getbyte(SPI_MemMapPtr );
void spi_send_string(SPI_MemMapPtr ,char *,int );
void spi_receive_string(SPI_MemMapPtr ,char *,int );

void spi_putword(SPI_MemMapPtr spix,int16_t ch);
void spi_putlong(SPI_MemMapPtr spix,int32_t ch);
int16_t spi_getword(SPI_MemMapPtr spix);
int32_t spi_getlong(SPI_MemMapPtr spix);

/*
*******************************************************************************
* void SpiDisable( SPI_MemMapPtr base );
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* Output        : void
* Description   : Disable respective SPI module
*******************************************************************************
*/
#define SPI_DISABLE(base) \
			SPI_MCR_REG(base) |=SPI_DISABLE_MASK;\
			SPI_MCR_REG(base) &= (~SPI_MCR_MDIS_MASK)

/*
*******************************************************************************
* void SpiSetMode( SPI_MemMapPtr base, uint8_t mode)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  mode : 1=> Master
* 				  		 0=> Slave
* Output        : void
* Description   : Enable Master or Slave mode for respective SPI module
*******************************************************************************
*/
#define SPI_SET_MODE( base , mode)\				
			(SPI_MASTER_MODE == mode) ? (SPI_MCR_REG(base) |= (SPI_MASTER_MODE <<SPI_MCR_MSTR_SHIFT )) : (SPI_MCR_REG(base) |= (SPI_SLAVE_MODE <<SPI_MCR_MSTR_SHIFT ))
	
/*
*******************************************************************************
* void SpiEnableReceiver( SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* Output        : void
* Description   : Enable Receiver for respective SPI module
*******************************************************************************
*/
#define SPI_EN_RX(base)			SPI_MCR_REG(base) |= (SPI_RECV_ENABLE <<SPI_MCR_ROOE_SHIFT )

/*
*******************************************************************************
* void SpiSetRxFifo( SPI_MemMapPtr base, uint8_t rx_fifo_enable)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  rx_fifo_enable: 1=> FIFO disabled
* 				  				  0=> FIFO Enabled
* Output        : void
* Description   : Enable or  disable Rx Fifo for respective SPI module
*******************************************************************************
*/
#define SPI_SET_Rx_FIFO(base, rx_fifo_enable)\
			(SPI_DISABLE_RXFIFO == rx_fifo_enable) ? (SPI_MCR_REG(base) |=(SPI_DISABLE_RXFIFO <<SPI_MCR_DIS_RXF_SHIFT)) : (SPI_MCR_REG(base) |=(SPI_ENABLE_RXFIFO <<SPI_MCR_DIS_RXF_SHIFT))

/*
*******************************************************************************
* void SpiSetTxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  tx_fifo_enable: 1=> FIFO disabled
* 				  				  0=> FIFO Enabled
* Output        : void
* Description   : Enable  or disable Tx Fifo for respective SPI module
*******************************************************************************
*/
#define SPI_SET_Tx_FIFO(base, tx_fifo_enable)\
		(SPI_DISABLE_RXFIFO == tx_fifo_enable)? (SPI_MCR_REG(base) |=(SPI_DISABLE_TXFIFO << SPI_MCR_DIS_TXF_SHIFT)) : (SPI_MCR_REG(base) |=(SPI_ENABLE_TXFIFO << SPI_MCR_DIS_TXF_SHIFT))

/*
*******************************************************************************
* void SpiClrFifoCounter( SPI_MemMapPtr base)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* Output        : void
* Description   : Clear RX and TX FIFO counters for respective SPI module
*******************************************************************************
*/
#define SPI_CLR_FIFO_CNT(base)\
		SPI_MCR_REG(base) |=(SPI_CLR_TXFIFO<<SPI_MCR_CLR_TXF_SHIFT);\
		SPI_MCR_REG(base) |=(SPI_CLR_RXFIFO<<SPI_MCR_CLR_RXF_SHIFT)

/*
*******************************************************************************
* void SpiSetPCS( SPI_MemMapPtr base , uint8_t peri_chip_sel)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
*				  peri_chip_sel: 0=> Low
*				  				 1=> High
* Output        : void
* Description   : Set Peripheral Chip Select for respective SPI module
*******************************************************************************
*/
#define SPI_SET_PCS(base, peri_chip_sel)				SPI_MCR_REG(base) |=SPI_MCR_PCSIS(peri_chip_sel);

/*
*******************************************************************************
* void SpiSetFrameSize( SPI_MemMapPtr base , uint8_t frame_size)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  frame_size : Number of bits transmitted -1
* Output        : void
* Description   : Set Frame Size for SPI
*******************************************************************************
*/
#define SPI_SET_FRAMESZ(base, frame_size)			SPI_CTAR_REG(base,0)= SPI_CTAR_FMSZ(frame_size)

/*
*******************************************************************************
* void SpiSetClockPhase( SPI_MemMapPtr base , uint8_t clock_phase )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  clock_phase : 0=>Data is captured on the leading edge of SCK
* 				                  and changed on the following edge.
								1=>Data is changed on the leading edge of SCK
								 and captured on the following edge.
* Output        : void
* Description   : Set Clock Phase for SPI
*******************************************************************************
*/
#define SPI_SET_CPHA(base, clock_phase)				SPI_CTAR_REG(base,0) |= (clock_phase <<SPI_CTAR_CPHA_SHIFT )

/*
*******************************************************************************
* void SpiSetClockPolarity( SPI_MemMapPtr base , uint8_t clock_polarity )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  clock_polarity: 0=> The inactive state value of SCK is low.
								  1=> The inactive state value of SCK is high.
* Output        : void
* Description   : Set Clock Polarity for SPI
*******************************************************************************
*/
#define SPI_SET_CPOL(base, clock_polarity)			SPI_CTAR_REG(base,0) |= (clock_polarity <<SPI_CTAR_CPOL_SHIFT )

/*
*******************************************************************************
* void SpiSetShiftPriority( SPI_MemMapPtr base , uint8_t shift_priority)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  shift_priority: 0=> Data is transferred MSB first.
								  1=>Data is transferred LSB first.
* Output        : void
* Description   : Set Shift priority for SPI
*******************************************************************************
*/
#define SPI_SET_SHIFT_PRIO(base, shift_priority)	SPI_CTAR_REG(base,0) |= (shift_priority<<SPI_CTAR_LSBFE_SHIFT )

/*
*******************************************************************************
* void SpiSetDelayAfterTX( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the scaler value for the After Transfer Delay.
* Output        : void
* Description   : Set After Transfer Delay for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_TX(base, delay)			SPI_CTAR_REG(base,0) |= SPI_CTAR_DT(delay)

/*
*******************************************************************************
* void SpiSetDelayAfterSCK( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the scaler value for the After SCK Delay.
* Output        : void
* Description   : Set After SCK Delay for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_SCK(base, delay)			SPI_CTAR_REG(base,0) |= SPI_CTAR_ASC(delay)

/*
*******************************************************************************
* void SpiSetDelayAfterCS( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the scaler value for the Delay after CS.
* Output        : void
* Description   : Set After CS Delay for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_CS(base, delay)			SPI_CTAR_REG(base,0) |= SPI_CTAR_CSSCK(delay)

/*
*******************************************************************************
* void SpiSetDelayAfterPCSSCK( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the prescaler value for the delay between
* 				  		  assertion of PCS and the first edge of the SCK.
* Output        : void
* Description   : Set After PCS and SCK Delay for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_PCSSCK(base, delay)		SPI_CTAR_REG(base,0) |= SPI_CTAR_PCSSCK(delay)

/*
*******************************************************************************
* void SpiSetDelayAfterPASC( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the prescaler value for the delay between
* 				  		  the last edge of SCK and the negation of PCS.
* Output        : void
* Description   : Set After SCK and PCS Delay for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_PASC(base, delay)			SPI_CTAR_REG(base,0) |= SPI_CTAR_PASC(delay)

/*
*******************************************************************************
* void SpiSetDelayAfterPDT( SPI_MemMapPtr base , uint8_t delay)
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* 				  delay : Selects the Delay after Transfer Prescaler value
* Output        : void
* Description   : Set delay after transfer prescaler value for SPI
*******************************************************************************
*/
#define SPI_SET_DLY_AFTER_PDT(base, delay)			SPI_CTAR_REG(base,0) |= SPI_CTAR_PDT(delay)

/*
*******************************************************************************
* void SpiEnable( SPI_MemMapPtr base )
*******************************************************************************
* Input         : base : Pointer to SPI0/ SPI1 /SPI2 Register Base
* Output        : void
* Description   : Enable The SPI module
*******************************************************************************
*/
#define SPI_ENABLE(base)							SPI_MCR_REG(base) &= SPI_ENABLE_MASK


/**************************************************************************************
 * 
 * @brief Enables or Disables the device.
 * @param PeripheralBase Pointer to a peripheral registers structure 
 * @param State Requested state of device. 
 * @remarks The macro accesses the following registers: SPI0_MCR, SPI1_MCR,
 *          SPI2_MCR, SPI0_C1, SPI1_C1 (depending on the peripheral).
 */
  #define 	SPI_Set_Device(PeripheralBase, State) ( \
				((State) == DISABLE) ? (SPI_MCR_REG(PeripheralBase) |= SPI_MCR_MDIS_MASK) : \
    		  	  	  	  ( SPI_MCR_REG(PeripheralBase) &= (uint32_t)(~(uint32_t)SPI_MCR_MDIS_MASK)) \
    			)
/*************************************************************************************
 * 
 * @brief Enables or Disables halt mode.
 * @param PeripheralBase Pointer to a peripheral registers structure 
 * @param State Requested state of halt mode.
 * @remarks The macro accesses the following registers: SPI0_MCR, SPI1_MCR,
 *          SPI2_MCR (depending on the peripheral).
 */
#define 	SPI_HaltMode(PeripheralBase, State) (\
				SPI_MCR_REG(PeripheralBase) = (uint32_t)( ((uint32_t)(SPI_MCR_REG(PeripheralBase) & \
					(uint32_t)(~(uint32_t)SPI_MCR_HALT_MASK))) | ((uint32_t)(State)) ) \
			)

/***************************************************************************************
 * 
 * @brief Clears Tx FIFO.
 * @param PeripheralBase Pointer to a peripheral registers structure
 * @remarks The macro accesses the following registers: SPI0_MCR, SPI1_MCR,
 *          SPI2_MCR (depending on the peripheral).
 */
#define 	SPI_ClearTxFIFO(PeripheralBase) (\
				SPI_MCR_REG(PeripheralBase) |=  SPI_MCR_CLR_TXF_MASK \
			)

/******************************************************************************************
 * 
 * @brief Clears Rx FIFO.
 * @param PeripheralBase Pointer to a peripheral registers structure 
 * @remarks The macro accesses the following registers: SPI0_MCR, SPI1_MCR,
 *          SPI2_MCR (depending on the peripheral).
 */
#define 	SPI_ClearRxFIFO(PeripheralBase) ( \
				SPI_MCR_REG(PeripheralBase) |= SPI_MCR_CLR_RXF_MASK \
			)
/*********************************************************************************************
 * 
 * @brief Returns the content of the Pop Rx FIFO register.
 * @param PeripheralBase Pointer to a peripheral registers structure 
 * @remarks The macro accesses the following registers: SPI0_POPR, SPI1_POPR,
 *          SPI2_POPR (depending on the peripheral).
 */
#define		SPI_Read_RxFIFOReg(PeripheralBase) ( \
				SPI_POPR_REG(PeripheralBase) \
			)

/**********************************************************************************************
 * 
 * @brief Writes value intended for master mode to the Push TX FIFO register.
 * @param PeripheralBase Pointer to a peripheral registers structure (
 * @param Value Value stored to the master push Tx FIFO register. This parameter
 *        is a 32-bit value.
 * @remarks The macro accesses the following registers: SPI0_PUSHR, SPI1_PUSHR,
 *          SPI2_PUSHR (depending on the peripheral).
 */
#define 	SPI_Write_TxFIFOReg(PeripheralBase, Value) ( \
				SPI_PUSHR_REG(PeripheralBase) = (uint32_t)(Value)\
	  	  	)

/**********************************************************************
  *
  * @brief  Get SPI status registers
  * @param  S_reg		: Status register of SPIx.
  * @param  Flag_mask	: SPIx Status register flag bit mask. 
  * @retval 	SET or RESET. 

*/
#define 	SPI_GET_STATUS(S_reg,Flag_mask) (\
				(!(S_reg & Flag_mask))?RESET:SET \
			)

/*
*******************************************************************************
*                         GLOBAL FUNCTION DECLARATION
*******************************************************************************
*/
 void SpiClockEnable(SPI_MemMapPtr base);
 void SpiNVICEnable(SPI_MemMapPtr base);
 void SpiSetMode( SPI_MemMapPtr base , uint8_t mode);
 void SpiEnableReceiver(SPI_MemMapPtr base);
 void SpiSetRxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable);
 void SpiSetTxFifo( SPI_MemMapPtr base, uint8_t tx_fifo_enable);
 void SpiClrFifoCounter( SPI_MemMapPtr base);
 void SpiSetPCS( SPI_MemMapPtr base, uint8_t peri_chip_sel);
 void SpiDisable( SPI_MemMapPtr base);
 void SpiSetBaudRate(SPI_MemMapPtr base, uint8_t baudrate);
 void SpiSetClockPhase( SPI_MemMapPtr base , uint8_t clock_phase);
 void SpiSetClockPolarity( SPI_MemMapPtr base , uint8_t clock_polarity);
 void SpiSetShiftPriority( SPI_MemMapPtr base , uint8_t shift_priority);
 void SpiSetFrameSize( SPI_MemMapPtr base , uint8_t frame_size);
 void SpiSetDelayAfterTX( SPI_MemMapPtr base , uint8_t delay);
 void SpiSetDelayAfterSCK( SPI_MemMapPtr base , uint8_t delay);
 void SpiSetDelayAfterCS( SPI_MemMapPtr base , uint8_t delay);
 void SpiSetDelayAfterPCSSCK( SPI_MemMapPtr base , uint8_t delay);
 void SpiSetDelayAfterPASC( SPI_MemMapPtr base , uint8_t delay);
 void SpiSetDelayAfterPDT( SPI_MemMapPtr base , uint8_t delay);
 void SpiEnable( SPI_MemMapPtr base );

 void SpiEnableTxInterrupt( SPI_MemMapPtr base );
 void SpiDisableTxInterrupt( SPI_MemMapPtr base );
 void SpiEnableRxInterrupt( SPI_MemMapPtr base );

 uint8_t SpiMasterTxRxByte( SPI_MemMapPtr base, uint8_t tx_byte);








#endif /* SPI_H_ */
