/*
 * I2C.c
 *
 *  Created on: Apr 27, 2019
 *      Author: SYS5
 */

#include "SD_I2C.h"

/*******************************************************************/
/*!
 * I2C Initialization
 * Set Baud Rate and turn on I2C0
 */


void I2C_init(I2C_MemMapPtr I2C_port, uint8_t flg)
{
	if (I2C_port == I2C0_BASE_PTR) 
	{
		SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;	//Turn on clock to I2C0 module

	    /* configure GPIO for I2C0 function */	
		PORTD_PCR9 |= PORT_PCR_MUX(2);	
		PORTD_PCR8 |= PORT_PCR_MUX(2);	
	}
	else if (I2C_port == I2C1_BASE_PTR) 
	{
		SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;	//Turn on clock to I2C1 module

	    /* configure GPIO for I2C1 function */	
		PORTC_PCR10 |= PORT_PCR_MUX(2);
		PORTC_PCR11 |= PORT_PCR_MUX(2);		
	}
	else{}
	I2C_F_REG(I2C_port) |= (
							I2C_F_MULT(0x0) |
							I2C_F_ICR(0x11)			
							/* 
							* ICR (hex) = 00, SCL Divider = 20, SDA Hold Value = 7, SCL Hold (Start) Value = 6, SCL Hold (Start) Value = 11
							* I2C BAUD RATE = 625 x 10^3
							*/
							);
	I2C_C1_REG(I2C_port) |= (I2C_C1_IICEN_MASK /*| I2C_C1_IICIE_MASK*/) ;
	if(flg & 01)
		I2C_S_REG(I2C_port) |= I2C_S_IICIF_MASK;
	else
		I2C_S_REG(I2C_port) &= ~I2C_S_IICIF_MASK;
		
}
/*******************************************************************/
/*!
 * Start I2C Transmision
 * @param SlaveID is the 7 bit Slave Address
 * @param Mode sets Read or Write Mode
 */
void I2C_Tx_init(I2C_MemMapPtr I2C_module, unsigned char mode)
{
	unsigned char Tx_M_or_S;
	unsigned char I2C_slave_ID;
	if (mode == MWSR) 
	{
	    /* set transmission mode */
		Tx_M_or_S = MWSR;
	}
	else 
	{
	    /* set transmission mode */
		Tx_M_or_S = MRSW;
	}

	  /* shift ID in right possition */
	I2C_slave_ID = (uint8_t)(MMA7660_I2C_ADDRESS << 1);

	  /* Set R/W bit at end of Slave Address */
	I2C_slave_ID |= Tx_M_or_S;

	  /* send start signal */
	I2C_C1_REG(I2C_module) |= 	I2C_C1_TX_MASK;		/* 0 Receive, 1 Transmit	 */
								

	I2C_C1_REG(I2C_module) |= 	I2C_C1_MST_MASK;		/* 0 Slave mode, 1 Master mode*/
	  /* send ID with W/R bit */
	  /**
	   * I2C_D_REG(I2C_module) -> 8 bits -> |Address of slave|Master/Slave|
	   * 										7-1				0
	  */
	I2C_D_REG(I2C_module) = I2C_slave_ID;
}

/*******************************************************************/
/*!
 * Read a register from the MPR084
 * @param u8RegisterAddress is Register Address
 * @return Data stored in Register
 */
uint8_t u8MMA7660ReadRegister(I2C_MemMapPtr I2C_module, uint8_t u8RegisterAddress)
{
	unsigned char read_val;
	unsigned int j;
	
	I2C_Tx_init(I2C_module, MWSR);
	/**
	 * Wait for interrupt to occur
	 */
	while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0);		I2C0_S |= I2C_S_IICIF_MASK;
	//I2C_S_REG(I2C_module) |= I2C_S_IICIF_MASK;
	
	I2C_D_REG(I2C_module) = u8RegisterAddress;/*WRITE THE DATA FROM ACCELEROMETER TO CONTROLLER*/
	/**
	 * Wait for interrupt to occur
	 */
	while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0){}	I2C0_S |= I2C_S_IICIF_MASK;

	  /* Do a repeated start */
	  I2C_C1_REG(I2C_module) |= I2C_C1_RSTA_MASK;

	  /* Send Slave Address */
	  I2C_D_REG(I2C_module) = (MMA7660_I2C_ADDRESS << 1) | 0x01;
	  while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0){}  I2C0_S |= I2C_S_IICIF_MASK;


	  /* Put in Rx Mode */
	  I2C_C1_REG(I2C_module) &= (~I2C_C1_TX_MASK);

	  /* Turn off ACK */
	  I2C_C1_REG(I2C_module) |= I2C_C1_TXAK_MASK;

	  /* Dummy read */
	  read_val = I2C_D_REG(I2C_module);
	  for (j=0; j<5000; j++){};
	  while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0){}	I2C0_S |= I2C_S_IICIF_MASK;

	  /* Send stop */
	  I2C_C1_REG(I2C_module) &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
	  
	  read_val = I2C_D_REG(I2C_module);
	  msDelay(5);
	  return read_val;
}

/*******************************************************************/
/*!
 * Write a byte of Data to specified register on MPR084
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void MMA7660WriteRegister(I2C_MemMapPtr I2C_module, uint8_t u8RegisterAddress, uint8_t u8Data)
{
	/* send data to slave */
	I2C_Tx_init(I2C_module, MWSR);
	while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0);
	I2C0_S |= I2C_S_IICIF_MASK;
	//Pause();
	
	I2C_D_REG(I2C_module) = u8RegisterAddress;
	//while((I2C_S_REG(I2C_module) & I2C_S_TCF_MASK) == 0);
	while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0);
	I2C0_S |= I2C_S_IICIF_MASK;
	//Pause();
	
	I2C_D_REG(I2C_module) = u8Data;
	//while((I2C_S_REG(I2C_module) & I2C_S_TCF_MASK) == 0);
	while((I2C_S_REG(I2C_module) & I2C_S_IICIF_MASK) == 0);
	I2C0_S |= I2C_S_IICIF_MASK;

	/* Send stop */
	I2C_C1_REG(I2C_module) &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
	  msDelay(5);
}

/*******************************************************************/
/*!
 * Pause Routine
 */
void Pause(void){
    int n;
	while(n++ < 50);
}

/*void I2C_Handler()
{
	
}
*/
int8_t I2C_Ack(I2C_MemMapPtr I2Cx, int tx_rx)
{
	if (tx_rx == TX)
		return (I2C_S_REG(I2Cx) & I2C_S_TCF_MASK);
	else if (tx_rx = RX) 
		return (I2C_S_REG(I2Cx) & I2C_S_RXAK_MASK);
	else
		return -1;
}
/*void I2C0_IRQHandler()
{
	*/