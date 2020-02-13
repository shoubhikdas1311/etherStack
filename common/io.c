/*
 * File:		io.c
 * Purpose:		Serial Input/Output routines
 *
 * Notes:       TERMINAL_PORT defined in <board>.h
 */

#include "common.h"


/********************************************************************/
char
in_char (void)
{
	while (!(UART_S1_REG(JTAG_UART) & UART_S1_RDRF_MASK));
    return UART_D_REG(JTAG_UART);
}
/********************************************************************/
void
out_char (char ch)
{
	while(!(UART_S1_REG(JTAG_UART) & UART_S1_TDRE_MASK));
    UART_D_REG(JTAG_UART) = (uint32_t)ch;
}
/********************************************************************/
int
char_present (void)
{
	return (UART_S1_REG(JTAG_UART) & UART_S1_RDRF_MASK);
}
/********************************************************************/
void JTAG_UART_Init(int sysclk, int baud){
	
	register uint16_t sbr, brfa;
	uint8_t temp;
/*
	PORTC_PCR3 = PORT_PCR_MUX(0x3); 
	PORTC_PCR4 = PORT_PCR_MUX(0x3); 
	
	SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
	*/
/*	PORTD_PCR2 = PORT_PCR_MUX(0x3); 
	PORTD_PCR3 = PORT_PCR_MUX(0x3); 
	
	SIM_SCGC4 |= SIM_SCGC4_UART2_MASK; 
	
	PORTA_PCR14 = PORT_PCR_MUX(3); 			//UART0 for RS422 to RS232 uart interface for az/el_card
	PORTA_PCR15 = PORT_PCR_MUX(3); 
	
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;*/
	SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
	
	PORTE_PCR8 |= PORT_PCR_MUX(0x3); 
	PORTE_PCR9 |= PORT_PCR_MUX(0x3); 
	
		
	/* Make sure that the transmitter and receiver are disabled while we 
	     * change settings.
	     */
	UART_C2_REG(JTAG_UART) &= ~(UART_C2_TE_MASK| UART_C2_RE_MASK );

	/* Configure the UART for 8-bit mode, no parity */
	UART_C1_REG(JTAG_UART) = 0;	/* We need all default settings, so entire register is cleared */
	    
	/* Calculate baud settings */
	sbr = (uint16_t)((sysclk*1000)/(baud * 16));
	        
	/* Save off the current value of the UARTx_BDH except for the SBR field */
	temp = UART_BDH_REG(JTAG_UART) & ~(UART_BDH_SBR(0x1F));
	    
	UART_BDH_REG(JTAG_UART) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
	UART_BDL_REG(JTAG_UART) = (uint8_t)(sbr & UART_BDL_SBR_MASK);
	    
	/* Determine if a fractional divider is needed to get closer to the baud rate */
	brfa = (((sysclk*32000)/(baud * 16)) - (sbr * 32));
	    
	/* Save off the current value of the UARTx_C4 register except for the BRFA field */
	temp = UART_C4_REG(JTAG_UART) & ~(UART_C4_BRFA(0x1F));
	    
	UART_C4_REG(JTAG_UART) = temp |  UART_C4_BRFA(brfa);    

	/* Enable receiver and transmitter */
	UART_C2_REG(JTAG_UART) |= (UART_C2_TE_MASK	| UART_C2_RE_MASK );
}
