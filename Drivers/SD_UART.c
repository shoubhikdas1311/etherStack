/*
 * uart.c
 *
 *  Created on: Apr 18, 2019
 *      Author: SYS6_NET
 */
#include "SD_UART.h"
//#define BAUD 115200

void uart5_init(UART_MemMapPtr uart,int sys_clk_mhz, int baud)
{
	register uint16_t sbr, brfa;
	int peripheral_clk_mhz, peripheral_clk_khz;
	uint8_t temp;

	if(uart == UART0_BASE_PTR || uart == UART1_BASE_PTR)
		peripheral_clk_mhz = sys_clk_mhz;
	else
		peripheral_clk_mhz = sys_clk_mhz / (((SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> 24)+ 1);
	
	peripheral_clk_khz = peripheral_clk_mhz*1000;
	//uint16_t  = 110;
/**
 * ENABLE CLK TO UART0 AND UART1
 */
	/* Enable the clock to the selected UART */   
    	SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
	/**
	 * SET "BRFA" FOR UART0 AND UART1 BOTH OF THEM USES EITHER SYS/CORE CLK
	 */
	UART_C2_REG(uart) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
	UART_C1_REG(uart) = 0X00;
	UART_C4_REG(uart) |= UART_C4_BRFA(0X0);
	/**
	 * CALCULATE "SBR" FROM GIVEN BAUDRATE FOR UART0 AND UART1
	 */
	/**
	 * UART0
	 */
	sbr = (uint16_t)((peripheral_clk_khz * 1000)/(16 * baud));
	temp = (UART5_BDH & ~UART_BDH_SBR_MASK);
	//error in baud rate
	//brfa = (uint8_t)(((peripheral_clk_mhz * 1000 * 32)/(16 * baud)) - (sbr * 32));
	UART_BDH_REG(uart) = temp | ((sbr & 0X1F00) >> 8);
	UART_BDL_REG(uart) = UART_BDL_SBR(sbr);
	//UART5_BDL |= UART_BDL_SBR(sbr & 0X00FF);
	/**
	 * UART0 AND 1 READY FOR Tx AND Rx
	 */
	brfa = (((peripheral_clk_khz*32000)/(baud * 16))-(sbr * 32));
	temp = (UART_C4_REG(uart) & ~UART_C4_BRFA_MASK);
	UART_C4_REG(uart) = (temp | UART_C4_BRFA(brfa));
	UART_C2_REG(uart) |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
	//uart_port_init(uart);
	
}
void uart_putChar(UART_MemMapPtr RxUart, char ch)
{
	while(!(UART_S1_REG(RxUart) & UART_S1_TDRE_MASK));

    UART_D_REG(RxUart) = (uint8_t)ch;
}
void uart_port_init(UART_MemMapPtr UARTx)
{	
	switch((uint32_t)UARTx)
	{
		case ((uint32_t)UART0_BASE_PTR):
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			
			PORTA_PCR2|=PORT_PCR_MUX(0x02);
			PORTA_PCR1|=PORT_PCR_MUX(0x02);	
			break;
		case ((uint32_t)UART1_BASE_PTR):
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
			PORTE_PCR1|=PORT_PCR_MUX(0x02);
			PORTE_PCR0|=PORT_PCR_MUX(0x02);
			break;
		case ((uint32_t)UART2_BASE_PTR):
			SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
						
			PORTD_PCR3|=PORT_PCR_MUX(0x03);
			PORTD_PCR2|=PORT_PCR_MUX(0x03);
			break;
		case ((uint32_t)UART3_BASE_PTR):
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
					
			PORTC_PCR17|=PORT_PCR_MUX(0x03);
			PORTC_PCR16|=PORT_PCR_MUX(0x03);
			break;
		case ((uint32_t)UART4_BASE_PTR):
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
						
			PORTC_PCR15|=PORT_PCR_MUX(0x03);
			PORTC_PCR14|=PORT_PCR_MUX(0x03);
			break;
		case ((uint32_t)UART5_BASE_PTR):
			SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;	
		
			PORTE_PCR8 |= PORT_PCR_MUX(0x3); 
			PORTE_PCR9 |= PORT_PCR_MUX(0x3); 
			break;
	}
	/*if(UART==UART0_BASE_PTR)
	{
		SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
		
		PORTA_PCR2|=PORT_PCR_MUX(0x02);
		PORTA_PCR1|=PORT_PCR_MUX(0x02);
		
	}
	else if(UART==UART1_BASE_PTR)
	{
		SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
		PORTE_PCR1|=PORT_PCR_MUX(0x02);
		PORTE_PCR0|=PORT_PCR_MUX(0x02);
	
	}
	else if(UART==UART2_BASE_PTR)
	{
		SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
						
		PORTD_PCR3|=PORT_PCR_MUX(0x03);
		PORTD_PCR2|=PORT_PCR_MUX(0x03);
				
	}
	else if(UART==UART3_BASE_PTR)
	{
		SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
					
		PORTC_PCR17|=PORT_PCR_MUX(0x03);
		PORTC_PCR16|=PORT_PCR_MUX(0x03);
					
	}
	else if(UART==UART4_BASE_PTR)
	{
		SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
						
		PORTC_PCR15|=PORT_PCR_MUX(0x03);
		PORTC_PCR14|=PORT_PCR_MUX(0x03);
						
	}
	else if(UART==UART5_BASE_PTR)
	{
		SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;	
		
		PORTE_PCR8 |= PORT_PCR_MUX(0x3); 
		PORTE_PCR9 |= PORT_PCR_MUX(0x3); 
	}*/
}
int SDprint(char *str)
{
	int i = 0;
	while(*(str + i))
	{
		/*if(*(str + i))
		{
			uart_putChar(UART5_BASE_PTR,*(str + i++));
			uart_putChar(UART5_BASE_PTR,'\r');
		}*/
		uart_putChar(UART5_BASE_PTR,*(str + i++));					
	}
	return i;
}
