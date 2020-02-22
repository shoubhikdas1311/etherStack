#include "SD_ETH.h"
#define NBUF_LITTLE_ENDIAN						1
//!< ENET DMA Tx&Rx Descriptors memory regin(must be 16 bit agiged)
extern uint16_t *Tx_buff;
extern uint32_t packetCount;

static  uint8_t xENETTxDescriptors_unaligned[(1*sizeof(NBUF))+16];
static  uint8_t xENETRxDescriptors_unaligned[(1*sizeof(NBUF))+16];
static NBUF *pxENETTxDescriptor;
static NBUF *pxENETRxDescriptors;
//!< enet received memory pool
static uint8_t ucENETRxBuffers[ ( 1 * 1518 )+ 16], ucENETTxBuffers[ ( 1 * 1518 )+ 16];

ENET_PHY_t eth_PHY;

/* Next BD indicies for BD ring */ 
static int next_rxbd;
static int next_txbd;

static int enetWritePacket(ENET_MemMapPtr enet, uint8_t* buffer, unsigned len);
static uint8_t ENET_HashAddress(const uint8_t* addr)
{
    uint32_t crc;
    uint8_t byte;
    int i, j;
    crc = 0xFFFFFFFF;
    for(i=0; i < 6; ++i)
    {
        byte = addr[i];
        for(j = 0; j < 8; ++j)
        {
            if((byte & 0x01)^(crc & 0x01))
            {
                crc >>= 1;
                crc = crc ^ 0xEDB88320;
            }
        else
            crc >>= 1;
        byte >>= 1;
    }
    }
    return (uint8_t)(crc >> 26);
}

/**
 * @brief   ����ENETģ��Ľ���MAC��ַ
 * @param   MAC��ַ
 * @retval  None
 */
static void ENET_SetAddress(uint8_t *pa)
{
    uint8_t crc;
    ENET_PALR = (uint32_t)((pa[0]<<24) | (pa[1]<<16) | (pa[2]<<8) | pa[3]);
    ENET_PAUR = (uint32_t)((pa[4]<<24) | (pa[5]<<16) | (0x88))| ENET_PAUR_TYPE(0x0800);
    crc = ENET_HashAddress(pa);
    if(crc >= 32)
    ENET_IAUR |= (uint32_t)(1 << (crc - 32));
    else
    ENET_IALR |= (uint32_t)(1 << crc);
}
void ETH_init(int sysClk, uint8_t *ucMACAddress)
{
	uint16_t clk_mhz;
	int usData;//, ;
	/* Enable the ENET clock. */
	SIM_SCGC2 |= SIM_SCGC2_ENET_MASK;
	SIM_SOPT2 = SIM_SOPT2_TIMESRC(0x2);
	/*FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error*/
	MPU_CESR = 0;
	BDinit();    /* Set the Reset bit and clear the Enable bit */
	ENET_ECR = ENET_ECR_RESET_MASK;
	/* Wait at least 8 clock cycles */
	for( usData = 0; usData < 10; usData++ )
	{
//		asm( "NOP" );
	}
	/*FSL: start MII interface*/
	clk_mhz = 100;
	mii_init(0, clk_mhz/*MHz*/);
	//enet_interrupt_routine
	set_irq_priority (76, 6);
	enable_irq(76);//ENET xmit interrupt  //enet_interrupt_routine
	set_irq_priority (77, 6);
	enable_irq(77);//ENET rx interrupt  //enet_interrupt_routine
	set_irq_priority (78, 6);
	enable_irq(78);//ENET error and misc interrupts
	/*   * Make sure the external interface signals are enabled   */
	PORTB_PCR0  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDIO/MII0_MDIO
	PORTB_PCR1  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDC/MII0_MDC
	#if configUSE_MII_MODE
	PORTA_PCR14 = PORT_PCR_MUX(4);//RMII0_CRS_DV/MII0_RXDV
	PORTA_PCR5  = PORT_PCR_MUX(4);//RMII0_RXER/MII0_RXER
	PORTA_PCR12 = PORT_PCR_MUX(4);//RMII0_RXD1/MII0_RXD1
	PORTA_PCR13 = PORT_PCR_MUX(4);//RMII0_RXD0/MII0_RXD0
	PORTA_PCR15 = PORT_PCR_MUX(4);//RMII0_TXEN/MII0_TXEN
	PORTA_PCR16 = PORT_PCR_MUX(4);//RMII0_TXD0/MII0_TXD0
	PORTA_PCR17 = PORT_PCR_MUX(4);//RMII0_TXD1/MII0_TXD1
	
	PORTA_PCR11 = PORT_PCR_MUX(4);//MII0_RXCLK
	PORTA_PCR25 = PORT_PCR_MUX(4);//MII0_TXCLK
	PORTA_PCR9  = PORT_PCR_MUX(4);//MII0_RXD3
	PORTA_PCR10 = PORT_PCR_MUX(4);//MII0_RXD2
	PORTA_PCR28 = PORT_PCR_MUX(4);//MII0_TXER
	PORTA_PCR24 = PORT_PCR_MUX(4);//MII0_TXD2
	PORTA_PCR26 = PORT_PCR_MUX(4);//MII0_TXD3
	PORTA_PCR27 = PORT_PCR_MUX(4);//MII0_CRS
	PORTA_PCR29 = PORT_PCR_MUX(4);//MII0_COL
	#else
	PORTA_PCR14 = PORT_PCR_MUX(4);//RMII0_CRS_DV/MII0_RXDV
	PORTA_PCR5  = PORT_PCR_MUX(4);//RMII0_RXER/MII0_RXER
	PORTA_PCR12 = PORT_PCR_MUX(4);//RMII0_RXD1/MII0_RXD1
	PORTA_PCR13 = PORT_PCR_MUX(4);//RMII0_RXD0/MII0_RXD0
	PORTA_PCR15 = PORT_PCR_MUX(4);//RMII0_TXEN/MII0_TXEN
	PORTA_PCR16 = PORT_PCR_MUX(4);//RMII0_TXD0/MII0_TXD0
	PORTA_PCR17 = PORT_PCR_MUX(4);//RMII0_TXD1/MII0_TXD1
	#endif
	/* Can we talk to the PHY? */
	do
	{
		msDelay(10);
		usData = 0xffff;
		mii_read(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
	}
	while( usData == 0xffff );
	/* Start auto negotiate. */
	mii_write(CFG_PHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );
	/* Wait for auto negotiate to complete. */
	do
	{
		msDelay(10);
		mii_read(CFG_PHY_ADDRESS, PHY_BMSR, &usData );
	}
	while( !( usData & PHY_BMSR_AN_COMPLETE ) );
	/* When we get here we have a link - find out what has been negotiated. */
	usData = 0;
	mii_read(CFG_PHY_ADDRESS, PHY_STATUS, &usData );
	mii_read(CFG_PHY_RMII, BC_REG, &usData );
    printf("BC_REG:0x%X\r\n",usData);
	eth_PHY.bc_reg = usData;
	mii_read(CFG_PHY_RMII, BS_REG, &usData );
    printf("BS_REG:0x%X\r\n",usData);
	eth_PHY.bs_reg = usData;
	mii_read(CFG_PHY_RMII, PHY_ID1_REG, &usData );
    printf("PHY_ID1_REG:0x%X\r\n",usData);
	eth_PHY.phy_id1_reg = usData;
	mii_read(CFG_PHY_RMII, PHY_ID2_REG, &usData );
    printf("PHY_ID2_REG:0x%X\r\n",usData);
	eth_PHY.phy_id2_reg = usData;
	mii_read(CFG_PHY_RMII, A_NEG_ADVT_REG, &usData );
    printf("A_NEG_ADVT_REG:0x%X\r\n",usData);
	eth_PHY.a_neg_advt_reg = usData;
	mii_read(CFG_PHY_RMII, A_NEG_ADVT_PARTNER_REG, &usData );
    printf("A_NEG_ADVT_PARTNER_REG:0x%X\r\n",usData);
	eth_PHY.a_neg_advt_partner_reg = usData;
	mii_read(CFG_PHY_RMII, A_NEG_EXP_REG, &usData );
    printf("A_NEG_EXP_REG:0x%X\r\n",usData);
	eth_PHY.a_neg_exp_reg = usData;
	mii_read(CFG_PHY_RMII, A_NEG_NXT_PG_REG, &usData );
    printf("A_NEG_NXT_PG_REG:0x%X\r\n",usData);
	eth_PHY.a_neg_nxt_pg_reg = usData;
	mii_read(CFG_PHY_RMII, A_NEG_NXT_PG_PARTNER_REG, &usData );
    printf("A_NEG_NXT_PG_PARTNER_REG:0x%X\r\n",usData);
	eth_PHY.a_neg_advt_partner_reg = usData;
	mii_read(CFG_PHY_RMII, MII_CTRL_REG, &usData );
    printf("MII_CTRL_REG:0x%X\r\n",usData);
	eth_PHY.mii_ctrl_reg = usData;
	mii_read(CFG_PHY_RMII, RXER_Counter_REG, &usData );
    printf("RXER_Counter_REG:0x%X\r\n",usData);
	eth_PHY.rxer_counter_reg = usData;
	mii_read(CFG_PHY_RMII, INT_REG, &usData );
    printf("INT_REG:0x%X\r\n",usData);
	eth_PHY.int_reg = usData;
	mii_read(CFG_PHY_RMII, PHY_CTRL1_REG, &usData );
    printf("PHY_CTRL1_REG:0x%X\r\n",usData);
	eth_PHY.phy_ctrl1_reg = usData;
	mii_read(CFG_PHY_RMII, PHY_CTRL2_REG, &usData );
    printf("PHY_CTRL2_REG:0x%X\r\n",usData);
	eth_PHY.phy_ctrl2_reg = usData;
	/* Clear the Individual and Group Address Hash registers */
	ENET_IALR = 0;
	ENET_IAUR = 0;
	ENET_GALR = 0;
	ENET_GAUR = 0;    /* Set the Physical Address for the selected ENET */
	ENET_SetAddress(ucMACAddress );
	#if configUSE_MII_MODE
	/* Various mode/status setup. */
		ENET_RCR = ENET_RCR_MAX_FL(CFG_ENET_BUFFER_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK;
	#else
		ENET_RCR = ENET_RCR_MAX_FL(CFG_ENET_BUFFER_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK | ENET_RCR_RMII_MODE_MASK;
	#endif
	/*FSL: clear rx/tx control registers*/
	ENET_TCR = 0;
	/* Setup half or full duplex. */
	if( usData & PHY_DUPLEX_STATUS )
	{
		/*Full duplex*/
		ENET_RCR &= (uint32_t)~ENET_RCR_DRT_MASK;
		ENET_TCR |= ENET_TCR_FDEN_MASK;
	}
	else
	{
		/*half duplex*/
		ENET_RCR |= ENET_RCR_DRT_MASK;
		ENET_TCR &= (uint32_t)~ENET_TCR_FDEN_MASK;
	}
	/* Setup speed */
	if( usData & PHY_SPEED_STATUS )
	{
		/*10Mbps*/
		ENET_RCR |= ENET_RCR_RMII_10T_MASK;
	}
	#if( configUSE_PROMISCUOUS_MODE == 1 )
	{
		ENET_RCR |= ENET_RCR_PROM_MASK;
	}
	#endif
	#ifdef ENHANCED_BD
	ENET_ECR = ENET_ECR_EN1588_MASK;
	#else
	ENET_ECR = 0;
	#endif
	/* Set Rx Buffer Size */
	ENET_MRBR = ENET_MRBR_R_BUF_SIZE_MASK << ENET_MRBR_R_BUF_SIZE_SHIFT;
  	/* Point to the start of the circular Rx buffer descriptor queue */
  	ENET_RDSR = (uint32_t)((uint32_t *)pxENETRxDescriptors);
  	/* Point to the start of the circular Tx buffer descriptor queue */
  	ENET_TDSR = (uint32_t)((uint32_t *)pxENETTxDescriptor);
  	/* Clear all ENET interrupt events */
	ENET_EIR |= 0xFFFFFFFF;//ENET_EIR_TXB_MASK | ENET_EIR_TXF_MASK | ENET_EIR_RXB_MASK | ENET_EIR_RXF_MASK;
  	/* Enable interrupts */
	ENET_EIMR = 0xFFFFFFFF;//ENET_EIMR_TXF_MASK | ENET_EIMR_TXB_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_RXB_MASK | ENET_EIMR_UN_MASK | ENET_EIMR_RL_MASK | ENET_EIMR_LC_MASK | ENET_EIMR_BABT_MASK | ENET_EIMR_BABR_MASK | ENET_EIMR_EBERR_MASK ;
  	/* Create the task that handles the MAC ENET RX */  /* RTOS + TCP/IP stack dependent */    /* Enable the MAC itself. */
	ENET_ECR |= ENET_ECR_ETHEREN_MASK;
  	/* Indicate that there have been empty receive buffers produced */
	ENET_RDAR = ENET_RDAR_RDAR_MASK;
} 
static void BDinit( void ) 
{
	uint8_t ux;
	uint8_t *pcBufPointer;
	/*Tx des*/
	pcBufPointer = &( xENETTxDescriptors_unaligned[ 0 ] );
	while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETTxDescriptor = ( NBUF * ) pcBufPointer;
	
	/*Rx des*/
    pcBufPointer = &( xENETRxDescriptors_unaligned[ 0 ] );
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETRxDescriptors = ( NBUF * ) pcBufPointer;
	/* Setup the buffers and descriptors. */
	pcBufPointer = &( ucENETTxBuffers[ 0 ] );
	while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	for( ux = 0; ux < CFG_NUM_ENET_TX_BUFFERS; ux++ )
	{
		pxENETTxDescriptor[ ux ].status = TX_BD_TC;
		#ifdef NBUF_LITTLE_ENDIAN
			Tx_buff = (uint16_t *)pcBufPointer;
			pxENETTxDescriptor[ ux ].data = (uint8_t *)__little2Big_end((uint32_t)pcBufPointer);
		#else
			pxENETTxDescriptor[ ux ].data = pcBufPointer;
		#endif
		pcBufPointer += CFG_ENET_BUFFER_SIZE;
		pxENETTxDescriptor[ ux ].length = 0;
		#ifdef ENHANCED_BD
			pxENETTxDescriptor[ ux ].ebd_status = TX_BD_IINS | TX_BD_PINS;
		#endif
	}
	pcBufPointer = &( ucENETRxBuffers[ 0 ] );
	while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	for( ux = 0; ux < CFG_NUM_ENET_RX_BUFFERS; ux++ )
	{
		pxENETRxDescriptors[ ux ].status = RX_BD_E;
		pxENETRxDescriptors[ ux ].length = 0;
		#ifdef NBUF_LITTLE_ENDIAN
			pxENETRxDescriptors[ ux ].data = (uint8_t *)__little2Big_end((uint32_t)pcBufPointer);
		#else
			pxENETRxDescriptors[ ux ].data = pcBufPointer;
		#endif
		pcBufPointer += CFG_NUM_ENET_RX_BUFFERS;
		#ifdef ENHANCED_BD
			pxENETRxDescriptors[ ux ].bdu = 0x00000000;
			pxENETRxDescriptors[ ux ].ebd_status = RX_BD_INT;
		#endif
	}
	/* Set the wrap bit in the last descriptors to form a ring. */
	pxENETTxDescriptor[ CFG_NUM_ENET_TX_BUFFERS - 1 ].status |= TX_BD_W;
	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
	//uxNextRxBuffer = 0;  uxNextTxBuffer = 0;
}

int __little2Big_end(int num)
{
    int retNum = 0x00, i;
    char inter = 0x00;
    for (i = 0; i < 4; i++)
    {
        inter = GETBYTE(&num, i);
        retNum |= SETBYTE(&inter, (3-i));
    }
    return retNum;
}
void enet_start_mii(void)
{
	uint16_t usData;
	PORTB_PCR0  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDIO/MII0_MDIO
	PORTB_PCR1  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDC/MII0_MDC    
	/*FSL: start MII interface*/
	
	mii_init(0, 100/*MHz*/); 
	/* Can we talk to the PHY? */
	do
	{
		msDelay(10);
		usData = 0xffff;
		mii_read(CFG_PHY_ADDRESS, PHY_PHYIDR1, (int *)&usData );
	}
	while(usData == 0xffff);
	/* Start auto negotiate. */
	mii_write(CFG_PHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );
}
void mii_init(int ch, int sys_clk_mhz)
{
	ENET_MSCR/*(ch)*/ = 0
	#ifdef TSIEVB/*TSI EVB requires a longer hold time than default 10 ns*/
	| ENET_MSCR_HOLDTIME(2)
	#endif
	| ENET_MSCR_MII_SPEED((2*sys_clk_mhz/5)+1);
}
int mii_write(int phy_addr, int reg_addr, int data)
{
	int timeout;
	/* Clear the MII interrupt bit */
	ENET_EIR/*(ch)*/ = ENET_EIR_MII_MASK;
	/* Initiatate the MII Management write */
	ENET_MMFR/*(ch)*/ = 0 | ENET_MMFR_ST(0x01) | ENET_MMFR_OP(0x01) | ENET_MMFR_PA(phy_addr) | ENET_MMFR_RA(reg_addr) | ENET_MMFR_TA(0x02) | ENET_MMFR_DATA(data);
	/* Poll for the MII interrupt (interrupt should be masked) */
	for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
	{
		if (ENET_EIR/*(ch)*/ & ENET_EIR_MII_MASK)
		break;
	}
	if(timeout == MII_TIMEOUT)
		return 1;
	/* Clear the MII interrupt bit */
	ENET_EIR/*(ch)*/ = ENET_EIR_MII_MASK;
	return 0;
}
/********************************************************************/
int mii_read(int phy_addr, int reg_addr, int *data)
{
	int timeout;
/* Clear the MII interrupt bit */
ENET_EIR/*(ch)*/ = ENET_EIR_MII_MASK;
/* Initiatate the MII Management read */
ENET_MMFR/*(ch)*/ = 0 | ENET_MMFR_ST(0x01) | ENET_MMFR_OP(0x2) | ENET_MMFR_PA(phy_addr) | ENET_MMFR_RA(reg_addr) | ENET_MMFR_TA(0x02);
/* Poll for the MII interrupt (interrupt should be masked) */
for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
{
	if (ENET_EIR/*(ch)*/ & ENET_EIR_MII_MASK)
		break;
}
if(timeout == MII_TIMEOUT)
	return 1;
/* Clear the MII interrupt bit */
ENET_EIR/*(ch)*/ = ENET_EIR_MII_MASK;
*data = ENET_MMFR/*(ch)*/ & 0x0000FFFF;
return (*data);
}
void ENET_Transmit_IRQHandler()
{
	SDprint("Inside ENET_Transmit_IRQHandler\n\r");
	if(ENET_EIR & ENET_EIMR_TXF_MASK)
	{
		SDprint("Packet transferred\n\r");
		ENET_EIR |= ENET_EIMR_TXF_MASK;
	}
	if(ENET_EIR & ENET_EIMR_TXB_MASK)
	{
		SDprint("Packet still there for tx\n\r");
		ENET_EIR |= ENET_EIMR_TXB_MASK;
	}
	if(ENET_EIR & ENET_EIMR_RXF_MASK)
	{
		SDprint("Packet received\n\r");
		ENET_EIR |= ENET_EIMR_RXF_MASK;
	}
	if(ENET_EIR & ENET_EIMR_RXB_MASK)
	{
		SDprint("Packet still there for rx\n\r");
		ENET_EIR |= ENET_EIMR_RXB_MASK;
	}
	if(ENET_EIR & ENET_EIR_GRA_MASK)
	{
		SDprint("ENET_EIR_GRA_MASK error\n\r");
		ENET_EIR |= ENET_EIR_GRA_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABR_MASK)
	{
		SDprint("ENET_EIR_BABR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABR_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABT_MASK)
	{
		SDprint("ENET_EIR_BABT_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABT_MASK;
	}
	if(ENET_EIR & ENET_EIR_EBERR_MASK)
	{
		SDprint("ENET_EIR_EBERR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_EBERR_MASK;
	}
	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
	{
		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
	}
	if(ENET_EIR & ENET_EIR_PLR_MASK)
	{
		SDprint("ENET_EIR_PLR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_PLR_MASK;
	}
	if(ENET_EIR & ENET_EIR_MII_MASK)
	{
		SDprint("ENET_EIR_MII_MASK error\n\r");
        printf(" >> %x", ENET_MMFR);
		ENET_EIR |= ENET_EIR_MII_MASK;
	}
	if(ENET_EIR & ENET_EIR_LC_MASK)
	{
		SDprint("ENET_EIR_LC_MASK error\n\r");
		ENET_EIR |= ENET_EIR_LC_MASK;
	}
	if(ENET_EIR & ENET_EIR_RL_MASK)
	{
		SDprint("ENET_EIR_RL_MASK error\n\r");
		ENET_EIR |= ENET_EIR_RL_MASK;
	}
	if(ENET_EIR & ENET_EIR_UN_MASK)
	{
		SDprint("ENET_EIR_UN_MASK error\n\r");
		ENET_EIR |= ENET_EIR_UN_MASK;
	}
		
	ENET_EIR |= ENET_EIMR_TXF_MASK;
	//ENET_EIR |= ENET_EIMR_TXB_MASK;
	pxENETTxDescriptor[ CFG_NUM_ENET_TX_BUFFERS - 1 ].status |= TX_BD_W;
	LED_GREEN_TOGGLE;
	packetCount++;
//	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
}
void ENET_Receive_IRQHandler()
{
	SDprint("Inside ENET_Receive_IRQHandler\n\r");	
	if(ENET_EIR & ENET_EIMR_TXF_MASK)
	{
		SDprint("Packet transferred\n\r");
		ENET_EIR |= ENET_EIMR_TXF_MASK;
	}
	if(ENET_EIR & ENET_EIMR_TXB_MASK)
	{
		SDprint("Packet still there for tx\n\r");
		ENET_EIR |= ENET_EIMR_TXB_MASK;
	}
	if(ENET_EIR & ENET_EIMR_RXF_MASK)
	{
		SDprint("Packet received\n\r");
		if(ENET_EIR & ENET_EIMR_RXB_MASK)
		{
			SDprint("Packet still there for rx\n\r");
			ENET_EIR |= ENET_EIMR_RXB_MASK;
		}
		else
		{
			ENET_EIR |= ENET_EIMR_RXF_MASK;
		}
	}
	if(ENET_EIR & ENET_EIR_GRA_MASK)
	{
		SDprint("ENET_EIR_GRA_MASK error\n\r");
		ENET_EIR |= ENET_EIR_GRA_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABR_MASK)
	{
		SDprint("ENET_EIR_BABR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABR_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABT_MASK)
	{
		SDprint("ENET_EIR_BABT_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABT_MASK;
	}
	if(ENET_EIR & ENET_EIR_EBERR_MASK)
	{
		SDprint("ENET_EIR_EBERR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_EBERR_MASK;
	}
	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
	{
		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
	}
	if(ENET_EIR & ENET_EIR_PLR_MASK)
	{
		SDprint("ENET_EIR_PLR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_PLR_MASK;
	}
	if(ENET_EIR & ENET_EIR_MII_MASK)
	{
		SDprint("ENET_EIR_MII_MASK error\n\r");
        printf(" >> %x", ENET_MMFR);
		ENET_EIR |= ENET_EIR_MII_MASK;
	}
	if(ENET_EIR & ENET_EIR_LC_MASK)
	{
		SDprint("ENET_EIR_LC_MASK error\n\r");
		ENET_EIR |= ENET_EIR_LC_MASK;
	}
	if(ENET_EIR & ENET_EIR_RL_MASK)
	{
		SDprint("ENET_EIR_RL_MASK error\n\r");
		ENET_EIR |= ENET_EIR_RL_MASK;
	}
	if(ENET_EIR & ENET_EIR_UN_MASK)
	{
		SDprint("ENET_EIR_UN_MASK error\n\r");
		ENET_EIR |= ENET_EIR_UN_MASK;
	}
	ENET_EIR |= ENET_EIMR_RXF_MASK;
	//ENET_EIR |= ENET_EIMR_RXB_MASK;
//	pxENETTxDescriptor[ CFG_NUM_ENET_TX_BUFFERS - 1 ].status |= TX_BD_W;
	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
}
void ENET_Error_IRQHandler()
{
	SDprint("Inside ENET_Error_IRQHandler\n\r");
	if(ENET_EIR & ENET_EIMR_TXF_MASK)
	{
		SDprint("Packet transferred\n\r");
		ENET_EIR |= ENET_EIMR_TXF_MASK;
	}
	if(ENET_EIR & ENET_EIMR_TXB_MASK)
	{
		SDprint("Packet still there for tx\n\r");
		ENET_EIR |= ENET_EIMR_TXB_MASK;
	}
	if(ENET_EIR & ENET_EIMR_RXF_MASK)
	{
		SDprint("Packet received\n\r");
		ENET_EIR |= ENET_EIMR_RXF_MASK;
	}
	if(ENET_EIR & ENET_EIMR_RXB_MASK)
	{
		SDprint("Packet still there for rx\n\r");
		ENET_EIR |= ENET_EIMR_RXB_MASK;
	}
	if(ENET_EIR & ENET_EIR_TS_AVAIL_MASK)
	{
		SDprint("ENET_EIR_TS_AVAIL_MASK error\n\r");
		ENET_EIR |= ENET_EIR_TS_AVAIL_MASK;
	}
	if(ENET_EIR & ENET_EIR_TS_TIMER_MASK)
	{
		SDprint("ENET_EIR_TS_TIMER_MASK error\n\r");
		ENET_EIR |= ENET_EIR_TS_TIMER_MASK;
	}
	if(ENET_EIR & ENET_EIR_GRA_MASK)
	{
		SDprint("ENET_EIR_GRA_MASK error\n\r");
		ENET_EIR |= ENET_EIR_GRA_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABR_MASK)
	{
		SDprint("ENET_EIR_BABR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABR_MASK;
	}
	if(ENET_EIR & ENET_EIR_BABT_MASK)
	{
		SDprint("ENET_EIR_BABT_MASK error\n\r");
		ENET_EIR |= ENET_EIR_BABT_MASK;
	}
	if(ENET_EIR & ENET_EIR_EBERR_MASK)
	{
		SDprint("ENET_EIR_EBERR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_EBERR_MASK;
	}
	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
	{
		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
	}
	if(ENET_EIR & ENET_EIR_PLR_MASK)
	{
		SDprint("ENET_EIR_PLR_MASK error\n\r");
		ENET_EIR |= ENET_EIR_PLR_MASK;
	}
	if(ENET_EIR & ENET_EIR_MII_MASK)
	{
		SDprint("ENET_EIR_MII_MASK error\n\r");
        printf(" >> %x", ENET_MMFR);
		ENET_EIR |= ENET_EIR_MII_MASK;
	}
	if(ENET_EIR & ENET_EIR_LC_MASK)
	{
		SDprint("ENET_EIR_LC_MASK error\n\r");
		ENET_EIR |= ENET_EIR_LC_MASK;
	}
	if(ENET_EIR & ENET_EIR_RL_MASK)
	{
		SDprint("ENET_EIR_RL_MASK error\n\r");
		ENET_EIR |= ENET_EIR_RL_MASK;
	}
	if(ENET_EIR & ENET_EIR_UN_MASK)
	{
		SDprint("ENET_EIR_UN_MASK error\n\r");
		ENET_EIR |= ENET_EIR_UN_MASK;
	}
}

static void StartRX()
{
    ENET_RDAR = ENET_RDAR_RDAR_MASK;
    while( !ENET_RDAR ) { /* if this gets stuck there is a DMA engine problem */
    }
}
/********************************************************************/
static int enetWritePacket(ENET_MemMapPtr enet, uint8_t* buffer, unsigned len)
{
    int num_txbds;
    int cur_txbd;
    int i;
    uint16_t buf_len;
    unsigned written_len = 0;
    ENET_MemMapPtr fec = enet;
	
    cur_txbd = next_txbd;
    num_txbds = (len/CFG_ENET_BUFFER_SIZE);
    if((num_txbds * CFG_ENET_BUFFER_SIZE) < len) {
        num_txbds = num_txbds + 1;
    }
    
    for (i = 0; i < num_txbds; i++) {

        /* Block while buffer is still in use, make sure xmit is on */
        while (pxENETTxDescriptor[cur_txbd].status & (TX_BD_R)) {
            if (!(fec->TDAR & ENET_TDAR_TDAR_MASK))  
                fec->TDAR |= ENET_TDAR_TDAR_MASK;
				fec->ECR = ENET_ECR_ETHEREN_MASK;
				pxENETTxDescriptor[cur_txbd].status |= (TX_BD_R);
        }
        
        pxENETTxDescriptor[cur_txbd].status |= (TX_BD_TC);
        if(i == num_txbds - 1) {
            buf_len = (uint16_t) len;
            pxENETTxDescriptor[cur_txbd].status |= (TX_BD_L);      
        } else {
            buf_len = (uint16_t) CFG_ENET_BUFFER_SIZE;
            len -= CFG_ENET_BUFFER_SIZE;
        }
        pxENETTxDescriptor[cur_txbd].length = end_bswap16(buf_len);
        
//        memcpy((void *)__little2Big_end((uint32_t)pxENETTxDescriptor[cur_txbd].data), (void *)((uint32_t) buffer), buf_len); 

        buffer += buf_len;
        written_len += buf_len;

        /* This buffer is ready to go, check for wrap condition */
        if(cur_txbd == (ENET_NUM_TXBDS - 1))
        {
            pxENETTxDescriptor[cur_txbd].status |= (TX_BD_W | TX_BD_R);
            cur_txbd = 0;
        } else {
            pxENETTxDescriptor[cur_txbd].status |= (TX_BD_R);
            cur_txbd++;
        }
    }
    
    next_txbd = cur_txbd;
    fec->TDAR = ENET_TDAR_TDAR_MASK; /* Tell FEC to let 'r rip */
    return written_len;
}


/*******************************************************************************/
static int enetReadPacket(NBUF *enet, uint8_t* buffer, unsigned maxlen )
{
    int last_buffer;
    int cur_rxbd;
    int accumulated_len;
    int read_len;
    NBUF* rx_packet = enet;

    last_buffer = 0;
    rx_packet->length = 0;
    accumulated_len = 0;
    read_len = 0;
    cur_rxbd = next_rxbd;

    if (maxlen < ETH_MIN_FRM) return -1;
    if(pxENETRxDescriptors[cur_rxbd].status & RX_BD_E) return -1;    


    while(!last_buffer) {
        rx_packet->data = (uint8_t *)__little2Big_end((uint32_t)pxENETRxDescriptors[cur_rxbd].data);
        rx_packet->status = pxENETRxDescriptors[cur_rxbd].status;
        rx_packet->length = /*end_bswap16*/(pxENETRxDescriptors[cur_rxbd].length);
        last_buffer = (rx_packet->status & RX_BD_L);

        if(last_buffer) {
            read_len = (rx_packet->length - accumulated_len); /* On last BD len is total */
        } else {
            read_len = rx_packet->length; /* On intermediate BD's len is BUFFER_SIZE */
        }

        /* accumation of all reads cannot blow past passed maxlen, so clamp read_len */
        read_len = ((accumulated_len + read_len) <= maxlen) ? read_len : (maxlen - accumulated_len);
        if (read_len > 0) { /* Got data to copy, move it so buffer can be re-used */
            memcpy((void *)buffer, (void *) rx_packet->data, read_len);
            buffer += read_len;
            accumulated_len += read_len;
        }

        /* Mark rxbd as empty so uDMA can re-use it */
        if(rx_packet->status & RX_BD_W) {
            pxENETRxDescriptors[cur_rxbd].status = (RX_BD_W | RX_BD_E);
            cur_rxbd = 0;
        } else {
            pxENETRxDescriptors[cur_rxbd].status = RX_BD_E;
            cur_rxbd++;
        }
    }

    StartRX();
    next_rxbd = cur_rxbd;
    return accumulated_len;
}

unsigned SD_enetWritePolled(void *enetPtr, const void *data, unsigned len)
{
    ENET_MemMapPtr enet = enetPtr;
    uint8_t *dataPtr = (uint8_t *) data;

    return enetWritePacket(enet, dataPtr, len);
}
unsigned SD_enetReadPolled(void *enetPtr, const void *data, unsigned len)
{
    ENET_MemMapPtr enet = enetPtr;
    uint8_t *dataPtr = (uint8_t *) data;

    return enetReadPacket(pxENETRxDescriptors, dataPtr, len);
}