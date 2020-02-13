/*
 * SD_ETH.c
 *
 *  Created on: JAN 03, 2020
 *      Author: SD
 */
 
#include "packet_Handler.h"
#include "SD_GPIO.h"
//#include "common.h"
#include <string.h>

#define ENET_NUM_RXBDS 16           /* defines number of BD,s and buffers */
#define ENET_NUM_TXBDS 16

ENET_PHY_t SD_eth_PHY;
uint8_t *macAdd;
////!< ENET DMA Tx&Rx Descriptors memory regin(must be 16 bit agiged)
extern  uint8_t pxENETTxDescriptor_unaligned[100];
extern  uint8_t pxENETRxDescriptors_unaligned[100];

/* Buffer descriptors  MUST BE 16-byte boundary aligned */
static enet_descr_t *pxENETTxDescriptor __attribute__ ((aligned (16)));
static enet_descr_t *pxENETRxDescriptors __attribute__ ((aligned (16)));
uint8_t SD_TxBuff[1518 + 16],SD_RxBuff[1518 + 16];
////!< Callback function slot
//static ENET_CallBackTxType ENET_CallBackTxTable = (void *)EnetTxCB;
//static ENET_CallBackRxType ENET_CallBackRxTable = (void *)EnetRxCB;
////!< enet received memory pool
//static uint8_t ucENETRxBuffers[ ( CFG_NUM_ENET_RX_BUFFERS * CFG_ENET_BUFFER_SIZE ) + 16 ];
static uint8_t ENET_MII_Write(uint16_t phy_addr, uint16_t reg_addr, uint16_t data);
static uint8_t ENET_MII_Read(uint16_t phy_addr, uint16_t reg_addr, uint16_t *data);

uint8_t macPhyAdd[] = {0x5D,0x5D,0x5D,0x5D,0x5D,0x5D};
unsigned short ipv4_check(uint16_t *buf, unsigned size)
{
	//unsigned sum = 0;
	int i, num = 0x00000000, fin_num = 0x00000000;
    short int res, carry;

	/* Accumulate checksum */
	for (i = 0; i < size; i++)
	{
		num += (uint16_t)buf[i];
	}
	/* Handle odd-sized case */
	if (num > (int)res)
	{
		carry = (num & 0x000f0000) >> 16;
		num += carry;
        res = num;
	}

	/* Fold to get the ones-complement result 
	while (sum >> 16) sum = (sum & 0xFFFF)+(sum >> 16);

	/* Invert to get the negative in ones-complement arithmetic */
	return ~res;
}
/**
 * @brief ��ʼ����̫��������������
 * @note  ��̫��ģ��Ϊ�����ٶ� ͨ��nent�ڲ�DMA�����û��Զ����ڴ��� ÿ���ڴ�����Ҫ����������
 * @retval None
 */
static void ENET_BDInit(void)
{
    unsigned long ux;
    unsigned char *pcBufPointer;
	/* find a 16bit agligned for TxDescriptors */
	pcBufPointer = &( pxENETTxDescriptor_unaligned[ 0 ]);
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETTxDescriptor = ( enet_descr_t * ) pcBufPointer;
	pcBufPointer = &( pxENETRxDescriptors_unaligned[ 0 ]);
	/* find a 16bit agligned for RxDescriptors */
//	pcBufPointer = &( pxENETRxDescriptors_unaligned[ 0 ] );
//	pcBufPointer += sizeof(enet_descr_t);
	while( ( ( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	pxENETRxDescriptors = ( enet_descr_t * ) pcBufPointer;	
	pcBufPointer = &( SD_TxBuff[ 0 ] );
	while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
//	pcBufPointer = &( pxENETRxDescriptors_unaligned[ 0 ]);
	for( ux = 0; ux < CFG_NUM_ENET_TX_BUFFERS; ux++ )
	{
        pxENETTxDescriptor->status = 0;
        pxENETTxDescriptor->length = 0;
		pcBufPointer += CFG_ENET_BUFFER_SIZE;
//		pxENETTxDescriptor->buf_addr = (uint8_t *)__little2Big_end((uint32_t)pxENETTxDescriptor_unaligned);
    }
	pcBufPointer = &( SD_RxBuff[ 0 ] );
	while( ( ( unsigned long ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	for( ux = 0; ux < CFG_NUM_ENET_RX_BUFFERS; ux++ )
	{
	    pxENETRxDescriptors->status = 0;
	    pxENETRxDescriptors->length = 0;
		pcBufPointer += CFG_ENET_BUFFER_SIZE;
//        pxENETRxDescriptors->buf_addr = (uint8_t *)__little2Big_end((uint32_t)pxENETRxDescriptors_unaligned);
	}
//    /* Tx Descriptor settings */
//	/* find a 16bit agligned for Rx buffer */
//	while((( uint32_t ) pcBufPointer & 0x0fUL ) != 0 )
//	{
//		pcBufPointer++;
//	}
	/* Rx Descriptor settings */
	/* set last Descriptor as a ring */
//    pxENETTxDescriptor[CFG_NUM_ENET_TX_BUFFERS - 1].status |= TX_BD_W;
//	pxENETRxDescriptors[CFG_NUM_ENET_RX_BUFFERS - 1].status |= RX_BD_W;
//    pxENETTxDescriptor[CFG_NUM_ENET_TX_BUFFERS - 1].status &= ~TX_BD_W;
//	pxENETRxDescriptors[CFG_NUM_ENET_RX_BUFFERS - 1].status &= ~RX_BD_W;
	/* Set the wrap bit in the last descriptors to form a ring. */
	pxENETTxDescriptor[ CFG_NUM_ENET_TX_BUFFERS - 1 ].status |= TX_BD_W;
	pxENETRxDescriptors[ CFG_NUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;
	//uxNextRxBuffer = 0;  uxNextTxBuffer = 0;

}

/********************************************************************/
static void enetBDInit(ENET_MemMapPtr enet)
{
    enet->MRBR = (uint16_t)CFG_ENET_BUFFER_SIZE;  
    enet->RDSR = (uint32_t)pxENETRxDescriptors;
    enet->TDSR = (uint32_t)pxENETTxDescriptor;
}
/********************************************************************/
static void enetBDStartRX(ENET_MemMapPtr enet)
{

    enet->RDAR= ENET_RDAR_RDAR_MASK;
    while( !enet->RDAR ) { /* if this gets stuck there is a DMA engine problem */
    }
}
/**
 * @brief �����ϣУ��ֵ
 * @note  ��������
 * @retval Hashֵ
 */
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
    ENET_PAUR = (uint32_t)((pa[4]<<24) | (pa[5]<<16) | (0x88));
    crc = ENET_HashAddress(pa);
    if(crc >= 32)
    ENET_IAUR |= (uint32_t)(1 << (crc - 32));
    else
    ENET_IALR |= (uint32_t)(1 << crc);
}

/**
 * @brief  ��ʼ����̫�� MII���ò�ӿ�
 * @param  None
 * @retval None
 */
static void ENET_MII_Init(void)
{
    uint32_t i;
    uint32_t clockMHZ;
	uint16_t usData;
	/* enable the ENET clock. */
    SIM_SCGC2 |= SIM_SCGC2_ENET_MASK;
	SIM_SOPT2 &= ~SIM_SOPT2_TIMESRC_MASK;
    /* FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error */
    MPU_CESR = 0;   
    clockMHZ = 100;
    i = clockMHZ;
   //Reveive control register
	ENET_RCR = ENET_RCR_MAX_FL(1518) | ENET_RCR_RMII_MODE_MASK | ENET_RCR_MII_MODE_MASK;
    ENET_RCR &= ~ENET_RCR_LOOP_MASK;
    ENET_RCR |= ENET_RCR_PROM_MASK;
   //Transmit control register
	ENET_TCR = 0;
    ENET_MSCR = 0 | ENET_MSCR_MII_SPEED((2*i/5)+1);
	/* Can we talk to the PHY? */
	do
	{
		msDelay(10);
	usData = 0xffff;
	ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
	}
	while( usData == 0xffff );
	/* Start auto negotiate. */
	ENET_MII_Write(CFG_PHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) ); 
}

/**
 * @brief  д����̫��MII���ò�����
 * @param   phy_addr  :PHYоƬ��ַ
 * @param   reg_addr  :�Ĵ�����PHY�ڲ���ƫ�Ƶ�ַ
 * @param   data      :��Ҫд�������
 * @retval  0 :�ɹ� ���� :ʧ��
 */
static uint8_t ENET_MII_Write(uint16_t phy_addr, uint16_t reg_addr, uint16_t data)
{
    uint32_t timeout;
    /* clear MII it pending bit */
    ENET_EIR |= ENET_EIR_MII_MASK;
    /* initiatate the MII Management write */
    ENET_MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x01)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02)
            | ENET_MMFR_DATA(data);
    /* waitting for transfer complete */
    for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
    {
        if (ENET_EIR & ENET_EIR_MII_MASK)
        {
            printf("MII - timeout:%d\r\n", timeout);
            break;  
        }
    }
    if(timeout == MII_TIMEOUT)
    {
        return timeout;
    }
    /* software clear it */
    ENET_EIR |= ENET_EIR_MII_MASK;
    return 0;
}

/**
 * @brief  ����̫��MII���ò�����
 * @param   phy_addr    :PHYоƬ��ַ
 * @param   reg_addr    :�Ĵ�����PHY�ڲ���ƫ�Ƶ�ַ
 * @param   data        :��Ҫ��������ݵ�ַ
 * @retval  0 :�ɹ� ���� :ʧ��
 */
static uint8_t ENET_MII_Read(uint16_t phy_addr, uint16_t reg_addr, uint16_t *data)
{
    uint32_t timeout;
    /* clear MII IT(interrupt) pending bit */
    ENET_EIR |= ENET_EIR_MII_MASK;
    /* initiatate the MII Management write */
    ENET_MMFR = 0
            | ENET_MMFR_ST(0x01)
            | ENET_MMFR_OP(0x02)
            | ENET_MMFR_PA(phy_addr)
            | ENET_MMFR_RA(reg_addr)
            | ENET_MMFR_TA(0x02);
  
	/* waitting for transfer complete */
    for (timeout = 0; timeout < MII_TIMEOUT; timeout++)
    {
        if (ENET_EIR & ENET_EIR_MII_MASK)
        {
            break; 
        }
    }
    if(timeout == MII_TIMEOUT) 
    {
        return timeout;
    }
    /* software clear it */
    ENET_EIR |= ENET_EIR_MII_MASK;
    *data = ENET_MMFR & 0x0000FFFF;
    return 0;
}

/**
 * @brief   ��ʼ����̫��ģ��
 * @note    �û����ú���
 * @param   ENET_InitStrut   :��̫����ʼ���ṹָ�룬���Ӧ������
 * @retval  None
 */
void ENET_Init(uint8_t* mac)
{
	uint16_t usData;
	uint16_t timeout = 0;
    SIM_SCGC2 |= SIM_SCGC2_ENET_MASK;
	SIM_SOPT2 = SIM_SOPT2_TIMESRC(0x2);
    MPU_CESR = 0;
   //Reset statistics counters
	ENET_MIBC = ENET_MIBC_MIB_CLEAR_MASK;
	ENET_MIBC = 0;
    ENET_BDInit();
	//MCG->C2 &= ~MCG_C2_EREFS0_MASK;
	ENET_ECR = ENET_ECR_RESET_MASK;
	while(ENET_ECR & ENET_ECR_RESET_MASK){}
	ENET_PAUR = ENET_PAUR_TYPE(0x0800);
	for( usData = 0; usData < 100; usData++ )
	{
		//__NOP();
	}
    ENET_MII_Init();
	/* Can we talk to the PHY? */
	//enet_interrupt_routine
	set_irq_priority (76, 6);
	enable_irq(76);//ENET xmit interrupt
	//enet_interrupt_routine
	set_irq_priority (77, 6);
	enable_irq(77);//ENET rx interrupt
	//enet_interrupt_routine
	set_irq_priority (78, 6);
	enable_irq(78);//ENET error and misc interrupts
	/* Make sure the external interface signals are enabled   */
    do
    {
        msDelay(10);
        timeout++;
        if(timeout > 50) break;
        usData = 0xFFFF;
        ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
    }
	while( (usData == 0xFFFF) || (usData == 0x0000));
	SDprint("We can talk now\n\r");
	
    /* software reset PHY *//* Start auto negotiate. */
    ENET_MII_Write(CFG_PHY_ADDRESS, PHY_BMCR, PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE);
    msDelay(100);
	/* Wait for auto negotiate to complete. */
	do
	{
        msDelay(10);
		ENET_MII_Read(CFG_PHY_ADDRESS, PHY_BMSR, &usData );
	}
	while( !( usData & PHY_BMSR_AN_COMPLETE ) );
	SDprint("auto negotiate done\n\r");
  /* When we get here we have a link - find out what has been negotiated. */
	usData = 0;
	
	ENET_MII_Read(CFG_PHY_RMII, BC_REG, &usData );
    printf("BC_REG:0x%X\r\n",usData);
	SD_eth_PHY.bc_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, BS_REG, &usData );
    printf("BS_REG:0x%X\r\n",usData);
	SD_eth_PHY.bs_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, PHY_ID1_REG, &usData );
    printf("PHY_ID1_REG:0x%X\r\n",usData);
	SD_eth_PHY.phy_id1_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, PHY_ID2_REG, &usData );
    printf("PHY_ID2_REG:0x%X\r\n",usData);
	SD_eth_PHY.phy_id2_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, A_NEG_ADVT_REG, &usData );
    printf("A_NEG_ADVT_REG:0x%X\r\n",usData);
	SD_eth_PHY.a_neg_advt_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, A_NEG_ADVT_PARTNER_REG, &usData );
    printf("A_NEG_ADVT_PARTNER_REG:0x%X\r\n",usData);
	SD_eth_PHY.a_neg_advt_partner_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, A_NEG_EXP_REG, &usData );
    printf("A_NEG_EXP_REG:0x%X\r\n",usData);
	SD_eth_PHY.a_neg_exp_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, A_NEG_NXT_PG_REG, &usData );
    printf("A_NEG_NXT_PG_REG:0x%X\r\n",usData);
	SD_eth_PHY.a_neg_nxt_pg_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, A_NEG_NXT_PG_PARTNER_REG, &usData );
    printf("A_NEG_NXT_PG_PARTNER_REG:0x%X\r\n",usData);
	SD_eth_PHY.a_neg_advt_partner_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, MII_CTRL_REG, &usData );
    printf("MII_CTRL_REG:0x%X\r\n",usData);
	SD_eth_PHY.mii_ctrl_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, RXER_Counter_REG, &usData );
    printf("RXER_Counter_REG:0x%X\r\n",usData);
	SD_eth_PHY.rxer_counter_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, INT_REG, &usData );
    printf("INT_REG:0x%X\r\n",usData);
	SD_eth_PHY.int_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, PHY_CTRL1_REG, &usData );
    printf("PHY_CTRL1_REG:0x%X\r\n",usData);
	SD_eth_PHY.phy_ctrl1_reg = usData;
	ENET_MII_Read(CFG_PHY_RMII, PHY_CTRL2_REG, &usData );
    printf("PHY_CTRL2_REG:0x%X\r\n",usData);
	SD_eth_PHY.phy_ctrl2_reg = usData;
/*	ENET_MII_Read(CFG_PHY_ADDRESS, PHY_STATUS, &usData );
    printf("PHY_STATUS:0x%X\r\n",usData);  
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_BMCR, &usData );
    printf("PHY_BMCR:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYIDR1, &usData );
    printf("PHY_PHYIDR1:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYIDR2, &usData );
    printf("PHY_PHYIDR2:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_ANAR, &usData );
    printf("PHY_ANAR:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_ANLPAR, &usData );
    printf("PHY_ANLPAR:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_ANER, &usData );
    printf("PHY_ANER:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_LPNPA, &usData );
    printf("PHY_LPNPA:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_RXERC, &usData );
    printf("PHY_RXERC:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_ICS, &usData );
    printf("PHY_ICS:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYC1, &usData );
    printf("PHY_PHYC1:0x%X\r\n",usData);
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_PHYC1, &usData );
    printf("PHY_PHYC1:0x%X\r\n",usData);

    /* ��ʼ�Զ�Э�� 
    ENET_MII_Write(CFG_PHY_ADDRESS, PHY_BMCR, (PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ));
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_BMCR, &usData );
    printf("PHY_BMCR=0x%X\r\n",usData);*/
    /* �ȴ��Զ�Э����� 
    do
    {
        msDelay(100);
		timeout++;
		if(timeout > 30)
        {
            printf("enet Auto-Negotiation failed\r\n");
            break;
        }
        ENET_MII_Read(CFG_PHY_ADDRESS, PHY_BMSR, &usData );
    } while( !( usData & PHY_BMSR_AN_COMPLETE ) );
    /* ����Э�̽������ENETģ�� */
    usData = 0;
    	
  
    /* ������������ַ��ϣ�Ĵ��� */
    ENET_IALR = 0;
    ENET_IAUR = 0;
    ENET_GALR = 0;
    ENET_GAUR = 0;
    //����ENETģ��MAC��ַ
    ENET_SetAddress(mac);
	macAdd = mac;
    //���ý��տ��ƼĴ�������󳤶ȡ�RMIIģʽ������CRCУ���
    ENET_RCR = ENET_RCR_MAX_FL(CFG_ENET_MAX_PACKET_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK | ENET_RCR_RMII_MODE_MASK;
    //������ͽ��տ���
    ENET_TCR = 0;
    ENET_MII_Read(CFG_PHY_ADDRESS, PHY_STATUS, &usData );
    if( usData & PHY_DUPLEX_STATUS )
    {
        ENET_RCR &= (unsigned long)~ENET_RCR_DRT_MASK;
        ENET_TCR |= ENET_TCR_FDEN_MASK;
        printf("full-duplex\r\n");
    }
    else
    {
        ENET_RCR |= ENET_RCR_DRT_MASK;
        ENET_TCR &= (unsigned long)~ENET_TCR_FDEN_MASK;
        printf("half-duplex\r\n");
    }
    //ͨ����������
    if( usData & PHY_SPEED_STATUS )
    {
        /* 10Mbps */
        printf("speed:10M\r\n");
        ENET_RCR |= ENET_RCR_RMII_10T_MASK;
    }
    /* ʹ����ǿ�ͻ����������� */
    ENET_ECR = ENET_ECR_EN1588_MASK;
    /* max receiced packet size *//* Set Rx Buffer Size */
    ENET_MRBR |= ENET_MRBR_R_BUF_SIZE_MASK;
	/* tell NENT the descriptors address *//* Point to the start of the circular Rx buffer descriptor queue */
	ENET_RDSR = (uint32_t)((uint32_t *)pxENETRxDescriptors);//(uint32_t)  pxENETRxDescriptors;
	/* Point to the start of the circular Tx buffer descriptor queue */
	ENET_TDSR = (uint32_t)((uint32_t *)pxENETTxDescriptor);//(uint32_t) pxENETTxDescriptor;
	/* clear all IT pending bit */
	ENET_EIR = ( uint32_t ) 0xFFFFFFFF;
	/* Enable interrupts */
	ENET_EIMR = 0xFFFFFFFF;//(ENET_EIR_TXF_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_RXB_MASK | ENET_EIMR_UN_MASK | ENET_EIMR_RL_MASK | ENET_EIMR_LC_MASK | ENET_EIMR_BABT_MASK | ENET_EIMR_BABR_MASK | ENET_EIMR_EBERR_MASK);

	ENET_TCR &= ~ENET_TCR_ADDSEL_MASK;
	
	ENET_ECR |= ENET_ECR_ETHEREN_MASK;
  	/* Indicate that there have been empty receive buffers produced */
	ENET_RDAR = ENET_RDAR_RDAR_MASK;

}

/**
 * @brief  ����һ֡��̫֡����
 * @note    �û����ú���
 * @param   data    :��������ָ��
 * @param   len     :���ݳ��� (< 1500�ֽ�)
 * @retval  None
 */
void ENET_MacSendData(uint16_t *data, uint16_t len)
{	
	uint16_t i;
	pxENETTxDescriptor->status |= ~( TX_BD_R | TX_BD_L | TX_BD_TC | TX_BD_W );
    pxENETTxDescriptor->length = end_bswap16(len);//__REVSH(len);
	for(i = 0; i < 300; i++)
		pxENETTxDescriptor->buf_addr[i] = end_bswap16(data[i]);
    /* set Tx Descriptor */
//    pxENETTxDescriptor->buf_addr = (uint8_t *)__little2Big_end((uint32_t)data);//(uint8_t *)__REV((uint32_t)data);		
//    pxENETTxDescriptor->bdu = 0x00000000;
//	pxENETTxDescriptor->ebd_status = TX_BD_INT | TX_BD_TS;// | TX_BD_IINS | TX_BD_PINS;
    /* enable transmit */
    ENET_TDAR = ENET_TDAR_TDAR_MASK;
	pxENETTxDescriptor->status |= TX_BD_R | TX_BD_TC;
    /* check if buffer is readly */
    while( pxENETTxDescriptor->status & TX_BD_R ) {};
}

/**
 * @brief  ����һ֡��̫֡����
 * @note    �û����ú���
 * @param   data    :����ָ��
 * @retval  ���յ������ݳ���
 */
uint16_t ENET_MacReceiveData(uint8_t *data)
{
    uint16_t len = 0;
    /* if buffer is ready */
    if(!(pxENETRxDescriptors->status & end_bswap16(RX_BD_E) ) && (pxENETRxDescriptors->status & end_bswap16(RX_BD_L) ))
    {
		uint16_t i;
		/* copy data to user bufer */
		len =  /*__REVSH*/end_bswap16(pxENETRxDescriptors[0].length);
		for(i=0; i < 20; i++)
			data[i] = end_bswap16(pxENETRxDescriptors[0].buf_addr[i]);
		//memcpy(data, (uint8_t *)/*__REV(*/__little2Big_end((uint32_t)pxENETRxDescriptors[0].buf_addr), len);
		/* buffer is ready and data is readed */
		pxENETRxDescriptors[0].status |= RX_BD_E;
		ENET_RDAR = ENET_RDAR_RDAR_MASK;
        return len;
	}
	return 0;
}

/**
 * @brief  ����ENETģ����жϻ���DMA����
 * @param  config     :ģʽѡ��
 *         @arg kENET_IT_TXF_Disable:��ֹ����һ֡��̫������֡�ж�
 *         @arg kENET_IT_RXF_Disable:��ֹ����һ֡��̫������֡�ж�
 *         @arg kENET_IT_TXF        :����һ֡��̫�������ж�
 *         @arg kENET_IT_RXF        :����һ֡��̫�������ж�
 * @retval None
 */
void ENET_ITDMAConfig(ENET_ITDMAConfig_Type config)
{
    switch(config)
    {
        case kENET_IT_TXF_Disable:
            ENET_EIMR &= ~ENET_EIMR_TXF_MASK;
            break;
        case kENET_IT_RXF_Disable:
            ENET_EIMR &= ~ENET_EIMR_RXF_MASK;
            break;
        case kENET_IT_TXF:
            enable_irq(ENET_Transmit_IRQn);
            ENET_EIMR |= ENET_EIMR_TXF_MASK;
            break;
        case kENET_IT_RXF:
            enable_irq(ENET_Receive_IRQn);
            ENET_EIMR |= ENET_EIMR_RXF_MASK;
            break;
        default:
            break;
    }
}

/**
 * @brief  ����ENET�����жϻص�����
 * @param  AppCBFun: �ص�����ָ��
 * @retval None
 */
//void ENET_CallbackTxInstall(ENET_CallBackTxType AppCBFun)
//{
//    if(AppCBFun != NULL)
//    {
//        ENET_CallBackTxTable = AppCBFun;
//    }
//}

/**
 * @brief  ����ENET�����жϻص�����
 * @param  AppCBFun: �ص�����ָ��
 * @retval None
 */
//void ENET_CallbackRxInstall(ENET_CallBackRxType AppCBFun)
//{
//    if(AppCBFun != NULL)
//    {
//        ENET_CallBackRxTable = AppCBFun;
//    }
//}

uint32_t ENET_IsTransmitComplete(void)
{
    if(ENET_EIR & ENET_EIMR_TXF_MASK)
    {
        ENET_EIR |= ENET_EIMR_TXF_MASK;
        return 1;
    }
    return 0;
}

//void ENET_Transmit_IRQHandler()
//{
//	SDprint("Inside ENET_Transmit_IRQHandler\n\r");
//	if(ENET_EIR & ENET_EIMR_TXF_MASK)
//	{
//		SDprint("Packet transferred\n\r");
//		ENET_EIR |= ENET_EIMR_TXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_TXB_MASK)
//	{
//		SDprint("Packet still there for tx\n\r");
//		ENET_EIR |= ENET_EIMR_TXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXF_MASK)
//	{
//		SDprint("Packet received\n\r");
//		ENET_EIR |= ENET_EIMR_RXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXB_MASK)
//	{
//		SDprint("Packet still there for rx\n\r");
//		ENET_EIR |= ENET_EIMR_RXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_GRA_MASK)
//	{
//		SDprint("ENET_EIR_GRA_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_GRA_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABR_MASK)
//	{
//		SDprint("ENET_EIR_BABR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABT_MASK)
//	{
//		SDprint("ENET_EIR_BABT_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABT_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_EBERR_MASK)
//	{
//		SDprint("ENET_EIR_EBERR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_EBERR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
//	{
//		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_PLR_MASK)
//	{
//		SDprint("ENET_EIR_PLR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_PLR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_MII_MASK)
//	{
//		SDprint("ENET_EIR_MII_MASK error\n\r");
//        printf(" >> %x\n\r", ENET_MMFR);
//		LED_GREEN_TOGGLE;
//		ENET_EIR |= ENET_EIR_MII_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_LC_MASK)
//	{
//		SDprint("ENET_EIR_LC_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_LC_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_RL_MASK)
//	{
//		SDprint("ENET_EIR_RL_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_RL_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_UN_MASK)
//	{
//		SDprint("ENET_EIR_UN_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_UN_MASK;
//	}
//		
//	ENET_EIR |= ENET_EIMR_TXF_MASK;
//	pxENETTxDescriptor->status &= ~TX_BD_R;
//}
//void ENET_Receive_IRQHandler()
//{
//		LED_GREEN_TOGGLE;
//	SDprint("Inside ENET_Receive_IRQHandler\n\r");	
//	if(ENET_EIR & ENET_EIMR_TXF_MASK)
//	{
//		SDprint("Packet transferred\n\r");
//		ENET_EIR |= ENET_EIMR_TXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_TXB_MASK)
//	{
//		SDprint("Packet still there for tx\n\r");
//		ENET_EIR |= ENET_EIMR_TXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXF_MASK)
//	{
//		SDprint("Packet received\n\r");
//		ENET_EIR |= ENET_EIMR_RXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXB_MASK)
//	{
//		SDprint("Packet still there for rx\n\r");
//		ENET_EIR |= ENET_EIMR_RXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_GRA_MASK)
//	{
//		SDprint("ENET_EIR_GRA_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_GRA_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABR_MASK)
//	{
//		SDprint("ENET_EIR_BABR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABT_MASK)
//	{
//		SDprint("ENET_EIR_BABT_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABT_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_EBERR_MASK)
//	{
//		SDprint("ENET_EIR_EBERR_MASK error\n\r");
//		ENET_ECR = ENET_ECR_RESET_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
//	{
//		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_PLR_MASK)
//	{
//		SDprint("ENET_EIR_PLR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_PLR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_MII_MASK)
//	{
//		SDprint("ENET_EIR_MII_MASK error\n\r");
//        printf(" >> %x", ENET_MMFR);
//		LED_GREEN_TOGGLE;
//		ENET_EIR |= ENET_EIR_MII_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_LC_MASK)
//	{
//		SDprint("ENET_EIR_LC_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_LC_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_RL_MASK)
//	{
//		SDprint("ENET_EIR_RL_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_RL_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_UN_MASK)
//	{
//		SDprint("ENET_EIR_UN_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_UN_MASK;
//	}
//		
//	ENET_EIR |= ENET_EIMR_RXF_MASK;
//	pxENETTxDescriptor->status &= ~TX_BD_R;
//}
//void ENET_Error_IRQHandler()
//{
//	SDprint("Inside ENET_Error_IRQHandler\n\r");
//	if(ENET_EIR & ENET_EIMR_TXF_MASK)
//	{
//		SDprint("Packet transferred\n\r");
//		ENET_EIR |= ENET_EIMR_TXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_TXB_MASK)
//	{
//		SDprint("Packet still there for tx\n\r");
//		ENET_EIR |= ENET_EIMR_TXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXF_MASK)
//	{
//		SDprint("Packet received\n\r");
//		ENET_EIR |= ENET_EIMR_RXF_MASK;
//	}
//	if(ENET_EIR & ENET_EIMR_RXB_MASK)
//	{
//		SDprint("Packet still there for rx\n\r");
//		ENET_EIR |= ENET_EIMR_RXB_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_TS_AVAIL_MASK)
//	{
//		SDprint("ENET_EIR_TS_AVAIL_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_TS_AVAIL_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_TS_TIMER_MASK)
//	{
//		SDprint("ENET_EIR_TS_TIMER_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_TS_TIMER_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_GRA_MASK)
//	{
//		SDprint("ENET_EIR_GRA_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_GRA_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABR_MASK)
//	{
//		SDprint("ENET_EIR_BABR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_BABT_MASK)
//	{
//		SDprint("ENET_EIR_BABT_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_BABT_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_EBERR_MASK)
//	{
//		SDprint("ENET_EIR_EBERR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_EBERR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_WAKEUP_MASK)
//	{
//		SDprint("ENET_EIR_WAKEUP_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_WAKEUP_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_PLR_MASK)
//	{
//		SDprint("ENET_EIR_PLR_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_PLR_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_MII_MASK)
//	{
//		SDprint("ENET_EIR_MII_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_MII_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_LC_MASK)
//	{
//		SDprint("ENET_EIR_LC_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_LC_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_RL_MASK)
//	{
//		SDprint("ENET_EIR_RL_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_RL_MASK;
//	}
//	if(ENET_EIR & ENET_EIR_UN_MASK)
//	{
//		SDprint("ENET_EIR_UN_MASK error\n\r");
//		ENET_EIR |= ENET_EIR_UN_MASK;
//	}
//}
int8_t ENET_pin_init(uint8_t mode)
{
	int8_t ret;
	switch(mode)
	{
		case ENET_MII_MODE_MASK:
			PORTA_PCR5 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR9 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR10 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR11 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR12 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR14 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR14 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR15 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR16 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR17 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR24 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR25 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR26 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR27 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR28 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR29 |= PORT_PCR_MUX(0x4);
			PORTB_PCR0 |= PORT_PCR_MUX(0x4); 
			PORTB_PCR1 |= PORT_PCR_MUX(0x4);
			ret = ENET_MII_MODE_MASK;
		break;
		
		case ENET_RMII_MODE_MASK:
			PORTA_PCR5 |= PORT_PCR_MUX(0x4);
			PORTA_PCR12 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR13 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR14 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR15 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR16 |= PORT_PCR_MUX(0x4); 
			PORTA_PCR17 |= PORT_PCR_MUX(0x4); 			
			PORTB_PCR0 |= PORT_PCR_MUX(0x4);
			PORTB_PCR1 |= PORT_PCR_MUX(0x4);
			ret = ENET_MII_MODE_MASK;
		break;
			
		
		default:
			ret = -1;
	}
	return ret;
}


/*
uint8_t IPv4_packer(uint8_t * src_ip_add, uint8_t * dest_ip_add, uint8_t * data, int data_len, uint8_t * txData)
{
	static uint16_t count;
	static uint16_t dataBuffIndx = 0;
	uint8_t i;
	IPv_4_ENET_PACKT_T packet;
	packet.TOS = 0xFF;
	packet.PROTO = 0x4;
	packet.;
	packet.FLAG = 0x01;
	packet.VER = 0x4;
	packet.IP_HEAD_LEN_B = 5;
	packet.DATAGRAM_LENGTH_B = 160;
	packet.PROTO = TCP_MASK;
	for(i = 0; i < 4; i++)
	{
		packet.SRC_IP = (uint32_t)(src_ip_add[i] << ((3-i)*8));
		packet.DEST_IP = (uint32_t)(dest_ip_add[i] << ((3-i)*8));
	}
	
	count++;
	
	packet.CHECK = ipv4_check();
}*/
//uint16_t ipv4_check(IPv_4_ENET_PACKT_T buf, unsigned size)
//{
//	//unsigned sum = 0;
//	int i, num = 0x00000000;
//    uint16_t res, carry;

//	/* Accumulate checksum */
//	for (i = 0; i < size; i++)
//	{
//		num += buf.rawBytes[i];
//	}
//	/* Handle odd-sized case */
//	if (num > (int)res)
//	{
//		carry = (num & 0x000f0000) >> 16;
//		num += carry;
//        res = num;
//	}

//	/* Fold to get the ones-complement result 
//	while (sum >> 16) sum = (sum & 0xFFFF)+(sum >> 16);

//	/* Invert to get the negative in ones-complement arithmetic */
//	return (uint16_t)(~res);
//}
uint8_t EnetTxCB()
{
	uint8_t ret = SDprint((char *)pxENETTxDescriptor[1].buf_addr);
	return ret;
}
uint16_t EnetRxCB()
{
	enetBDStartRX(ENET_BASE_PTR);
	return 1;
}
