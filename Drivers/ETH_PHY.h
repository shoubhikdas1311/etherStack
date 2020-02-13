#define BIT_SET_MASK(bit)                            1 << bit
#define BIT_MASK(num, bit)                          num << bit


#define CFG_PHY_MII								0x00
#define CFG_PHY_RMII							0x01
#define CFG_PHY_MII_100M						0x04

# define BC_REG                  0x0
# define BS_REG                  0x1
# define PHY_ID1_REG                0x2 
# define PHY_ID2_REG                 0x3
# define A_NEG_ADVT_REG                 0x4 
# define A_NEG_ADVT_PARTNER_REG            0x5      
# define A_NEG_EXP_REG                   0x6
# define A_NEG_NXT_PG_REG                  0x7  
# define A_NEG_NXT_PG_PARTNER_REG             0x8       
# define MII_CTRL_REG                    0x14
# define RXER_Counter_REG                    0x15
# define INT_REG                 0x1b
# define PHY_CTRL1_REG               0x1e    
# define PHY_CTRL2_REG                   0x1f

# define BC_Reset_MASK                   BIT_SET_MASK(15)
# define BC_LoopBack_MASK                    BIT_SET_MASK(14)
# define BC_Speed_Select_MASK                    BIT_SET_MASK(13)
# define BC_AutoNegotiation_Enable_MASK                  BIT_SET_MASK(12)
# define BC_Power_Down_MASK                  BIT_SET_MASK(11)
# define BC_Isolate_MASK                 BIT_SET_MASK(10)
# define BC_Restart_AutoNegotiation_MASK                 BIT_SET_MASK(9)
# define BC_Duplex_Mode_MASK                 BIT_SET_MASK(8)
# define BC_Collision_Test_MASK                  BIT_SET_MASK(7)
# define BC_Reserved_MASK                    BIT_SET_MASK()
# define BC_Disable_Transmitter_MASK                 BIT_SET_MASK(0)

# define BS_100BASET4_MASK                   BIT_SET_MASK(15)
# define BS_100BASETX_Full_Duplex_MASK                   BIT_SET_MASK(14)
# define BS_100BASETX_Half_Duplex_MASK                   BIT_SET_MASK(13)
# define BS_SET_Full_Duplex_MASK                 BIT_SET_MASK(12)
# define BS_10BASET_Half_Duplex_MASK                 BIT_SET_MASK(11)
# define BS_Reserved_MASK                    BIT_SET_MASK()
# define BS_No_Preamble_MASK                 BIT_SET_MASK(6)
# define BS_AutoNegotiation_Complete_MASK                    BIT_SET_MASK(5)
# define BS_Remote_Fault_MASK                    BIT_SET_MASK(4)
# define BS_AutoNegotiation_Ability_MASK                 BIT_SET_MASK(3)
# define BS_Link_Status_MASK                 BIT_SET_MASK(2)
# define BS_Jabber_Detect_MASK                   BIT_SET_MASK(1)
# define BS_Extended_Capability_MASK                 BIT_SET_MASK(0)

# define PHY_ID1_Number_MASK(id)                 BIT_MASK(id,0)

# define PHY_ID2_Number_MASK(id)                 BIT_MASK(id,10)
# define PHY_ID2_Model_Number_MASK(mdl_no)                   BIT_MASK(mdl_no, 4)
# define PHY_ID2_Revision_Number_MASK(rev)                    BIT_MASK(rev, 0)
 
# define A_NEG_ADVT_Next_Page_MASK                   BIT_SET_MASK(15)
# define A_NEG_ADVT_Reserved_MASK                    BIT_SET_MASK()
# define A_NEG_ADVT_Remote_Fault_MASK                    BIT_SET_MASK(13)
# define A_NEG_ADVT_Reserved_MASK                    BIT_SET_MASK(12)
# define A_NEG_ADVT_Pause_MASK(pause)                   BIT_MASK(pause,10)
# define A_NEG_ADVT_100BASET4_MASK                   BIT_SET_MASK(9)
# define A_NEG_ADVT_100BASETX_FullDuplex_MASK                    BIT_SET_MASK(8)
# define A_NEG_ADVT_100BASETX_HalfDuplex_MASK                    BIT_SET_MASK(7)
# define A_NEG_ADVT_10BASET_FullDuplex_MASK                  BIT_SET_MASK(6)
# define A_NEG_ADVT_10BASET_HalfDuplex_MASK                  BIT_SET_MASK(5)
# define A_NEG_ADVT_Selector_Field_MASK(std)                  BIT_MASK(std,0)

# define A_NEG_ADVT_PARTNER_Next_Page_MASK                   BIT_SET_MASK(15)
# define A_NEG_ADVT_PARTNER_Reserved_MASK                    BIT_SET_MASK()
# define A_NEG_ADVT_PARTNER_Remote_Fault_MASK                    BIT_SET_MASK(13)
# define A_NEG_ADVT_PARTNER_Reserved_MASK                    BIT_SET_MASK(12)
# define A_NEG_ADVT_PARTNER_Pause_MASK(pause)                   BIT_MASK(pause,10)
# define A_NEG_ADVT_PARTNER_100BASET4_MASK                   BIT_SET_MASK(9)
# define A_NEG_ADVT_PARTNER_100BASETX_FullDuplex_MASK                    BIT_SET_MASK(8)
# define A_NEG_ADVT_PARTNER_100BASETX_HalfDuplex_MASK                    BIT_SET_MASK(7)
# define A_NEG_ADVT_PARTNER_10BASET_FullDuplex_MASK                  BIT_SET_MASK(6)
# define A_NEG_ADVT_PARTNER_10BASET_HalfDuplex_MASK                  BIT_SET_MASK(5)
# define A_NEG_ADVT_PARTNER_Selector_Field_MASK(std)                  BIT_MASK(std,0)

# define A_NEG_EXP_Reserved_MASK                 BIT_SET_MASK()
# define A_NEG_EXP_Parallel_Detection_Fault_MASK                 BIT_SET_MASK(4)
# define A_NEG_EXP_Link_Partner_Next_Page_Able_MASK                  BIT_SET_MASK(3)
# define A_NEG_EXP_Next_Page_Able_MASK                   BIT_SET_MASK(2)
# define A_NEG_EXP_Page_Received_MASK                    BIT_SET_MASK(1)
# define A_NEG_EXP_Link_Partner_AutoNegotiation_Able_MASK                    BIT_SET_MASK(0)

# define A_NEG_NXT_PG_Next_Page_MASK                 BIT_SET_MASK(15)
# define A_NEG_NXT_PG_Reserved_MASK                  BIT_SET_MASK(14)
# define A_NEG_NXT_PG_Message_Page_MASK                  BIT_SET_MASK(13)
# define A_NEG_NXT_PG_Acknowledge2_MASK                  BIT_SET_MASK(12)
# define A_NEG_NXT_PG_Toggle_MASK                    BIT_SET_MASK(11)
# define A_NEG_NXT_PG_Message_Field_MASK(msg)                 BIT_MASK(msg,0)

# define A_NEG_NXT_PG_PARTNER_Next_Page_MASK                 BIT_SET_MASK(15)
# define A_NEG_NXT_PG_PARTNER_Acknowledge_MASK                  BIT_SET_MASK(14)
# define A_NEG_NXT_PG_PARTNER_Message_Page_MASK                  BIT_SET_MASK(13)
# define A_NEG_NXT_PG_PARTNER_Acknowledge2_MASK                  BIT_SET_MASK(12)
# define A_NEG_NXT_PG_PARTNER_Toggle_MASK                    BIT_SET_MASK(11)
# define A_NEG_NXT_PG_PARTNER_Message_Field_MASK(msg)                 BIT_MASK(msg,0)

# define MII_CTRL_Reserved_MASK                  BIT_MASK(0x00,8)
# define MII_CTRL_100BASETX_Preamble_Restore_MASK                    BIT_SET_MASK(7)
# define MII_CTRL_10BASET_Preamble_Restore_MASK                  BIT_SET_MASK(6)
# define MII_CTRL_Reserved_MASK                  BIT_MASK(0x01,0)

# define RXER_Counter_MASK(count)                   BIT_MASK(count,0)

# define INT_CTRL_Jabber_Interrupt_Enable_MASK                   BIT_SET_MASK(15)
# define INT_CTRL_Receive_Error_Interrupt_Enable_MASK                    BIT_SET_MASK(14)
# define INT_CTRL_Page_Received_Interrupt_Enable_MASK                    BIT_SET_MASK(13)
# define INT_CTRL_Parallel_Detect_Fault_Interrupt_Enable_MASK                    BIT_SET_MASK(12)
# define INT_CTRL_Link_Partner_Acknowledge_Interrupt_Enable_MASK                 BIT_SET_MASK(11)
# define INT_CTRL_Link_Down_Interrupt_Enable_MASK                    BIT_SET_MASK(10)
# define INT_CTRL_Remote_Fault_Interrupt_Enable_MASK                 BIT_SET_MASK(9)
# define INT_CTRL_Link_Up_Interrupt_Enable_MASK                  BIT_SET_MASK(8)
# define INT_STATUS_Jabber_Interrupt_MASK                    BIT_SET_MASK(7)
# define INT_STATUS_Receive_Error_Interrupt_MASK                 BIT_SET_MASK(6)
# define INT_STATUS_Page_Receive_Interrupt_MASK                  BIT_SET_MASK(5)
# define INT_STATUS_Parallel_Detect_FaultInterrupt_MASK                  BIT_SET_MASK(4)
# define INT_STATUS_Link_Partner_Acknowledge_Interrupt_MASK                  BIT_SET_MASK(3)
# define INT_STATUS_Link_Down_Interrupt_MASK                 BIT_SET_MASK(2)
# define INT_STATUS_Remote_Fault_Interrupt_MASK                  BIT_SET_MASK(1)
# define INT_STATUS_Link_Up_Interrupt_MASK                   BIT_SET_MASK(0)

# define PHY_CTRL1_LED_mode_MASK(led)                 BIT_MASK(led, 13)
# define PHY_CTRL1_Polarity_MASK                 BIT_SET_MASK(13)
# define PHY_CTRL1_Reserved_MASK                 BIT_SET_MASK(12)
# define PHY_CTRL1_MDI_MDIX_State_MASK                   BIT_SET_MASK(11)
# define PHY_CTRL1_Reserved_MASK                 BIT_MASK(0x0,8)
# define PHY_CTRL1_Remote_loopback_MASK                  BIT_SET_MASK(7)
# define PHY_CTRL1_Reserved_MASK                 BIT_MASK(0x00,0)

# define PHY_CTRL2_HP_MDIX_MASK                  BIT_SET_MASK(15)
# define PHY_CTRL2_MDI_MDIX_Select_MASK                  BIT_SET_MASK(14)
# define PHY_CTRL2_Pair_Swap_Disable_MASK                    BIT_SET_MASK(13)
# define PHY_CTRL2_Energy_Detect_MASK                    BIT_SET_MASK(12)
# define PHY_CTRL2_Force_Link_MASK                   BIT_SET_MASK(11)
# define PHY_CTRL2_Power_Saving_MASK                 BIT_SET_MASK(10)
# define PHY_CTRL2_Interrupt_Level_MASK                  BIT_SET_MASK(9)
# define PHY_CTRL2_Enable_Jabber_MASK                    BIT_SET_MASK(8)
# define PHY_CTRL2_AutoNegotiation_Complete_MASK                 BIT_SET_MASK(7)
# define PHY_CTRL2_Enable_Pause_MASK                 BIT_SET_MASK(6)
# define PHY_CTRL2_PHY_Isolate_MASK                  BIT_SET_MASK(5)
# define PHY_CTRL2_Operation_Mode_Indication_MASK(op)                    BIT_MASK(op, 2)
# define PHY_CTRL2_Enable_SQE_test_MASK                  BIT_SET_MASK(1)
# define PHY_CTRL2_Disable_Data_Scrambling_MASK                  BIT_SET_MASK(0)


typedef struct  __attribute__((__packed__))
{
	uint16_t bc_reg;
	uint16_t bs_reg;
	uint16_t phy_id1_reg;
	uint16_t phy_id2_reg;
	uint16_t a_neg_advt_reg ;
	uint16_t a_neg_advt_partner_reg;
	uint16_t a_neg_exp_reg;
	uint16_t a_neg_nxt_pg_reg;
	uint16_t a_neg_nxt_pg_partner_reg;
	uint16_t mii_ctrl_reg;
	uint16_t rxer_counter_reg;
	uint16_t int_reg;
	uint16_t phy_ctrl1_reg;
	uint16_t phy_ctrl2_reg;
} ENET_PHY_t;

typedef struct  __attribute__((__packed__)){
    uint16_t status;    /* control and status */
    uint16_t length;    /* transfer length */
    uint16_t  buf_addr[150]; /* buffer address NOTE Big Endian!!!*/
} enet_descr_t;
static __inline uint16_t end_bswap16(uint16_t __x)
{
        return (__x<<8) | (__x>>8);
}
