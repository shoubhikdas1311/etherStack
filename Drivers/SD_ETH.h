/*
 * SD_ETH.h
 *
 *  Created on: JAN 03, 2020
 *      Author: SD
 */
#include "SD_Clk.h"
#include "ETH_PHY.h"
#include "../common/common.h"

#ifndef SD_ETH_H_
#define SD_ETH_H_

#pragma anon_unions

#define GETBYTE(num, byte)              ((*((int *)num) & (0xFF << (byte*8))) >> (byte*8))
#define SETBYTE(byte_data, byte)        (*((char *)byte_data) << (byte*8))

#define ICMP_MASK		1
#define IGMP_MASK		2
#define TCP_MASK		6
#define UDP_MASK		17

#define ETH_ADDR_LEN    (6)
#define ETH_IP4ADDR_LEN	4

#define ENET_Transmit_IRQn			76
#define ENET_Receive_IRQn			77

#define ENET_MII_MODE_MASK			'M'		//0x4D
#define ENET_RMII_MODE_MASK			'R'		//0x52

/* MII寄存器地址 */
#define PHY_BMCR                    (0x00) /* Basic Control */
#define PHY_BMSR                    (0x01) /* Basic Status */
#define PHY_PHYIDR1                 (0x02) /* PHY Identifer 1 */
#define PHY_PHYIDR2                 (0x03) /* PHY Identifer 2 */
#define PHY_ANAR                    (0x04) /* Auto-Negotiation Advertisement */
#define PHY_ANLPAR                  (0x05) /* Auto-Negotiation Link Partner Ability */
#define PHY_ANER                    (0x06) /* Auto-Negotiation Expansion */
#define PHY_LPNPA                   (0x07) /* Link Partner Next Page Ability */
#define PHY_RXERC                   (0x15) /* RXER Counter */
#define PHY_ICS                     (0x1B) /* Interrupt Control/Status */
#define PHY_PHYC1                   (0x1E) /* PHY Control 1 */
#define PHY_PHYC2                   (0x1F) /* PHY Control 2 */     
     
     
    


/* PHY_BMCR寄存器位定义 */
#define PHY_BMCR_RESET              (0x8000)
#define PHY_BMCR_LOOP               (0x4000)
#define PHY_BMCR_SPEED              (0x2000)
#define PHY_BMCR_AN_ENABLE          (0x1000)
#define PHY_BMCR_POWERDOWN          (0x0800)
#define PHY_BMCR_ISOLATE            (0x0400)
#define PHY_BMCR_AN_RESTART         (0x0200)
#define PHY_BMCR_FDX                (0x0100)
#define PHY_BMCR_COL_TEST           (0x0080)

/* PHY_BMSR寄存器位定义 */
#define PHY_BMSR_100BT4             (0x8000)
#define PHY_BMSR_100BTX_FDX         (0x4000)
#define PHY_BMSR_100BTX             (0x2000)
#define PHY_BMSR_10BT_FDX           (0x1000)
#define PHY_BMSR_10BT               (0x0800)
#define PHY_BMSR_NO_PREAMBLE        (0x0040)
#define PHY_BMSR_AN_COMPLETE        (0x0020)
#define PHY_BMSR_REMOTE_FAULT       (0x0010)
#define PHY_BMSR_AN_ABILITY         (0x0008)
#define PHY_BMSR_LINK               (0x0004)
#define PHY_BMSR_JABBER             (0x0002)
#define PHY_BMSR_EXTENDED           (0x0001)

/* PHY_ANAR寄存器位定义 */
#define PHY_ANAR_NEXT_PAGE          (0x8001)
#define PHY_ANAR_REM_FAULT          (0x2001)
#define PHY_ANAR_PAUSE              (0x0401)
#define PHY_ANAR_100BT4             (0x0201)
#define PHY_ANAR_100BTX_FDX         (0x0101)
#define PHY_ANAR_100BTX             (0x0081)
#define PHY_ANAR_10BT_FDX           (0x0041)
#define PHY_ANAR_10BT               (0x0021)
#define PHY_ANAR_802_3              (0x0001)

/* PHY_ANLPAR寄存器位定义 */
#define PHY_ANLPAR_NEXT_PAGE        (0x8000)
#define PHY_ANLPAR_ACK              (0x4000)
#define PHY_ANLPAR_REM_FAULT        (0x2000)
#define PHY_ANLPAR_PAUSE            (0x0400)
#define PHY_ANLPAR_100BT4           (0x0200)
#define PHY_ANLPAR_100BTX_FDX       (0x0100)
#define PHY_ANLPAR_100BTX           (0x0080)
#define PHY_ANLPAR_10BTX_FDX        (0x0040)
#define PHY_ANLPAR_10BT             (0x0020)


/* PHY_PHYSTS寄存器位定义 */
#define PHY_PHYSTS_MDIXMODE         (0x4000)
#define PHY_PHYSTS_RX_ERR_LATCH     (0x2000)
#define PHY_PHYSTS_POL_STATUS       (0x1000)
#define PHY_PHYSTS_FALSECARRSENSLAT (0x0800)
#define PHY_PHYSTS_SIGNALDETECT     (0x0400)
#define PHY_PHYSTS_PAGERECEIVED     (0x0100)
#define PHY_PHYSTS_MIIINTERRUPT     (0x0080)
#define PHY_PHYSTS_REMOTEFAULT      (0x0040)
#define PHY_PHYSTS_JABBERDETECT     (0x0020)
#define PHY_PHYSTS_AUTONEGCOMPLETE  (0x0010)
#define PHY_PHYSTS_LOOPBACKSTATUS   (0x0008)
#define PHY_PHYSTS_DUPLEXSTATUS     (0x0004)
#define PHY_PHYSTS_SPEEDSTATUS      (0x0002)
#define PHY_PHYSTS_LINKSTATUS       (0x0001)

/* PHY硬件特性 */
#define PHY_STATUS                  ( 0x1F )
#define PHY_DUPLEX_STATUS           ( 4<<2 )
#define PHY_SPEED_STATUS            ( 1<<2 )
/* PHY收发器硬件地址 */
#define CFG_PHY_ADDRESS             (0x01)
     
//Freescale处理器相关定义

/* TX缓冲区描述符位定义 */
#define TX_BD_R			0x0080
#define TX_BD_TO1		0x0040
#define TX_BD_W			0x0020
#define TX_BD_TO2		0x0010
#define TX_BD_L			0x0008
#define TX_BD_TC		0x0004
#define TX_BD_ABC		0x0002

/* TX增强型缓冲区描述符位定义 */
#define TX_BD_INT       0x00000040 
#define TX_BD_TS        0x00000020 
#define TX_BD_PINS      0x00000010 
#define TX_BD_IINS      0x00000008 
#define TX_BD_TXE       0x00800000 
#define TX_BD_UE        0x00200000 
#define TX_BD_EE        0x00100000
#define TX_BD_FE        0x00080000 
#define TX_BD_LCE       0x00040000 
#define TX_BD_OE        0x00020000 
#define TX_BD_TSE       0x00010000 

#define TX_BD_BDU       0x00000080    

/* RX缓冲区描述符位定义 */
// 0偏移标志 - 状态:大端格式
#define RX_BD_E			0x0080
#define RX_BD_R01		0x0040
#define RX_BD_W			0x0020
#define RX_BD_R02		0x0010
#define RX_BD_L			0x0008
#define RX_BD_M			0x0001
#define RX_BD_BC		0x8000
#define RX_BD_MC		0x4000
#define RX_BD_LG		0x2000
#define RX_BD_NO		0x1000
#define RX_BD_CR		0x0400
#define RX_BD_OV		0x0200
#define RX_BD_TR		0x0100

/* RX增强型缓冲区描述符位定义 */
#define RX_BD_ME               0x00000080    
#define RX_BD_PE               0x00000004    
#define RX_BD_CE               0x00000002    
#define RX_BD_UC               0x00000001
    
#define RX_BD_INT              0x00008000    

#define RX_BD_ICE              0x20000000    
#define RX_BD_PCR              0x10000000    
#define RX_BD_VLAN             0x04000000    
#define RX_BD_IPV6             0x02000000    
#define RX_BD_FRAG             0x01000000    

#define RX_BD_BDU              0x00000080   

/* MII接口超时 */
#define MII_TIMEOUT		0x1FFFF

/* 以太帧相关定义 */
#define CFG_NUM_ENET_TX_BUFFERS     1     //发送缓冲区个数
#define CFG_NUM_ENET_RX_BUFFERS     1     //接收缓冲区个数  驱动程序设定必须为1了 改了就会出错
#define CFG_ENET_BUFFER_SIZE        1520    //以太发送帧缓冲区长度
#define CFG_ENET_MAX_PACKET_SIZE    1520    //以太发最大数据包长度

#define LIB_TRACE					printf
#define kBusClock					50000
#define IP_HEAD_LEN					20
#define RAWLEN_8_t(data_len)		(data_len + IP_HEAD_LEN)
#define RAWLEN_16_t(data_len)		(data_len + IP_HEAD_LEN)/2

///* 缓冲区描述符结构体 *//*
//  PDD layer implementation for peripheral type ENET
//  (C) 2013 Freescale, Inc. All rights reserved.

//  This file is static and it is generated from API-Factory
//*/

//#if !defined(ENET_PDD_H_)
//#define ENET_PDD_H_

///* ----------------------------------------------------------------------------
//   -- Test if supported MCU is active
//   ---------------------------------------------------------------------------- */

//#if !defined(MCU_ACTIVE)
//  // No MCU is active
//  #error ENET PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
//#elif \
//      !defined(MCU_MK52D10) /* ENET */ && \
//      !defined(MCU_MK52DZ10) /* ENET */ && \
//      !defined(MCU_MK53D10) /* ENET */ && \
//      !defined(MCU_MK53DZ10) /* ENET */ && \
//      !defined(MCU_MK60D10) /* ENET */ && \
//      !defined(MCU_MK60F12) /* ENET */ && \
//      !defined(MCU_MK60F15) /* ENET */ && \
//      !defined(MCU_MK60DZ10) /* ENET */ && \
//      !defined(MCU_MK60N512VMD100) /* ENET */ && \
//      !defined(MCU_MK61F12) /* ENET */ && \
//      !defined(MCU_MK61F15) /* ENET */ && \
//      !defined(MCU_MK61F12WS) /* ENET */ && \
//      !defined(MCU_MK61F15WS) /* ENET */ && \
//      !defined(MCU_MK63F12) /* ENET */ && \
//      !defined(MCU_MK63F12WS) /* ENET */ && \
//      !defined(MCU_MK64F12) /* ENET */ && \
//      !defined(MCU_MK65F18) /* ENET */ && \
//      !defined(MCU_MK65F18WS) /* ENET */ && \
//      !defined(MCU_MK66F18) /* ENET */ && \
//      !defined(MCU_MK70F12) /* ENET */ && \
//      !defined(MCU_MK70F15) /* ENET */ && \
//      !defined(MCU_MK70F12WS) /* ENET */ && \
//      !defined(MCU_MK70F15WS) /* ENET */
//  // Unsupported MCU is active
//  #error ENET PDD library: Unsupported derivative is active.
//#endif

////#include "PDD_Types.h"

///* ----------------------------------------------------------------------------
//   -- Method symbol definitions
//   ---------------------------------------------------------------------------- */

///* Interrupt masks */
//#define ENET_PDD_BABBLING_RX_ERROR_INT      ENET_EIR_BABR_MASK /**< Babbling receive error interrupt. */
//#define ENET_PDD_BABBLING_TX_ERROR_INT      ENET_EIR_BABT_MASK /**< Babbling transmit error interrupt. */
//#define ENET_PDD_GRACEFUL_STOP_COMPLETE_INT ENET_EIR_GRA_MASK /**< Graceful stop complete interrupt. */
//#define ENET_PDD_TX_FRAME_INT               ENET_EIR_TXF_MASK /**< Transmit frame interrupt. */
//#define ENET_PDD_TX_BUFFER_INT              ENET_EIR_TXB_MASK /**< Transmit buffer interrupt. */
//#define ENET_PDD_RX_FRAME_INT               ENET_EIR_RXF_MASK /**< Receive frame interrupt. */
//#define ENET_PDD_RX_BUFFER_INT              ENET_EIR_RXB_MASK /**< Receive buffer interrupt. */
//#define ENET_PDD_MII_INT                    ENET_EIR_MII_MASK /**< MII interrupt. */
//#define ENET_PDD_ETHERNET_BUS_ERROR_INT     ENET_EIR_EBERR_MASK /**< Ethernet bus error interrupt. */
//#define ENET_PDD_LATE_COLLISION_INT         ENET_EIR_LC_MASK /**< Late collision interrupt. */
//#define ENET_PDD_COLLISION_RETRY_LIMIT_INT  ENET_EIR_RL_MASK /**< Collision retry limit interrupt. */
//#define ENET_PDD_TX_FIFO_UNDERRUN_INT       ENET_EIR_UN_MASK /**< Transmit FIFO underrun interrupt. */
//#define ENET_PDD_PAYLOAD_RX_ERROR_INT       ENET_EIR_PLR_MASK /**< Payload receive error interrupt. */
//#define ENET_PDD_WAKEUP_INT                 ENET_EIR_WAKEUP_MASK /**< Node wake-up request indication interrupt. */
//#define ENET_PDD_TX_TIMESTAMP_AVAIL_INT     ENET_EIR_TS_AVAIL_MASK /**< Transmit timestamp available interrupt. */
//#define ENET_PDD_TIMER_PERIOD_INT           ENET_EIR_TS_TIMER_MASK /**< Timestamp timer interrupt. */

///* Filter modes */
//#define ENET_PDD_NONE                         0U /**< No filtering. */
//#define ENET_PDD_UNICAST_AND_MULTICAST_FILTER ENET_RCR_PROM_MASK /**< Unicast and multicast address filtering. */
//#define ENET_PDD_BROADCAST_REJECT             ENET_RCR_BC_REJ_MASK /**< Broadcast frames rejection. */

///* Message Information Block counter indices */
//#define ENET_PDD_RMON_T_DROP        0U           /**< Count of frames not counted correctly. */
//#define ENET_PDD_RMON_T_PACKETS     0x1U         /**< RMON Tx packet count. */
//#define ENET_PDD_RMON_T_BC_PKT      0x2U         /**< RMON Tx Broadcast Packets. */
//#define ENET_PDD_RMON_T_MC_PKT      0x3U         /**< RMON Tx Multicast Packets. */
//#define ENET_PDD_RMON_T_CRC_ALIGN   0x4U         /**< RMON Tx Packets w CRC/Align error. */
//#define ENET_PDD_RMON_T_UNDERSIZE   0x5U         /**< RMON Tx Packets < 64 bytes, good crc. */
//#define ENET_PDD_RMON_T_OVERSIZE    0x6U         /**< RMON Tx Packets > MAX_FL bytes, good crc. */
//#define ENET_PDD_RMON_T_FRAG        0x7U         /**< RMON Tx Packets < 64 bytes, bad crc. */
//#define ENET_PDD_RMON_T_JAB         0x8U         /**< RMON Tx Packets > MAX_FL bytes, bad crc. */
//#define ENET_PDD_RMON_T_COL         0x9U         /**< RMON Tx collision count. */
//#define ENET_PDD_RMON_T_P64         0xAU         /**< RMON Tx 64 byte packets. */
//#define ENET_PDD_RMON_T_P65TO127    0xBU         /**< RMON Tx 65 to 127 byte packets. */
//#define ENET_PDD_RMON_T_P128TO255   0xCU         /**< RMON Tx 128 to 255 byte packets. */
//#define ENET_PDD_RMON_T_P256TO511   0xDU         /**< RMON Tx 256 to 511 byte packets. */
//#define ENET_PDD_RMON_T_P512TO1023  0xEU         /**< RMON Tx 512 to 1023 byte packets. */
//#define ENET_PDD_RMON_T_P1024TO2047 0xFU         /**< RMON Tx 1024 to 2047 byte packets. */
//#define ENET_PDD_RMON_T_P_GTE2048   0x10U        /**< RMON Tx packets w > 2048 bytes. */
//#define ENET_PDD_RMON_T_OCTETS      0x11U        /**< RMON Tx Octets. */
//#define ENET_PDD_IEEE_T_DROP        0x12U        /**< Count of frames not counted correctly. */
//#define ENET_PDD_IEEE_T_FRAME_OK    0x13U        /**< Frames Transmitted OK. */
//#define ENET_PDD_IEEE_T_1COL        0x14U        /**< Frames Transmitted with Single Collision. */
//#define ENET_PDD_IEEE_T_MCOL        0x15U        /**< Frames Transmitted with Multiple Collisions. */
//#define ENET_PDD_IEEE_T_DEF         0x16U        /**< Frames Transmitted after Deferral Delay. */
//#define ENET_PDD_IEEE_T_LCOL        0x17U        /**< Frames Transmitted with Late Collision. */
//#define ENET_PDD_IEEE_T_EXCOL       0x18U        /**< Frames Transmitted with Excessive Collisions. */
//#define ENET_PDD_IEEE_T_MACERR      0x19U        /**< Frames Transmitted with Tx FIFO Underrun. */
//#define ENET_PDD_IEEE_T_CSERR       0x1AU        /**< Frames Transmitted with Carrier Sense Error. */
//#define ENET_PDD_IEEE_T_SQE         0x1BU        /**< Frames Transmitted with SQE Error. */
//#define ENET_PDD_IEEE_T_FDXFC       0x1CU        /**< Flow Control Pause frames transmitted. */
//#define ENET_PDD_IEEE_T_OCTETS_OK   0x1DU        /**< Octet count for Frames Transmitted w/o Error. */
//#define ENET_PDD_RMON_R_DROP        0x20U        /**< Count of frames not counted correctly. */
//#define ENET_PDD_RMON_R_PACKETS     0x21U        /**< RMON Rx packet count. */
//#define ENET_PDD_RMON_R_BC_PKT      0x22U        /**< RMON Rx Broadcast Packets. */
//#define ENET_PDD_RMON_R_MC_PKT      0x23U        /**< RMON Rx Multicast Packets. */
//#define ENET_PDD_RMON_R_CRC_ALIGN   0x24U        /**< RMON Rx Packets w CRC/Align error. */
//#define ENET_PDD_RMON_R_UNDERSIZE   0x25U        /**< RMON Rx Packets < 64 bytes, good crc. */
//#define ENET_PDD_RMON_R_OVERSIZE    0x26U        /**< RMON Rx Packets > MAX_FL bytes, good crc. */
//#define ENET_PDD_RMON_R_FRAG        0x27U        /**< RMON Rx Packets < 64 bytes, bad crc. */
//#define ENET_PDD_RMON_R_JAB         0x28U        /**< RMON Rx Packets > MAX_FL bytes, bad crc. */
//#define ENET_PDD_RMON_R_RESVD_0     0x29U        /**< Reserved. */
//#define ENET_PDD_RMON_R_P64         0x2AU        /**< RMON Rx 64 byte packets. */
//#define ENET_PDD_RMON_R_P65TO127    0x2BU        /**< RMON Rx 65 to 127 byte packets. */
//#define ENET_PDD_RMON_R_P128TO255   0x2CU        /**< RMON Rx 128 to 255 byte packets. */
//#define ENET_PDD_RMON_R_P256TO511   0x2DU        /**< RMON Rx 256 to 511 byte packets. */
//#define ENET_PDD_RMON_R_P512TO1023  0x2EU        /**< RMON Rx 512 to 1023 byte packets. */
//#define ENET_PDD_RMON_R_P1024TO2047 0x2FU        /**< RMON Rx 1024 to 2047 byte packets. */
//#define ENET_PDD_RMON_R_P_GTE2048   0x30U        /**< RMON Rx packets w > 2048 bytes. */
//#define ENET_PDD_RMON_R_OCTETS      0x31U        /**< RMON Rx Octets. */
//#define ENET_PDD_IEEE_R_DROP        0x32U        /**< Count of frames not counted correctly. */
//#define ENET_PDD_IEEE_R_FRAME_OK    0x33U        /**< Frames Received OK. */
//#define ENET_PDD_IEEE_R_CRC         0x34U        /**< Frames Received with CRC Error. */
//#define ENET_PDD_IEEE_R_ALIGN       0x35U        /**< Frames Received with Alignment Error. */
//#define ENET_PDD_IEEE_R_MACERR      0x36U        /**< Receive Fifo Overflow count. */
//#define ENET_PDD_IEEE_R_FDXFC       0x37U        /**< Flow Control Pause frames received. */
//#define ENET_PDD_IEEE_R_OCTETS_OK   0x38U        /**< Octet count for Frames Rcvd w/o Error  */

///* Control frame types */
//#define ENET_PDD_PAUSE     ENET_RCR_PAUFWD_MASK  /**< Pause frame. */
//#define ENET_PDD_NON_PAUSE ENET_RCR_CFEN_MASK    /**< Control frame other than pause frame. */

///* Received frame processing types */
//#define ENET_PDD_CRC_REMOVE     ENET_RCR_CRCFWD_MASK /**< Frame CRC removal. */
//#define ENET_PDD_PADDING_REMOVE ENET_RCR_PADEN_MASK /**< Frame header padding removal. */

///* Transmit frame processing types */
//#define ENET_PDD_CRC_INSERT ENET_TCR_CRCFWD_MASK /**< Frame CRC insertion. */

///* Protocol checksum accelerators */
//#define ENET_PDD_INSERT_ON_TX 0x1U               /**< Insert checksum into frame on transmission. */
//#define ENET_PDD_DROP_ON_RX   0x2U               /**< Drop frames with wrong checksum on reception. */

///* FIFO shift types */
//#define ENET_PDD_TX_ONLY   0x1U                  /**< On transmission only. */
//#define ENET_PDD_RX_ONLY   0x2U                  /**< On reception only. */
//#define ENET_PDD_TX_AND_RX 0x3U                  /**< On transmission and reception. */

///* Timer status flags */
//#define ENET_PDD_TIMER_CHANNEL_0 ENET_TGSR_TF0_MASK /**< Timer channel 0. */
//#define ENET_PDD_TIMER_CHANNEL_1 ENET_TGSR_TF1_MASK /**< Timer channel 1. */
//#define ENET_PDD_TIMER_CHANNEL_2 ENET_TGSR_TF2_MASK /**< Timer channel 2. */
//#define ENET_PDD_TIMER_CHANNEL_3 ENET_TGSR_TF3_MASK /**< Timer channel 3. */

///* Timer clock sources */
//#define ENET_PDD_CORE_SYSTEM_CLOCK 0U            /**< Core/system clock. */
//#define ENET_PDD_PLL_FLL_CLOCK     0x1U          /**< PLL or FLL clock. */
//#define ENET_PDD_EXTAL_CLOCK       0x2U          /**< Extal clock. */
//#define ENET_PDD_EXTERNAL_CLOCK    0x3U          /**< External clock. */

///* MII management frame operation code */
//#define ENET_PDD_MII_WRITE 0x10000000U           /**< Write MII register. */
//#define ENET_PDD_MII_READ  0x20000000U           /**< Read MII register. */

///* Duplex modes */
//#define ENET_PDD_FULL_DUPLEX 0x4U                /**< Full duplex. */
//#define ENET_PDD_HALF_DUPLEX 0U                  /**< Half duplex. */

///* Power saving modes */
//#define ENET_PDD_SLEEP 0U                        /**< Sleep mode. */
//#define ENET_PDD_STOP  0x1U                      /**< Stop mode. */
//#define ENET_PDD_RUN   0x2U                      /**< Run mode. */

///* MII modes */
//#define ENET_PDD_MII          0U                 /**< MII mode. */
//#define ENET_PDD_RMII_10MBIT  0x1U               /**< 10MBit RMII mode. */
//#define ENET_PDD_RMII_100MBIT 0x2U               /**< 100MBit RMII mode. */

///* MAC address sources */
//#define ENET_PDD_KEEP        0U                  /**< Keep frame MAC address. */
//#define ENET_PDD_AUTO        0x1U                /**< Auto-complete frame MAC address. */
//#define ENET_PDD_AUTO_SUPPL0 0x2U                /**< Auto-complete supplemental MAC address 0. */
//#define ENET_PDD_AUTO_SUPPL1 0x3U                /**< Auto-complete supplemental MAC address 1. */
//#define ENET_PDD_AUTO_SUPPL2 0x4U                /**< Auto-complete supplemental MAC address 2. */
//#define ENET_PDD_AUTO_SUPPL3 0x5U                /**< Auto-complete supplemental MAC address 3. */

///* Timer modes */
//#define ENET_PDD_DISABLED                                        0U /**< Disabled. */
//#define ENET_PDD_INPUT_CAPTURE_RISING_EDGE                       0x4U /**< Input capture on rising edge. */
//#define ENET_PDD_INPUT_CAPTURE_FALLING_EDGE                      0x8U /**< Input capture on falling edge. */
//#define ENET_PDD_INPUT_CAPTURE_BOTH_EDGES                        0xCU /**< Input capture on both edges. */
//#define ENET_PDD_OUTPUT_COMPARE_SOFTWARE_ONLY                    0x10U /**< Output compare - software only. */
//#define ENET_PDD_OUTPUT_COMPARE_TOGGLE_ON_COMPARE                0x14U /**< Output compare - toggle on compare. */
//#define ENET_PDD_OUTPUT_COMPARE_CLEAR_ON_COMPARE                 0x18U /**< Output compare - clear on compare. */
//#define ENET_PDD_OUTPUT_COMPARE_SET_ON_COMPARE                   0x1CU /**< Output compare - set on compare. */
//#define ENET_PDD_OUTPUT_COMPARE_CLEAR_ON_COMPARE_SET_ON_OVERFLOW 0x28U /**< Output compare - clear on compare, set on overflow. */
//#define ENET_PDD_OUTPUT_COMPARE_SET_ON_COMPARE_CLEAR_ON_OVERFLOW 0x24U /**< Output compare - set on compare, clear on overflow. */
//#define ENET_PDD_OUTPUT_COMPARE_PULSE_LOW                        0x38U /**< Output compare - pulse low. */
//#define ENET_PDD_OUTPUT_COMPARE_PULSE_HIGH                       0x3CU /**< Output compare - pulse high. */


//typedef struct ipv4_packet_s {
//    uint8_t     ver_ihl;        /* 0-3 version (4), 4-7 header len (5 Min)*/
//    uint8_t     dscp_ecn;       /* 0-5 diffentiated services,6-7 congestion */
//    uint16_t    len;            /* Total length - min 20bytes, max 65K */
//    uint16_t    id;             /* fragement ID */
//    uint16_t    flg_off;        /* 0-2 flags, 3-15 fragment offset */
//    uint8_t     ttl;            /* time to live counter */
//    uint8_t     protocol;       /* Datagram protocol number */
//    uint16_t    header_csum;    /* Header checksum */
//    uint8_t     spa[ETH_IP4ADDR_LEN]; /* source IPV4 Address */
//    uint8_t     tpa[ETH_IP4ADDR_LEN]; /* target IPV4 Address */
//    uint8_t*    opt_or_payload; /* Either payload (len=5) or options (rare) */
//} ipv4_packet_t;
//!< 以太网初始化结构
typedef struct
{
    uint8_t* pMacAddress;
}ENET_InitTypeDef;
     
typedef enum
{
    kENET_IT_TXF_Disable,   //!< 禁止发送一帧后产生中断     
    kENET_IT_RXF_Disable,   //!< 禁止接收一帧后产生中断    
    kENET_IT_TXF,           //!< 开启ENET发送一帧中断    
    kENET_IT_RXF,           //!< 开启ENET接收一帧中断
}ENET_ITDMAConfig_Type;

/* Ethernet Frame Header definition */
typedef union  __attribute__((__packed__))
	{
	struct __attribute__((__packed__)){
    uint8_t    dest[ETH_ADDR_LEN];
    uint8_t    src[ETH_ADDR_LEN];
    uint16_t   type;
	};
	uint8_t rawBytes[14];
} SD_eth_header_t;
/* Defined Ethernet Types */
#define ETHTYPE_IPV4    (0x0800)        /* IP V4 RFC-894 (RFC-791) */
#define ETHTYPE_ARP     (0x0806)        /* ARP RFC-826 */
#define ETHTYPE_IPV6    (0x86DD)        /* IP V6 RFC-2464 */


//!< ENET CallBack Type
typedef void (*ENET_CallBackTxType)(void);
typedef void (*ENET_CallBackRxType)(void);

//!< API functions
void ENET_Init(uint8_t* mac);
void ENET_MacSendData(uint16_t *data, uint16_t len);
uint16_t ENET_MacReceiveData(uint8_t *data);
void ENET_ITDMAConfig(ENET_ITDMAConfig_Type config);
void ENET_CallbackTxInstall(ENET_CallBackTxType AppCBFun);
void ENET_CallbackRxInstall(ENET_CallBackRxType AppCBFun);
uint32_t ENET_IsTransmitComplete(void);
int8_t ENET_pin_init(uint8_t mode);
unsigned short ipv4_check(uint16_t *buf, unsigned size);
uint16_t EnetRxCB();
uint8_t EnetTxCB();
uint16_t SD_ETH_write_IP_Packet(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest);

#endif /* SYS_H_ */