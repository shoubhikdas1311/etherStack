/*******************************************************************************
*
* eth_frame.h
*
* Steve Holford
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#if !defined(ETH_FRAME_H)
#define ETH_FRAME_H
#include <stdint.h>
#include "MK60N512.h"
#include "../common/common.h"
#include <stdlib.h>
#include <string.h>
#include "err.h"
//#include "globalDefs.h"

#pragma anon_unions

#define DEV_MAJ_ENET					36
#define HEADER_LEN_MASK					5
#define DATA_LEN_MASK					5

#define GETBYTE(num, byte)              ((*((int *)num) & (0xFF << (byte*8))) >> (byte*8))
#define SETBYTE(byte_data, byte)        (*((char *)byte_data) << (byte*8))

#define ICMP_MASK		1
#define IGMP_MASK		2
#define TCP_MASK		6
#define UDP_MASK		17

#define ENET_MII_MODE_MASK			'M'		//0x4D
#define ENET_RMII_MODE_MASK			'R'		//0x52
	
#define iprintf			printf
#define MII_TIMEOUT         0x1FFFF     /* Timeout when talking MII to PHY */
#define MII_LINK_TIMEOUT    0x1FFFF     /* Timeout when resetting PHY */

/* MII Register Addresses */
#define PHY_BMCR                    (0x00)
#define PHY_BMSR                    (0x01)
#define PHY_PHYIDR1                 (0x02)
#define PHY_PHYIDR2                 (0x03)
#define PHY_ANAR                    (0x04)
#define PHY_ANLPAR                  (0x05)
#define PHY_ANLPARNP                (0x05)
#define PHY_ANER                    (0x06)
#define PHY_ANNPTR                  (0x07)
#define PHY_PHYSTS                  (0x10)
#define PHY_MICR                    (0x11)
#define PHY_MISR                    (0x12)
#define PHY_PAGESEL                 (0x13)

/*TWR definition: Micrel*/
#define PHY_PHYCTRL1                (0x1E)
#define PHY_PHYCTRL2                (0x1F)

/* Bit definitions and macros for PHY_BMCR */
#define PHY_BMCR_RESET              (0x8000)
#define PHY_BMCR_LOOP               (0x4000)
#define PHY_BMCR_SPEED              (0x2000)
#define PHY_BMCR_AN_ENABLE          (0x1000)
#define PHY_BMCR_POWERDOWN          (0x0800)
#define PHY_BMCR_ISOLATE            (0x0400)
#define PHY_BMCR_AN_RESTART         (0x0200)
#define PHY_BMCR_FDX                (0x0100)
#define PHY_BMCR_COL_TEST           (0x0080)

/* Bit definitions and macros for PHY_BMSR */
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

/* Bit definitions and macros for PHY_ANAR */
#define PHY_ANAR_NEXT_PAGE          (0x8001)
#define PHY_ANAR_REM_FAULT          (0x2001)
#define PHY_ANAR_PAUSE              (0x0401)
#define PHY_ANAR_100BT4             (0x0201)
#define PHY_ANAR_100BTX_FDX         (0x0101)
#define PHY_ANAR_100BTX             (0x0081)
#define PHY_ANAR_10BT_FDX           (0x0041)
#define PHY_ANAR_10BT               (0x0021)
#define PHY_ANAR_802_3              (0x0001)

/* Bit definitions and macros for PHY_ANLPAR */
#define PHY_ANLPAR_NEXT_PAGE        (0x8000)
#define PHY_ANLPAR_ACK              (0x4000)
#define PHY_ANLPAR_REM_FAULT        (0x2000)
#define PHY_ANLPAR_PAUSE            (0x0400)
#define PHY_ANLPAR_100BT4           (0x0200)
#define PHY_ANLPAR_100BTX_FDX       (0x0100)
#define PHY_ANLPAR_100BTX           (0x0080)
#define PHY_ANLPAR_10BTX_FDX        (0x0040)
#define PHY_ANLPAR_10BT             (0x0020)


/* Bit definition and macros for PHY_PHYCTRL1 */
#define PHY_PHYCTRL1_LED_MASK       (0xC000)
#define PHY_PHYCTRL1_POLARITY       (0x2000)
#define PHY_PHYCTRL1_MDX_STATE      (0x0800)
#define PHY_PHYCTRL1_REMOTE_LOOP    (0x0080)

/* Bit definition and macros for PHY_PHYCTRL2 */
#define PHY_PHYCTRL2_HP_MDIX        (0x8000)
#define PHY_PHYCTRL2_MDIX_SELECT    (0x4000)
#define PHY_PHYCTRL2_PAIRSWAP_DIS   (0x2000)
#define PHY_PHYCTRL2_ENERGY_DET     (0x1000)
#define PHY_PHYCTRL2_FORCE_LINK     (0x0800)
#define PHY_PHYCTRL2_POWER_SAVING   (0x0400)
#define PHY_PHYCTRL2_INT_LEVEL      (0x0200)
#define PHY_PHYCTRL2_EN_JABBER      (0x0100)
#define PHY_PHYCTRL2_AUTONEG_CMPLT  (0x0080)
#define PHY_PHYCTRL2_ENABLE_PAUSE   (0x0040)
#define PHY_PHYCTRL2_PHY_ISOLATE    (0x0020)
#define PHY_PHYCTRL2_OP_MOD_MASK    (0x001C)
#define PHY_PHYCTRL2_EN_SQE_TEST    (0x0002)
#define PHY_PHYCTRL2_DATA_SCRAM_DIS (0x0001)


/* Bit definitions of PHY_PHYCTRL2_OP_MOD_MASK */
#define PHY_PHYCTRL2_OP_MOD_SHIFT             2
#define PHY_PHYCTRL2_MODE_OP_MOD_STILL_NEG    0
#define PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_HD    1
#define PHY_PHYCTRL2_MODE_OP_MOD_100MBPS_HD   2
#define PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_FD    5
#define PHY_PHYCTRL2_MODE_OP_MOD_100MBPS_FD   6

/*******************************************************************************
  ENET BUFFER Descriptors control and status bits
*******************************************************************************/

/* Standard TX Buffer Descriptors */
#define ENET_BD_TX_R        0x0080
#define ENET_BD_TX_TO1      0x0040
#define ENET_BD_TX_W        0x0020
#define ENET_BD_TX_TO2      0x0010
#define ENET_BD_TX_L        0x0008
#define ENET_BD_TX_TC       0x0004
#define ENET_BD_TX_ABC      0x0002

/* Enhanced TX Buffer Descriptors */
#define ENET_BD_TX_INT      0x00000040
#define ENET_BD_TX_TS       0x00000020
#define ENET_BD_TX_PINS     0x00000010
#define ENET_BD_TX_IINS     0x00000008
#define ENET_BD_TX_TXE      0x00800000
#define ENET_BD_TX_UE       0x00200000
#define ENET_BD_TX_EE       0x00100000
#define ENET_BD_TX_FE       0x00080000
#define ENET_BD_TX_LCE      0x00040000
#define ENET_BD_TX_OE       0x00020000
#define ENET_BD_TX_TSE      0x00010000
#define ENET_BD_TX_BDU      0x00000080

/* Standard RX Buffer Descriptors */
#define ENET_BD_RX_E        0x0080
#define ENET_BD_RX_R01      0x0040
#define ENET_BD_RX_W        0x0020
#define ENET_BD_RX_R02      0x0010
#define ENET_BD_RX_L        0x0008
#define ENET_BD_RX_M        0x0001
#define ENET_BD_RX_BC       0x8000
#define ENET_BD_RX_MC       0x4000
#define ENET_BD_RX_LG       0x2000
#define ENET_BD_RX_NO       0x1000
#define ENET_BD_RX_CR       0x0400
#define ENET_BD_RX_OV       0x0200
#define ENET_BD_RX_TR       0x0100

/* Enhanced RX Buffer Descriptors */
#define ENET_BD_RX_ME       0x00000080
#define ENET_BD_RX_PE       0x00000004
#define ENET_BD_RX_CE       0x00000002
#define ENET_BD_RX_UC       0x00000001
#define ENET_BD_RX_INT      0x00008000
#define ENET_BD_RX_ICE      0x20000000
#define ENET_BD_RX_PCR      0x10000000
#define ENET_BD_RX_VLAN     0x04000000
#define ENET_BD_RX_IPV6     0x02000000
#define ENET_BD_RX_FRAG     0x01000000
#define ENET_BD_RX_BDU      0x00000080

/* PHY硬件特性 */
#define PHY_STATUS                  ( 0x1F )
#define PHY_DUPLEX_STATUS           ( 4<<2 )
#define PHY_SPEED_STATUS            ( 1<<2 )
/* PHY收发器硬件地址 */
#define CFG_PHY_ADDRESS             (0x01)

typedef struct devoptab_s {                       /* Device Operations Table */
    const char *name;
    uint32_t    maj;
    uint32_t    min;
    void       *priv;
} devoptab_t;

typedef enum  __attribute__((__packed__)){
    ENET_MODULE_0,
    NUM_ENET_MODULES,
} enetModule_t;


/*******************************************************************************
* Define basic ethernet frame per IEEE802.3 standard
*******************************************************************************/

/* Ethernet standard lengths in bytes*/
#define ETH_ADDR_LEN    (6)
#define ETH_TYPE_LEN    (2)
#define ETH_CRC_LEN     (4)
#define ETH_MAX_DATA    (1500)
#define ETH_MIN_DATA    (46)
#define ETH_HDR_LEN     (ETH_ADDR_LEN * 2 + ETH_TYPE_LEN)

/* Defined Ethernet Types */
#define ETHTYPE_IPV4    (0x0800)        /* IP V4 RFC-894 (RFC-791) */
#define ETHTYPE_ARP     (0x0806)        /* ARP RFC-826 */
#define ETHTYPE_IPV6    (0x86DD)        /* IP V6 RFC-2464 */

/* Maximum and Minimum Ethernet Frame Sizes */
#define ETH_MAX_FRM     (ETH_HDR_LEN + ETH_MAX_DATA + ETH_CRC_LEN)
#define ETH_MIN_FRM     (ETH_HDR_LEN + ETH_MIN_DATA + ETH_CRC_LEN)
#define ETH_MTU         (ETH_HDR_LEN + ETH_MAX_DATA)

#define SCGC2_BASE_PTR	0x40048030

/* Ethernet Frame Header definition */
typedef struct eth_header_s {
    uint8_t    dest[ETH_ADDR_LEN];
    uint8_t    src[ETH_ADDR_LEN];
    uint16_t   type;
} eth_header_t;

/*******************************************************************************
* Define Packet elements for common protocols
*******************************************************************************/

#define ETH_IP4ADDR_LEN   (4)

/* ARP , EthType = 0x0806 */
typedef struct arp_packet_s {
    uint16_t    htype;          /* Network protocol type, Ethernet = 1 */
    uint16_t    ptype;          /* Internetwork protocol, IPV4 = 0x0800 */
    uint8_t     hlen;           /* Length of hardware address in octets = 6 */
    uint8_t     plen;           /* Length of protocol address in octets = 4 */
    uint16_t    oper;           /* Operation request=1, answer=2 */
    uint8_t     sha[ETH_ADDR_LEN]; /* sender hardware address */
    uint8_t     spa[ETH_IP4ADDR_LEN]; /* sender protocol address */
    uint8_t     tha[ETH_ADDR_LEN]; /* target hardware address */
    uint8_t     tpa[ETH_IP4ADDR_LEN]; /* target IP4 address */
} arp_packet_t;

#define ETH_ARP_LEN (28)

/* IPV4, EthType = 0x0800 */
typedef struct ipv4_packet_s {
    uint8_t     ver_ihl;        /* 0-3 version (4), 4-7 header len (5 Min)*/
    uint8_t     dscp_ecn;       /* 0-5 diffentiated services,6-7 congestion */
    uint16_t    len;            /* Total length - min 20bytes, max 65K */
    uint16_t    id;             /* fragement ID */
    uint16_t    flg_off;        /* 0-2 flags, 3-15 fragment offset */
    uint8_t     ttl;            /* time to live counter */
    uint8_t     protocol;       /* Datagram protocol number */
    uint16_t    header_csum;    /* Header checksum */
    uint8_t     spa[ETH_IP4ADDR_LEN]; /* source IPV4 Address */
    uint8_t     tpa[ETH_IP4ADDR_LEN]; /* target IPV4 Address */
    uint8_t*    opt_or_payload; /* Either payload (len=5) or options (rare) */
} ipv4_packet_t;

#define ETH_IPV4_LEN (20)

/* Some IPV4 datagram protocol numbers per ipv4_packet_t.protocol */

#define ETH_PROTO_ICMP      (0x01)
#define ETH_PROTO_TCP       (0x06)
#define ETH_PROTO_UDP       (0x11)

typedef struct icmp_packet_s {
    uint8_t type;
    uint8_t code;
    uint16_t csum;
} icmp_packet_t;

#define ETH_ICMP_ECHO_REQUEST (8)
#define ETH_ICMP_ECHO_REPLY (0)

#define ETH_ICMP_LEN (4)

typedef struct udp_packet_s {
    uint16_t source_port;
    uint16_t dest_port;
    uint16_t len;
    uint16_t csum;
} udp_packet_t;

#define ETH_UDP_LEN (8)

typedef struct tcp_packet_s {
    uint16_t source_port;
    uint16_t dest_port;
    uint32_t tx_sequence;
    uint32_t ack_number;
    uint16_t flags;
    uint16_t window;
    uint16_t csum;
    uint16_t urgent;
    uint32_t options;
} tcp_packet_t;

#define ETH_TCP_LEN (24)

#define TCP_FLG_FIN_MSK (0x0001)
#define TCP_FLG_SYN_MSK (0x0002)
#define TCP_FLG_RST_MSK (0x0004)
#define TCP_FLG_PSH_MSK (0x0008)
#define TCP_FLG_ACK_MSK (0x0010)
#define TCP_FLG_URG_MSK (0x0020)
#define TCP_FLG_ECE_MSK (0x0040)
#define TCP_FLG_CWR_MSK (0x0080)
#define TCP_FLG_NS_MSK  (0x0100)
#define TCP_FLG_DATA_MSK (0xF000)
#define TCP_FLG_DATA_SHIFT (12)

#define TCP_FLG_GET_DATA(data) ((data & TCP_FLG_DATA_MSK) >> TCP_FLG_DATA_SHIFT)
#define TCP_FLG_SET_DATA(data) ((data << TCP_FLG_DATA_SHIFT) & TCP_FLG_DATA_MASK)

/* ENET ***********************************************************************/

#define DEVOPTAB_ENET0_STR "eth0"

int  enet_install (void);
int eth_hwaddr_match (uint8_t* a1, uint8_t* a2);
int eth_hwaddr_zero (uint8_t* a1);
int eth_hwaddr_eff (uint8_t* a1);
void eth_hwaddr_copy (uint8_t* dest, uint8_t* src);
int __SD_little2Big_end(int num);

/* Need these swap macros to deal with little/big endian issues */
#define BSWAP32(inval) (__SD_little2Big_end((uint32_t)(inval)))
#define BSWAP16(inval) ((uint16_t)__SD_little2Big_end(((uint32_t)(inval)) << 16 ))

enum {
    IO_IOCTL_ENET_SET_MAC_ADDRESS,      /* You have to set a MAC address */
    IO_IOCTL_ENET_SET_ENET_STATE,       /* Turn ethernet on/off - Must have a MAC */
    IO_IOCTL_ENET_GET_ENET_STATE,       /* Return ethernet on / off state */
    IO_IOCTL_ENET_GET_DETAILED_ERROR,   /* Return more detailed error info */
    IO_IOCTL_ENET_SET_AUTONEGOTIATE,    /* Set Phy autonegotiate on/off */
    IO_IOCTL_ENET_SET_SPEED,            /* Set desired link speed */
    IO_IOCTL_ENET_SET_DUPLEX,           /* Set desired duplex */
    IO_IOCTL_ENET_GET_PHY_CONFIG,       /* Return current phy config struct */
    IO_IOCTL_ENET_GET_PHY_STATUS,       /* Return current phy status struct */
    IO_IOCTL_ENET_GET_PHY_REG,          /* Read a raw phy register */
    IO_IOCTL_ENET_GET_LAST_RXBD,        /* Return contents of last rxbd */
    MAX_IO_IOCTRL_ENET_CMDS,
};

typedef enum  __attribute__((__packed__)){
    ENET_OFF,
    ENET_ON
} enet_state_t;

typedef enum  __attribute__((__packed__)){
    ENET_AUTONEG_ON,
    ENET_AUTONEG_OFF
} enet_autoneg_t;

typedef enum  __attribute__((__packed__)){
    ENET_LINK_DOWN,
    ENET_LINK_UP
} enet_link_t;

typedef enum  __attribute__((__packed__)){
    ENET_10BASET,
    ENET_100BASET
} enet_speed_t;

typedef enum  __attribute__((__packed__)){
    ENET_DUPLEX_HALF,
    ENET_DUPLEX_FULL
} enet_duplex_t;

typedef enum  __attribute__((__packed__)){
    ENET_LOOPBACK_INT,
    ENET_LOOPBACK_EXT,
    ENET_LOOPBACK_OFF
} enet_loopback_t;

typedef enum  __attribute__((__packed__)){
    ENET_PROM_OFF,
    ENET_PROM_ON
} enet_prom_t;

typedef struct  __attribute__((__packed__)){
    uint8_t         phy_addr;
    enet_autoneg_t  autoneg;
    enet_speed_t    speed;
    enet_duplex_t   duplex;
    enet_loopback_t loopback;
    enet_prom_t     prom;
    uint8_t         mac_addr[6];
} enet_cfg_t;

typedef struct  __attribute__((__packed__)){
    enet_state_t    on_off;
    enet_link_t     link;
    enet_speed_t    speed;
    enet_duplex_t   duplex;
} enet_status_t;

typedef struct  __attribute__((__packed__)){
    uint16_t status;    /* control and status */
    uint16_t length;    /* transfer length */
    uint8_t  *buf_addr; /* buffer address NOTE Big Endian!!!*/
} enet_descr_t;

    
/*****************************************************************************/
/*
 * Enums and structure to hold per-channel ethernet configuration
 *
 * Note: You need a real legal MAC address if you are doing anything
 * other than testing.
 *
 */

/*****************************************************************************/
/* Physical Kinetis defines
 */
typedef enum  __attribute__((__packed__)){
    PIN_ENET_MDC,
    PIN_ENET_MDIO,
    PIN_ENET_RXER,
    PIN_ENET_RXDV,
    PIN_ENET_RXD0,
    PIN_ENET_RXD1,
    PIN_ENET_TXEN,
    PIN_ENET_TXD0,
    PIN_ENET_TXD1,
    NUM_PINS,
} enetPins_t;

typedef struct  __attribute__((__packed__)){
    unsigned num;
    unsigned port;
    unsigned mux;
} enetPin_t;

typedef struct  __attribute__((__packed__)){
    unsigned vector;
    unsigned (*isr)(void);
} enetIrq_t;

typedef enum  __attribute__((__packed__)){
    HND_ENET_MAC_1588_TIMER,
    HND_ENET_MAC_TX,
    HND_ENET_MAC_RX,
    HND_ENET_MAC_ERROR,
    NUM_HANDLERS
} enetIrqHandler_t;

/*****************************************************************************/
/* 
 * Buffers and buffer descriptors
 */

#define ENET_RX_BUFFER_SIZE 256     /* Must be a multiple of 16 */
#define ENET_TX_BUFFER_SIZE 256

#define ENET_NUM_RXBDS 16           /* defines number of BD,s and buffers */
#define ENET_NUM_TXBDS 16

typedef uint8_t enet_rxbuf_t [ENET_RX_BUFFER_SIZE];
typedef uint8_t enet_txbuf_t [ENET_TX_BUFFER_SIZE];

/* Buffer descriptors  MUST BE 16-byte boundary aligned */
static enet_descr_t rxbds[ENET_NUM_RXBDS] __attribute__ ((aligned (16)));
static enet_descr_t txbds[ENET_NUM_TXBDS] __attribute__ ((aligned (16)));

/* Actual buffers MUST BE 16-byte boundary aligned */
static enet_rxbuf_t rxbuffers[ENET_NUM_RXBDS] __attribute__ ((aligned (16)));
static enet_txbuf_t txbuffers[ENET_NUM_TXBDS] __attribute__ ((aligned (16)));

/* Next BD indicies for BD ring */ 
static int next_rxbd;
static int next_txbd;

typedef struct __attribute__((__packed__))
{
    uint32_t * addr;
    enet_cfg_t config;
    enet_status_t status;
    int phy_auto_req_data;
    enet_descr_t last_rx_packet;
    enetPin_t  pin[NUM_PINS];
    enetIrq_t  irq[NUM_HANDLERS];
    unsigned (*write)();//(void *enetPtr, const void *data, unsigned len);
    unsigned (*read) ();//(void *enetPtr, void *data, unsigned len);
    uint32_t * simBBPtr;
} enet_t;

typedef union  __attribute__((__packed__))
{
	struct __attribute__((__packed__))
	{
		uint16_t 	DATAGRAM_LENGTH_B;	//
		uint8_t		TOS;//
		uint8_t		IP_HEAD_LEN_B:4;//
		uint8_t		VER:4;//
		uint16_t	FRAGMENT_OFF:13;//
		uint8_t		FLAG:3;
		uint16_t	FRAGMENT_ID;
		uint16_t	CHECK;
		uint8_t		PROTO;
		uint8_t		TTL;
		uint32_t	SRC_IP;
		uint32_t	DEST_IP;
		uint32_t	DATA[5];
	};
	uint32_t rawBytes[5+5];
}IPv_4_ENET_PACKT_T;

void SD_ENET_install();
int enetOpen(devoptab_t *dot);
void enetInit (enet_t *enet);
uint8_t SD_ENET_start();
uint8_t SD_ENET_read(devoptab_t dev, uint8_t * buf, int len);
uint8_t SD_ENET_write(devoptab_t dev, uint8_t * buf);
int8_t SD_ENET_pin_init(uint8_t mode);

#endif  /* !defined(ETH_FRAME_H) */
