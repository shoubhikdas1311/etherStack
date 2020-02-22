
#ifndef _PACKET_HANDLER_H_
#define _PACKET_HANDLER_H_
//#include "ETH.h"
//#include "SD_ETH.h"
#include "SD_ETH.h"

#pragma anon_unions

/* Defined Ethernet Types */
#define ETHTYPE_IPV4    (0x0800)        /* IP V4 RFC-894 (RFC-791) */
#define ETHTYPE_ARP     (0x0806)        /* ARP RFC-826 */
#define ETHTYPE_IPV6    (0x86DD)        /* IP V6 RFC-2464 */

/* IPV4, EthType = 0x0800 */
typedef union  __attribute__((__packed__))
{
	struct __attribute__((__packed__))
	{
		uint8_t		IP_HEAD_LEN_B:4;//	x
		uint8_t		VER:4;//			x
		uint8_t		TOS;//				x
		uint16_t 	DATAGRAM_LENGTH_B;	//x
		//uint8_t		FLAG:3;			//	x
		uint16_t	FRAGMENT_OFF;//:13;//	x
		uint16_t	FRAGMENT_ID;	//	x
		uint8_t		TTL;		//		x
		uint8_t		PROTO;
		uint16_t	CHECK;
		uint8_t		SRC_IP[ETH_IP4ADDR_LEN];
		uint8_t		DEST_IP[ETH_IP4ADDR_LEN];
		uint8_t		DATA[300];
	};
	uint16_t rawBytes[RAWLEN_16_t(300)];
} IPv_4_ENET_PACKT_T;

typedef union  __attribute__((__packed__))
{
	struct __attribute__((__packed__))
	{
    eth_header_t HEADER;
    IPv_4_ENET_PACKT_T IP4;
	};
	uint16_t rawBytes[((RAWLEN_16_t(300)) + 14)];
} SD_eth_test_t;
uint16_t SD_ETH_write_IP_Packet(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest);
uint16_t SD_ETH_write_MAC_Pack(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest);
uint16_t ipv4_check(IPv_4_ENET_PACKT_T buf, unsigned size);
#endif/*_PACKET_HANDLER_H_*/