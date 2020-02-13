
#ifndef _PACKET_HANDLER_H_
#define _PACKET_HANDLER_H_
//#include "ETH.h"
#include "SD_ETH.h"
#include "ETH.h"

/* IPV4, EthType = 0x0800 */
typedef union  __attribute__((__packed__))
{
	struct __attribute__((__packed__))
	{
		uint8_t		VER:4;//			x
		uint8_t		IP_HEAD_LEN_B:4;//	x
		uint8_t		TOS;//				x
		uint16_t 	DATAGRAM_LENGTH_B;	//x
		uint16_t	FRAGMENT_ID;	//	x
		uint8_t		FLAG:3;			//	x
		uint16_t	FRAGMENT_OFF:13;//	x
		uint8_t		TTL;		//		x
		uint8_t		PROTO;
		uint16_t	CHECK;
		uint8_t		SRC_IP[ETH_IP4ADDR_LEN];
		uint8_t		DEST_IP[ETH_IP4ADDR_LEN];
		uint8_t		DATA[200];
	};
	uint16_t rawBytes[RAWLEN_16_t(200)];
} IPv_4_ENET_PACKT_T;

typedef union  __attribute__((__packed__))
{
	struct __attribute__((__packed__))
	{
    SD_eth_header_t HEADER;
    IPv_4_ENET_PACKT_T IP4;
	};
	uint16_t rawBytes[150];
} SD_eth_test_t;
uint16_t SD_ETH_write_MAC_Pack(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest);
#endif/*_PACKET_HANDLER_H_*/