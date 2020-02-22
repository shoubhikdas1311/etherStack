#include "packet_Handler.h"

uint16_t *Tx_buff;

uint16_t SD_ETH_write_IP_Packet(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest)
{
	uint8_t j = 0, *data;// = "ABCDEFGHIJKLMNOPQRSTUVWXYZ\0";
	uint16_t i, len;
	SD_eth_test_t eth_pack;
//	arp_packet_t arp_pack;
	eth_pack.HEADER.type = ETHTYPE_ARP;
	for(i = 0; i < 300; i++)
	{
		eth_pack.IP4.DATA[i] = ('A' + j++);
		if((j+'A') == 'Z')
			j = 0;
	}
	eth_pack.IP4.DATA[i] = '\0';
	len = i;
	for(i = 0; i < ETH_ADDR_LEN; i++)
	{
		eth_pack.HEADER.src[i] = src[i];
		eth_pack.HEADER.dest[i] = dest[i];
//		arp_pack.sha[i] = src[i];		
//		arp_pack.tha[i] = dest[i];
	}
	eth_pack.HEADER.type = end_bswap16(0x0800);
	eth_pack.IP4.VER = 0X4;
	eth_pack.IP4.IP_HEAD_LEN_B = 0x5;
	eth_pack.IP4.TOS = 0xFF;
	eth_pack.IP4.DATAGRAM_LENGTH_B = end_bswap16(len);
	eth_pack.IP4.FRAGMENT_ID = 1;
	//eth_pack.IP4.FLAG = 0x2;
	eth_pack.IP4.FRAGMENT_OFF = /*end_bswap16*/(0x4000);
	eth_pack.IP4.TTL = 128;
	eth_pack.IP4.PROTO = 0x1;
	eth_pack.IP4.CHECK = /*end_bswap16*/(uint16_t)(ipv4_check(eth_pack.IP4, 9));
	for(i = 0; i < 4; i++)
	{
		eth_pack.IP4.SRC_IP[i] = /*end_bswap16*/(IP_src[i]);
		eth_pack.IP4.DEST_IP[i] = /*end_bswap16*/(IP_dest[i]);
	}
//	eth_pack.IP4.DATA = data;
//	ENET_MacSendData((uint16_t *)eth_pack.rawBytes, len+2);
	for(i = 0; i < 150 ; i++)
		Tx_buff[i] = eth_pack.rawBytes[i];
	SD_enetWritePolled((void *)ENET_BASE_PTR ,(uint8_t *)eth_pack.rawBytes, len+2+IP_HEAD_LEN);
//	arp_pack.hlen = 1;
//	arp_pack.htype = 0x0800;
//	arp_pack.hlen = 6;
//	arp_pack.plen = 4;
//	arp_pack.oper = 1;	
//	for(i = 0; i < ETH_IP4ADDR_LEN; i++)
//	{
//		arp_pack.spa[i] = IP_src[i];		
//		arp_pack.tpa[i] = IP_dest[i];
//	}
//	for(i = 0; i < 15; i++)
//	{
//		arp_pack.data[i] = 'A' + i;
//	}
//	enetWritePacket(ENET_BASE_PTR, arp_pack.rawBytes, 200);
//	while(arp_pack.rawBytes[i])
//	{
//		pxENETTxDescriptor[i]
//	}
/*	for(i = 0; i < 7; i++)
	{
		mii_write(0, CFG_PHY_ADDRESS, PHY_PHYIDR1, eth_pack.rawBytes[i]);
	}
	for(; i < 14+7; i++)
	{
		mii_write(0, CFG_PHY_ADDRESS, PHY_PHYIDR1, arp_pack.rawBytes[i]);
	}*/
	return (eth_pack.IP4.DATAGRAM_LENGTH_B + eth_pack.IP4.IP_HEAD_LEN_B); 
}
uint16_t ETH_write_ARP_Packet(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest, uint16_t req_ans)
{
	uint8_t i;
	eth_header_t eth_pack;
	arp_packet_t arp_pack;
	eth_pack.type = ETHTYPE_ARP;
	for(i = 0; i < ETH_ADDR_LEN; i++)
	{
		eth_pack.src[i] = src[i];
		eth_pack.dest[i] = dest[i];
		arp_pack.sha[i] = src[i];		
		arp_pack.tha[i] = dest[i];
	}
	arp_pack.hlen = 1;
	arp_pack.htype = 0x0800;
	arp_pack.hlen = 6;
	arp_pack.plen = 4;
	arp_pack.oper = 1;	
	for(i = 0; i < ETH_IP4ADDR_LEN; i++)
	{
		arp_pack.spa[i] = IP_src[i];		
		arp_pack.tpa[i] = IP_dest[i];
	}
	for(i = 0; i < 15; i++)
	{
		arp_pack.data[i] = 'A' + i;
	}
/*	enetWritePacket(ENET_BASE_PTR, arp_pack.rawBytes, 200);
	while(arp_pack.rawBytes[i])
	{
		pxENETTxDescriptor[i];
	}
/*	for(i = 0; i < 7; i++)
	{
		mii_write(0, CFG_PHY_ADDRESS, PHY_PHYIDR1, eth_pack.rawBytes[i]);
	}
	for(; i < 14+7; i++)
	{
		mii_write(0, CFG_PHY_ADDRESS, PHY_PHYIDR1, arp_pack.rawBytes[i]);
	}*/
	return i; 
}
uint16_t ipv4_check(IPv_4_ENET_PACKT_T buf, unsigned size)
{
	//unsigned sum = 0;
	int i, num = 0x00000000;
    uint16_t res, carry;

	/* Accumulate checksum */
	for (i = 0; i < size; i++)
	{
		num += buf.rawBytes[i];
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
	return (uint16_t)(~res);
}