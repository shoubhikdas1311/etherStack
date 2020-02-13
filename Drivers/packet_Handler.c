#include "packet_Handler.h"

uint16_t *Tx_buff;

uint16_t SD_ETH_write_IP_Packet(uint8_t* src, uint8_t* dest, uint8_t* IP_src, uint8_t* IP_dest)
{
	uint8_t *data = "The hash table algorithm used in the group and individual hash filtering operates as follows. The 48-bit destination address is mapped into one of 64 bits, represented by 64 bits in ENETn_GAUR/GALR (group address hash match) or ENETn_IAUR/IALR (individual address hash match). This mapping is performed by passing the 48-bit address through the on-chip 32-bit CRC generator and selecting the six most significant bits of the CRC-encoded result to generate a number between 0 and 63. The msb of the CRC result selects ENETn_GAUR (msb = 1) or ENETn_GALR (msb = 0). The five lsbs of the hash result select the bit within the selected register. If the CRC generator selects a bit set in the hash table, the frame is accepted; else, it is rejected. For example, if eight group addresses are stored in the hash table and random group addresses are received, the hash table prevents roughly 56/64 (or 87.5%) of the group address frames from reaching memory. Those that do reach memory must be further filtered by the processor to determine if they truly contain one of the eight desired addresses\n\r\0";
	uint16_t i, len;
	SD_eth_test_t eth_pack;
//	arp_packet_t arp_pack;
	eth_pack.HEADER.type = ETHTYPE_ARP;
	len = SDprint(data);
	for(i = 0; i < ETH_ADDR_LEN; i++)
	{
		eth_pack.HEADER.src[i] = src[i];
		eth_pack.HEADER.dest[i] = dest[i];
//		arp_pack.sha[i] = src[i];		
//		arp_pack.tha[i] = dest[i];
	}
	eth_pack.HEADER.type = 0x0800;
	eth_pack.IP4.VER = 0X4;
	eth_pack.IP4.IP_HEAD_LEN_B = 0x5;
	eth_pack.IP4.TOS = 0xFF;
	eth_pack.IP4.DATAGRAM_LENGTH_B = len;
	eth_pack.IP4.FRAGMENT_ID = 1;
	eth_pack.IP4.FLAG = 0x2;
	eth_pack.IP4.FRAGMENT_OFF = 0;
	eth_pack.IP4.TTL = 128;
	eth_pack.IP4.PROTO = 0x1;
	eth_pack.IP4.CHECK = ipv4_check((uint16_t *)eth_pack.IP4.rawBytes, 9);
	for(i = 0; i < 4; i++)
	{
		eth_pack.IP4.SRC_IP[i] = IP_src[i];
		eth_pack.IP4.DEST_IP[i] = IP_dest[i];
	}
//	eth_pack.IP4.DATA = data;
	for(i = 0; i < 200; i++)
		eth_pack.IP4.DATA[i] = data[i];
//	ENET_MacSendData((uint16_t *)eth_pack.rawBytes, len+2);
	for(i = 0; i < 800; i++)
		Tx_buff[i] = eth_pack.rawBytes[i];
	SD_enetWritePolled((void *)ENET_BASE_PTR ,(uint8_t *)eth_pack.rawBytes, len+2);
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