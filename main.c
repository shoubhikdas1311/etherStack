/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#include "main.h" /* include peripheral declarations */
uint8_t flg, RxBuff[20];
uint8_t pxENETTxDescriptor_unaligned[100];
uint8_t pxENETRxDescriptors_unaligned[100];
uint16_t wdogIntCnt, len = 0;
uint32_t src_add = (uint32_t)&wdogIntCnt, dest_add = 0x140007FF, jiffies = 0, TimeSlice = 5000000, packetCount = 0;
int main(void)
{
//	devoptab_t ioctl;
	int counter = 0;
	uint16_t length = 0, i = 0;
	int8_t in = 'A', out, decryptOut,buff[20], *data;
	uint32_t dataCheck;
    int8_t key = 5, src_ip_add[] = {192,45,67,89}, dest_ip_add[] = {192,168,1,24};
	int sys_clk_mhz;
	uint8_t greet[] = "HELLO WORLD\0";
//	IPv_4_ENET_PACKT_T packet;
	uint8_t destMAC[] = {0x1C ,0x66 ,0x6D ,0x8F ,0x6A ,0xC2}, srcMAC[] = /*{0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF};*/{0x12 ,0x34 ,0x56 ,0x78 ,0x90 ,0x00};
//	uint16_t j = 0, reading;
//	uint16_t i = 0;
//	int16_t temp,dac_val = 0xFFF;
//    int increment = 0 ; 
//    int16_t center = 3000;
	sys_clk_mhz = sys_clk_init(1);
	gpio_init();
		LED_RED_ON;
		msDelay(100);
		LED_YELLOW_ON;
		msDelay(100);
		LED_RED_OFF;
		msDelay(100);
		LED_YELLOW_OFF;
		msDelay(100);
	uart_port_init(UART5_BASE_PTR);
	uart5_init(UART5_BASE_PTR, sys_clk_mhz, 115200);
	SDprint("HI WORLD\n\r");
//	ioctl.name = "ETH_0";
//	ioctl.maj = 31;
//	ioctl.min = 0;
//	ioctl.priv = ((void *)0);
	//ENET_BDInit();
	jiffiesInit();
//	memcpy(pxENETTxDescriptor_unaligned, "HELLO WORLD", 15);
//	ENET_pin_init(ENET_RMII_MODE_MASK);
//	ENET_Init(srcMAC);
	ETH_init(100, srcMAC);
//	EnetRxCB();
//	SD_ENET_install();
//    enetOpen(&ioctl);
//	SD_ENET_start();
//    printf("Size of char : %d\n", sizeof(char));
//    printf("Before encrypt : %c, %d, %x\n", in, in, in);
    out = (in << key) | (in >> ((sizeof(char)*8)-key));
//    printf("After encrypt : %c, %d, %x\n", out, out, out);
    decryptOut = (out << ((sizeof(char) * 8)-key)) | (out >> key);
//    printf("After decrypt : %c, %d, %x\n", decryptOut, decryptOut, decryptOut);
    launchJiffies();
    //WDOG_QuickInit(5000);
    //WDOG_CallbackInstall(wDog_isr_usr);
	set_irq_priority(-1,0);
	set_irq_priority(22,15);
	/*packet.TOS = 0xFF;
	packet.PROTO = 0x6;
	packet.TTL = 64;
	packet.FLAG = 0x01;
	packet.VER = 0x4;
	packet.IP_HEAD_LEN_B = 5;
	packet.DATAGRAM_LENGTH_B = 160;
	packet.PROTO = TCP_MASK;
	for(i = 0; i < 4; i++)
	{
		packet.SRC_IP |= (uint32_t)(src_ip_add[i] << ((3-i)*8));
		packet.DEST_IP |= (uint32_t)(dest_ip_add[i] << ((3-i)*8));
	}
	packet.FRAGMENT_OFF = 1;
	
		packet.FRAGMENT_ID = counter;
		packet.CHECK = ipv4_check(packet,9);
		for(i = 0; i < 26; i++)
		{
			packet.DATA[i] = 'A'+ i;
		}
		packet.DATA[i] = '\0';
	ENET_MacReceiveData((uint8_t *)data);*/
    while(1)
	{
//		LED_RED_ON;
//		msDelay(100);
//		LED_YELLOW_ON;
//		msDelay(100);
//		LED_RED_OFF;
//		msDelay(100);
//		LED_YELLOW_OFF;
//		msDelay(100);
		
		msDelay(100);
		uart_putChar(UART5_BASE_PTR, (uint8_t)counter++);
//		length = SDprint("This section provides a description of the enhanced operation of the driver/DMA via the buffer descriptor\n\r\0");
//		SD_enetWritePolled(ENET_BASE_PTR, "This section provides a description of the enhanced operation of the driver/DMA via the buffer descriptor", length);
//		ENET_MacReceiveData(data);
//		SDprint("BYE WORLD\n");
//		ENET_MacSendData((uint8_t *)packet.rawBytes, 12);
//		ENET_MacReceiveData((uint8_t *)RxBuff);
//		ENET_MacReceiveData(RxBuff);
//		SDprint((char *)RxBuff);
		SD_ETH_write_IP_Packet(srcMAC, destMAC, (uint8_t *)src_ip_add, (uint8_t *)dest_ip_add);
		printf("jiffies %d\tpacketCount %d", jiffies, packetCount);
//		ENET_MacReceiveData(RxBuff);
//		if(i == RAWLEN_16_t - 1)
		msDelay(100);
//		SDprint((char *)pxENETTxDescriptor_unaligned);
		
//		SDprint("----------------------------");
//		
//		SDprint(RxBuff);
////		SDprint((char *)pxENETRxDescriptors_unaligned);
//		
//		SDprint("*****************************");
//		
//		SDprint("\n\r\0");
//		SD_ENET_write(ioctl, (uint8_t *)"Hi World\n\r");
//		SD_ENET_read(ioctl, buff, 19);
		
		msDelay(100);
	}
	SDprint("BYE WORLD\n");
	
	return 0;
}
uint8_t wDog_isr_usr()
{
	static uint8_t count = 0;
	uint32_t doggy;
//	uint8_t intData[4], i;
//	char *ptr;
	SDprint("inside wDog_isr_usr\n\r\0");
	doggy = WDOG_GetCurrentCounter();
	//intToStr(doggy, ptr, 8);
	//SDprint(ptr);
	return count++;
}
// reverses a string 'str' of length 'len' 
void reverse(char *str, int len) 
{ 
	int i=0, j=len-1, temp; 
	while (i<j) 
	{ 
		temp = str[i]; 
		str[i] = str[j]; 
		str[j] = temp; 
		i++; j--; 
	} 
} 

// Converts a given integer x to string str[]. d is the number 
// of digits required in output. If d is more than the number 
// of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
	int i = 0; 
	while (x) 
	{ 
		str[i++] = (x%10) + '0'; 
		x = x/10; 
	} 

	// If number of digits required is more, then 
	// add 0s at the beginning 
	while (i < d) 
		str[i++] = '0'; 

	reverse(str, i); 
	str[i] = '\0'; 
	return i; 
} 
