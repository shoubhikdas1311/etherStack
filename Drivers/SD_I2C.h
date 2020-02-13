/*
 * I2C.h
 *
 *  Created on: Apr 27, 2019
 *      Author: SYS5
 */

#ifndef I2C_H_
#define I2C_H_

#include "SD_Delay.h"
/**
 * MMA7660FC has a 7-bit long slave address, shown in Figure 11. The bit following the 7-bit slave address (bit eight) is the
 * R/W bit, which is low for a write command and high for a read command. The device has a factory set I2C slave address which
 * is normally 1001100 (0x4C). Contact the factory to request a different I2C slave address, which is available in the range 0001000
 * to 1110111 (0x08 to 0xEF), by metal mask option.
 */
#define MMA7660_I2C_ADDRESS			0x4c
/*
#define i2c_DisableAck()       I2C0_C1 |= I2C_C1_TXAK_MASK
#define i2c_RepeatedStart()    I2C0_C1     |= 0x04;

#define i2c_Start()            I2C0_C1     |= 0x10;\
                               I2C0_C1     |= I2C_C1_MST_MASK

#define i2c_Stop()             I2C0_C1  &= ~I2C_C1_MST_MASK;\
                               I2C0_C1  &= ~I2C_C1_TX_MASK

#define i2c_EnterRxMode()      I2C0_C1   &= ~I2C_C1_TX_MASK;\
                               I2C0_C1   &= ~I2C_C1_TXAK_MASK

#define i2c_Wait()               while((I2C0_S & I2C_S_IICIF_MASK)==0) {} \
                                  I2C0_S |= I2C_S_IICIF_MASK;

#define i2c_write_byte(data)   I2C0_D = data*/
/* Master write  */
#define		MWSR					0x00
/* Master read */
#define		MRSW					0x01
/* tx define for acknowledgement */
#define		TX						0x00
/* rx define for acknowledgement */
#define		RX						0x01
void I2C_init(I2C_MemMapPtr I2C_port, uint8_t flg);
void I2C_Tx_init(I2C_MemMapPtr I2C_module, unsigned char mode);
uint8_t u8MMA7660ReadRegister(I2C_MemMapPtr I2C_module, uint8_t u8RegisterAddress);
void MMA7660WriteRegister(I2C_MemMapPtr I2C_module, uint8_t u8RegisterAddress, uint8_t u8Data);
void Pause(void);
int8_t I2C_Ack(I2C_MemMapPtr I2Cx, int tx_rx);

#endif /* I2C_H_ */
