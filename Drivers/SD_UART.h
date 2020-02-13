/*
 * uart.h
 *
 *  Created on: Apr 18, 2019
 *      Author: SYS6_NET
 */

#ifndef UART_H_
#define UART_H_

#include "SD_Clk.h"
#include "MK60N512.h"
//#include "SIMPLE_CLK.h"
//#include "../common/common.h"


void uart5_init(UART_MemMapPtr, int ,int);
void uart_putChar(UART_MemMapPtr RxUart, char ch);
void uart_port_init(UART_MemMapPtr uart);
int SDprint(char *str);

#endif /* UART_H_ */
