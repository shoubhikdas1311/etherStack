
#ifndef CLOCK_H_
#define CLOCK_H_

#include "MK60N512.h"

int sys_clk_init(int clk_option);
void set_sys_dividers(uint8_t out1, uint8_t out2, uint8_t out3, uint8_t out4);

#endif /* CLOCK_H_ */
