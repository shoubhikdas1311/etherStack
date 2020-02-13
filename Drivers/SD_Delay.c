#include "SD_Delay.h"
void msDelay(int count)
{
	SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;
	lptmr_init();
	LPTMR0_CMR = LPTMR_CMR_COMPARE(count);
	LPTMR0_PSR |= LPTMR_PSR_PCS(0X1)|LPTMR_PSR_PBYP_MASK;
	LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
	while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK)  == 0)
	{}
	LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK; 
	return;
}
void lptmr_init(void)
{
	  LPTMR0_CSR=0x00;
	  LPTMR0_PSR=0x00;
	  LPTMR0_CMR=0x00;
	  return;
}
