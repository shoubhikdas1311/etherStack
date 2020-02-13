#include "SD_Clk.h"
#define K60_CLK									1
#define Clk144									1
#if (defined(Clk144))
	int sys_clk_init(int clk_option)
	{
		  unsigned char mcgclkout;
		  
		  SIM_SCGC5 = (
					  SIM_SCGC5_PORTA_MASK	|
					  SIM_SCGC5_PORTB_MASK	|
					  SIM_SCGC5_PORTC_MASK	|
					  SIM_SCGC5_PORTD_MASK	|
					  SIM_SCGC5_PORTE_MASK
					  );
		  
		  if (clk_option > 3) {return 0;} //return 0 if one of the available options is not selected
		  //if (prdiv_val > 15) {return 1;} // return 1 if one of the available crystal options is not available
		//This assumes that the MCG is in default FEI mode out of reset.

		// First move to FBE mode
		#if (defined(K60_CLK) || defined(ASB817))
			 MCG_C2 = 0;
		#else
		// Enable external oscillator, RANGE=2, HGO=1, EREFS=1, LP=0, IRCS=0
			MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
		#endif

			OSC_CR |= OSC_CR_ERCLKEN_MASK;// | OSC_CR_SC16P_MASK;
		// after initialization of oscillator release latched state of oscillator and GPIO
			SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
		  LLWU_FILT1 |= 0x80u;	// it is LLWU_CS but not defined in present header file
		  
		// Select external oscilator and Reference Divider and clear IREFS to start ext osc
		// CLKS=2, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
		  MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);

		  /* if we aren't using an osc input we don't need to wait for the osc to init */
		#if (!defined(K60_CLK) && !defined(ASB817))
			while (!(MCG_S & MCG_S_OSCINIT_MASK)){};  // wait for oscillator to initialize
		#endif
			
		  MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK ));
		  MCG_C4 |=  MCG_C4_DRST_DRS(0x03);
		  while (MCG_S & MCG_S_IREFST_MASK){}; // wait for Reference clock Status bit to clear

		  while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}; // Wait for clock status bits to show clock source is ext ref clk

		// Now in FBE


			MCG_C6 = 0;
		#if (defined(K60_CLK))
		   MCG_C5 = MCG_C5_PRDIV0(0x18);
		#else
		// Configure PLL Ref Divider, PLLCLKEN=0, PLLSTEN=0, PRDIV=5
		// The crystal frequency is used to select the PRDIV value. Only even frequency crystals are supported
		// that will produce a 2MHz reference clock to the PLL.
		  MCG_C5 = MCG_C5_PRDIV(0x14); // Set PLL ref divider to match the crystal used
		#endif

		  // Ensure MCG_C6 is at the reset default of 0. LOLIE disabled, PLL disabled, clk monitor disabled, PLL VCO divider is clear
		  MCG_C6 = 0x0;
		// Select the PLL VCO divider and system clock dividers depending on clocking option
		  switch (clk_option) {
			case 0:
			  // Set system options dividers
			  //MCG=PLL, core = MCG, bus = MCG, FlexBus = MCG, Flash clock= MCG/2
			  set_sys_dividers(0,0,0,1);
			  // Set the VCO divider and enable the PLL for 50MHz, LOLIE=0, PLLS=1, CME=0, VDIV=1
			  MCG_C6 |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(1); //VDIV = 1 (x25)
			  mcgclkout = 50;
			  break;
		   case 1:
			  // Set system options dividers
			  //MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/2, Flash clock= MCG/4
			 set_sys_dividers(0,1,1,3);
			  // Set the VCO divider and enable the PLL for 100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
			  MCG_C6 |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(26); //VDIV = 26 (x50)
			  mcgclkout = 100;
			  break;
			case 2:
			  // Set system options dividers
			  //MCG=PLL, core = MCG, bus = MCG/2, FlexBus = MCG/2, Flash clock= MCG/4
			  set_sys_dividers(0,1,1,3);
			  // Set the VCO divider and enable the PLL for 96MHz, LOLIE=0, PLLS=1, CME=0, VDIV=24
			  MCG_C6 |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(24); //VDIV = 24 (x48)
			  mcgclkout = 96;
			  break;
		   case 3:
			  // Set system options dividers
			  //MCG=PLL, core = MCG, bus = MCG, FlexBus = MCG, Flash clock= MCG/2
			  set_sys_dividers(0,0,0,1);
			  // Set the VCO divider and enable the PLL for 48MHz, LOLIE=0, PLLS=1, CME=0, VDIV=0
			  MCG_C6 |= MCG_C6_PLLS_MASK; //VDIV = 0 (x24)
			  mcgclkout = 48;
			  break;
		  }
		  while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set
		  MCG_C5 |= MCG_C5_PLLCLKEN0_MASK;

	//	  while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set

		// Now running PBE Mode

		// Transition into PEE by setting CLKS to 0
		// CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
		  MCG_C1 &= ~MCG_C1_CLKS_MASK;

		// Wait for clock status bits to update
		  while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

		// Now running PEE Mode

		return mcgclkout;
	}
#else
	#define CLK100 1
	int sys_clk_init(int clk_option)
	{
		int mcgclkout;
		SIM_CLKDIV1 |= (
					SIM_CLKDIV1_OUTDIV1(0x0)  |
					SIM_CLKDIV1_OUTDIV2(0x0)  |
					SIM_CLKDIV1_OUTDIV3(0x0)  |
					SIM_CLKDIV1_OUTDIV4(0x1)
				);
		/* all core = peri = flexbus = flash*/
		if ((PMC_REGSC & 0x8u) != 0x0u) 
		{
			/* PMC_REGSC: ACKISO=1 */
			PMC_REGSC |= 0X8u; /* Release IO pads after wakeup from VLLS mode. */
		}	
		SIM_SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL_MASK); /* Select FLL as a clock source for various peripherals */
		/* SIM_SOPT1: OSC32KSEL=3 */
		SIM_SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;
		MCG_C1 |= (
				MCG_C1_CLKS(0X00) |
				MCG_C1_IREFS_MASK | 
				MCG_C1_IRCLKEN_MASK
			);//C1[CLKS-00], C6[PLLS-0]
		/* MCG_C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
		MCG_C2 = MCG_C2_RANGE0(0x00);
		MCG_C6 &= (MCG_C6_PLLS_MASK);//C6[PLLS-0]
		/* MCG_C4: DMX32=0,DRST_DRS=0 */
		MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK ));
		MCG_C4 |=  MCG_C4_DRST_DRS(0x03);//	DCO range = 80 - 100
		while( MCG_S != ( MCG_S & (MCG_S_CLKST(0X00) | MCG_S_OSCINIT0_MASK | MCG_S_IRCST_MASK | MCG_S_IREFST_MASK )))// FLL selected for o/p, osc initialized, fast clk selected, selected FLL src is internal
		{ 
			/* Check that the source of the FLL reference clock is the internal reference clock. */
		}
	/*}
	void FEI_2_FBE()
	{	*/
		//External oscillator init
		OSC_CR |= OSC_CR_ERCLKEN_MASK ;
	/**
	*	C2[RANGE] set to 2'b01 because the frequency of 4 MHz is within the high frequency range
	*	C2[HGO] set to 1 to configure the crystal oscillator for high gain operation
	*	C2[EREFS] set to 1, because a crystal is being used
	*
	*/
		MCG_C2 |= (
				MCG_C2_RANGE0(0X1)	|
				MCG_C2_HGO0_MASK	|
				MCG_C2_EREFS0_MASK
			);
		// after initialization of oscillator release latched state of oscillator and GPIO
		SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
		LLWU_FILT1 |= 0x80u;	// it is LLWU_CS but not defined in present header file
	/**
	*	C1[CLKS] set to 2'b10 in order to select external reference clock as system clock source
	*	C1[FRDIV] set to 3'b010, or divide-by-128 because 4 MHz / 128 = 31.25 kHz which is in the 31.25 kHz to 39.0625 kHz range required by the FLL
	*	C1[IREFS] cleared to 0, selecting the external reference clock and enabling the external oscillator.
	*/
		MCG_C1 |= (
				MCG_C1_CLKS(0X2)	|
				MCG_C1_FRDIV(0X4)
			);
		while((MCG_S & MCG_S_IREFST_MASK)){}
		while( (MCG_S & (MCG_S_CLKST(0X0) | MCG_S_OSCINIT0_MASK)))
		{ 
			/* Check that the source of the FLL reference clock is the internal reference clock. */
		}
	/*}
	void FBE_2_PBE()
	{*/

		  // set clock dividers to desired value  
		  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) 
					 | SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(3);

		  while(!(SIM_CLKDIV1 & (SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(3))));
		  
		  MCG_C5 = MCG_C5_PRDIV0(0x0A);						// CLK / 1

		  // Ensure MCG_C6 is at the reset default of 0. LOLIE disabled, PLL disabled, clk monitor disabled, PLL VCO divider is clear
		  switch(clk_option)
		  {
				case 0:
					MCG_C6 |= (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(26));	// CLK x 50
					set_CLKDIV(0,1,3,3);//100,50,25,25
					mcgclkout = 100;
					break;
				case 1:
					MCG_C6 |= (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(26));	// CLK x 50
					set_CLKDIV(0,1,1,3);//100,50,50,25
					mcgclkout = 100;
					break;
				case 2:
					MCG_C6 |= (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(1));	// CLK x 50
					set_CLKDIV(0,1,1,1);//50,25,25,25
					mcgclkout = 50;
					break;
				case 3:
					MCG_C6 |= (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(26));	// CLK x 50
					set_CLKDIV(0,0,0,0);//25,25,25,25
					mcgclkout = 25;
					break;

		  }
		while(!(MCG_S & MCG_S_PLLST_MASK))
		{
			/**
			*	Loop until S[PLLST] is set, indicating that the current source for the PLLS clock is the PLL
			*	Then loop until S[LOCK] is set, indicating that the PLL has acquired lock.
			*/
		}
		while(!(MCG_S & MCG_S_LOCK0_MASK))
			{}
	/*}
	void PBE_2_PEE()
	{*/
		MCG_C1 |= MCG_C1_CLKS(0X00);
		while(((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0X3)
		{}
		return mcgclkout;
	}
	

	/*future*/
#endif  
void set_sys_dividers(uint8_t out1, uint8_t out2, uint8_t out3, uint8_t out4)
{
	SIM_CLKDIV1 |= (
						SIM_CLKDIV1_OUTDIV1(out1)	|
						SIM_CLKDIV1_OUTDIV2(out2)	|
						SIM_CLKDIV1_OUTDIV3(out3)	|
						SIM_CLKDIV1_OUTDIV4(out4)
					);
}
