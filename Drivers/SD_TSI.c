
#include "SD_TSI.h"

extern uint32 __VECTOR_RAM[];

uint16  g16ElectrodeTouch[16] = {0};
uint16  g16ElectrodeBaseline[16] = {0};

uint32  g32DebounceCounter[16] = {DBOUNCE_COUNTS};

/********************************************************************************
 *   TSI_Init: Initializes TSI module
 * Notes:
 *    -
 ********************************************************************************/
void TSI_Init(void)
{
  SIM_SCGC5 |= (SIM_SCGC5_TSI_MASK); //Turn on clock to TSI module

#ifdef TWR_K60N512
  PORTA_PCR4 = PORT_PCR_MUX(0);      //Enable ALT0 for portA4
  PORTB_PCR2 = PORT_PCR_MUX(0);      //Enable ALT0 for portB2
  PORTB_PCR3 = PORT_PCR_MUX(0);      //Enable ALT0 for portB3
  PORTB_PCR16 = PORT_PCR_MUX(0);      //Enable ALT0 for portB16

#else
  PORTB_PCR16 = PORT_PCR_MUX(0);      //Enable ALT0 for portB16
  PORTB_PCR17 = PORT_PCR_MUX(0);      //Enable ALT0 for portB17
  PORTB_PCR18 = PORT_PCR_MUX(0);      //Enable ALT0 for portB18
  PORTB_PCR19 = PORT_PCR_MUX(0);      //Enable ALT0 for portB19

#endif

  TSI0_GENCS |= ((TSI_GENCS_NSCN(10))|(TSI_GENCS_PS(3)));
  TSI0_SCANC |= ((TSI_SCANC_EXTCHRG(3))|(TSI_SCANC_REFCHRG(31))|(TSI_SCANC_DELVOL(7))|(TSI_SCANC_SMOD(0))|(TSI_SCANC_AMPSC(0)));

  ELECTRODE_ENABLE_REG = ELECTRODE_RED_EN_MASK|ELECTRODE_GREEN_EN_MASK|ELECTRODE_YELLOW_EN_MASK|ELECTRODE_BLUE_EN_MASK;

  TSI0_GENCS |= (TSI_GENCS_TSIEN_MASK);  //Enables TSI

  /* Init TSI interrupts */
  enable_irq(83);  //TSI Vector is 99. IRQ# is 99-16=83
  /***********************/

}

/********************************************************************************
 *   TSI_SelfCalibration: Simple auto calibration version
 * Notes:
 *    - Very simple, only sums a prefixed amount to the current baseline
 ********************************************************************************/
void TSI_SelfCalibration(void)
{
  TSI0_GENCS |= TSI_GENCS_SWTS_MASK;

  while(!(TSI0_GENCS&TSI_GENCS_EOSF_MASK)){};

  msDelay(25000);

  g16ElectrodeBaseline[ELECTRODE_RED] = ELECTRODE_RED_COUNT;
  ELECTRODE_RED_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE_RED]+ELECTRODE_RED_OVRRUN));
  g16ElectrodeTouch[ELECTRODE_RED] = g16ElectrodeBaseline[ELECTRODE_RED] + ELECTRODE_RED_TOUCH;

  g16ElectrodeBaseline[ELECTRODE_GREEN] = ELECTRODE_GREEN_COUNT;
  ELECTRODE_GREEN_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE_GREEN]+ELECTRODE_GREEN_OVRRUN));
  g16ElectrodeTouch[ELECTRODE_GREEN] = g16ElectrodeBaseline[ELECTRODE_GREEN] + ELECTRODE_GREEN_TOUCH;

  g16ElectrodeBaseline[ELECTRODE_YELLOW] = ELECTRODE_YELLOW_COUNT;
  ELECTRODE_YELLOW_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE_YELLOW]+ELECTRODE_YELLOW_OVRRUN));
  g16ElectrodeTouch[ELECTRODE_YELLOW] = g16ElectrodeBaseline[ELECTRODE_YELLOW] + ELECTRODE_YELLOW_TOUCH;

  g16ElectrodeBaseline[ELECTRODE_BLUE] = ELECTRODE_BLUE_COUNT;
  ELECTRODE_BLUE_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE_BLUE]+ELECTRODE_BLUE_OVRRUN));
  g16ElectrodeTouch[ELECTRODE_BLUE] = g16ElectrodeBaseline[ELECTRODE_BLUE] + ELECTRODE_BLUE_TOUCH;

  DISABLE_TSI;

}

/********************************************************************************
 *   TSI_isr: TSI interrupt subroutine
 * Notes:
 *    -
 ********************************************************************************/

void TSI_isr(void)
{
  static uint16 TouchEvent = 0;
  uint16 lValidTouch = 0;
  uint16 l16Counter;

  TSI0_GENCS |= TSI_GENCS_OUTRGF_MASK;
  TSI0_GENCS |= TSI_GENCS_EOSF_MASK;


  /* Process electrode 0 */
  l16Counter = ELECTRODE_RED_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE_RED]) //See if detected a touch
  {
    TouchEvent |= (1<<ELECTRODE_RED); //Set touch event variable
    g32DebounceCounter[ELECTRODE_RED]--; //Decrement debounce counter
    if(!g32DebounceCounter[ELECTRODE_RED]) //If debounce counter reaches 0....
    {
      lValidTouch |= ((1<<ELECTRODE_RED)); //Signal that a valid touch has been detected
      TouchEvent &= ~(1<<ELECTRODE_RED);  //Clear touch event variable
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE_RED); //Clear touch event variable
    g32DebounceCounter[ELECTRODE_RED] = DBOUNCE_COUNTS; //Reset debounce counter
  }
  /***********************/

  /* Process electrode 1 */
  l16Counter = ELECTRODE_GREEN_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE_GREEN])
  {
    TouchEvent |= (1<<ELECTRODE_GREEN);
    g32DebounceCounter[ELECTRODE_GREEN]--;
    if(!g32DebounceCounter[ELECTRODE_GREEN])
    {
      lValidTouch |= ((1<<ELECTRODE_GREEN));
      TouchEvent &= ~(1<<ELECTRODE_GREEN);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE_GREEN);
    g32DebounceCounter[ELECTRODE_GREEN] = DBOUNCE_COUNTS;
  }
  /***********************/

  /* Process electrode 2 */
  l16Counter = ELECTRODE_YELLOW_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE_YELLOW])
  {
    TouchEvent |= (1<<ELECTRODE_YELLOW);
    g32DebounceCounter[ELECTRODE_YELLOW]--;
    if(!g32DebounceCounter[ELECTRODE_YELLOW])
    {
      lValidTouch |= ((1<<ELECTRODE_YELLOW));
      TouchEvent &= ~(1<<ELECTRODE_YELLOW);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE_YELLOW);
    g32DebounceCounter[ELECTRODE_YELLOW] = DBOUNCE_COUNTS;
  }
  /***********************/

  /* Process electrode 3 */
  l16Counter = ELECTRODE_BLUE_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE_BLUE])
  {
    TouchEvent |= (1<<ELECTRODE_BLUE);
    g32DebounceCounter[ELECTRODE_BLUE]--;
    if(!g32DebounceCounter[ELECTRODE_BLUE])
    {
      lValidTouch |= ((1<<ELECTRODE_BLUE));
      TouchEvent &= ~(1<<ELECTRODE_BLUE);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE_BLUE);
    g32DebounceCounter[ELECTRODE_BLUE] = DBOUNCE_COUNTS;
  }
  /***********************/

  if(lValidTouch&((1<<ELECTRODE_RED))) //If detected a valid touch...
  {
    LED_RED_TOGGLE; //Toggle LED
    lValidTouch &= ~((1<<ELECTRODE_RED)); //Clear valid touch
  }
  if(lValidTouch&((1<<ELECTRODE_GREEN)))
  {
    LED_GREEN_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE_GREEN));
  }
  if(lValidTouch&((1<<ELECTRODE_YELLOW)))
  {
    LED_YELLOW_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE_YELLOW));
  }
  if(lValidTouch&((1<<ELECTRODE_BLUE)))
  {
    LED_BLUE_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE_BLUE));
  }

  TSI0_WUCNTR = 0xFFFFFFFF;
}

