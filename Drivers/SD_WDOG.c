/**
  ******************************************************************************
  * @file    wdog.c
  * @author  YANDLD
  * @version V2.5
  * @date    2014.3.24
  * @brief   www.beyondcore.net  
  ******************************************************************************
  */
#include "SD_WDOG.h"
//#include "common.h"
static uint16_t *chawkidarInt = 0;
extern uint16_t wdogIntCnt;
void (* WDOG_CallBackTable)();// = 0x00;//{NULL};
uint8_t Watchdog_IRQn = 22;

static void WDOG_Unlock(void)
{
	uint8_t i;
    WDOG_UNLOCK = 0xC520u;
    WDOG_UNLOCK = 0xD928u;
}

void WDOG_QuickInit(uint32_t timeInMs)
{
    WDOG_InitTypeDef WDOG_InitStruct1;
    WDOG_InitStruct1.mode = kWDOG_Mode_Normal;
    WDOG_InitStruct1.timeOutInMs = timeInMs;
    WDOG_InitStruct1.windowInMs = timeInMs/2;
    WDOG_Init(&WDOG_InitStruct1);
}

void WDOG_Init(WDOG_InitTypeDef* WDOG_InitStruct)
{
    uint32_t clock = 100000000;
    uint32_t time_out;
    //CLOCK_GetClockFrequency(kBusClock, &clock);
    uint32_t wdag_value = 0x01D3u;
    switch((uint32_t)WDOG_InitStruct->mode)
    {
        case kWDOG_Mode_Normal:
            wdag_value &= ~WDOG_STCTRLH_WINEN_MASK;
            break;
        case kWDOG_Mode_Window:
            wdag_value |= WDOG_STCTRLH_WINEN_MASK;
            break;		
        default:
            break;
    }
    WDOG_Unlock();
    /* set timeout value */
    time_out = ((clock/8)/1000)*(WDOG_InitStruct->timeOutInMs);
    WDOG_TOVALH = (time_out & 0xFFFF0000)>>16;
    WDOG_TOVALL = (time_out & 0x0000FFFF)>>0;
    /* set window time value :timeout must greater then window time */
    time_out = ((clock/8)/1000)*(WDOG_InitStruct->windowInMs);
    WDOG_WINH = (time_out & 0xFFFF0000)>>16;
    WDOG_WINL = (time_out & 0x0000FFFF)>>0;
    WDOG_PRESC = WDOG_PRESC_PRESCVAL(7); // perscale = 8
    /* enable wdog */
    wdag_value |= WDOG_STCTRLH_WDOGEN_MASK | WDOG_STCTRLH_IRQRSTEN_MASK | WDOG_STCTRLH_TESTSEL_MASK;
//	wdag_value &= ~WDOG_STCTRLH_CLKSRC_MASK;
    WDOG_STCTRLH = wdag_value;
    WDOG_STCTRLH &= ~WDOG_STCTRLH_IRQRSTEN_MASK;
//	WDOG_ClearResetCounter();
}


void WDOG_ITDMAConfig(uint8_t NewState)
{
    WDOG_Unlock();
//    (NewState == 0x1)?(WDOG_STCTRLH |= WDOG_STCTRLH_IRQRSTEN_MASK):(WDOG_STCTRLH &= ~(WDOG_STCTRLH_IRQRSTEN_MASK));
//    (0x1 == NewState)?(u32fnNVIC_EnableIRQ(Watchdog_IRQn)):(u32fnNVIC_DisableIRQ(Watchdog_IRQn));
//    WDOG_STCTRLH |= WDOG_STCTRLH_IRQRSTEN_MASK;
    enable_irq(22);
}

void WDOG_CallbackInstall(WDOG_CallBackType AppCBFun)
{
    if(AppCBFun != 0x0)
    {
        WDOG_CallBackTable = AppCBFun;
    }
}

uint32_t WDOG_GetResetCounter(void)
{
    return (WDOG_RSTCNT);
}

void WDOG_ClearResetCounter(void)
{
    WDOG_RSTCNT = WDOG_RSTCNT_RSTCNT_MASK;
}

uint32_t WDOG_GetCurrentCounter(void)
{
    uint32_t val;
    val = (WDOG_TMROUTH << 16);
    val |= WDOG_TMROUTL;
    return val;
}

void WDOG_Refresh(void)
{
    uint32_t i;
    DisableInterrupts;
	WDOG_REFRESH = 0xA602u;
	WDOG_REFRESH = 0xB480u;
    EnableInterrupts;
    /* a gap of more then 20 bus cycle between 2 refresh sequence 
    for(i = 0; i < 20; i++)
    {
    }*/
}

void Watchdog_IRQHandler(void)
{
	WDOG_STCTRLL |= WDOG_STCTRLL_INTFLG_MASK;
	wdogIntCnt++;
	LED_GREEN_TOGGLE;
	
    //(* WDOG_CallBackTable)();  
    /*if(WDOG_CallBackTable)
    {
        WDOG_CallBackTable();
    }*/
}

