/**
  ******************************************************************************
  * @file    wdog.h
  * @author  YANDLD
  * @version V2.5
  * @date    2014.3.24
  * @brief   www.beyondcore.net   http://upcmcu.taobao.com 
	* @note    ���ļ�ΪоƬ���Ź�ģ��ĵײ㹦�ܺ���
  ******************************************************************************
  */

#ifndef __CH_LIB_WDOG_H__
#define __CH_LIB_WDOG_H__

#include "../common/common.h"
#include "SD_GPIO.h"
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum
{
    kWDOG_Mode_Normal,            //��ͨģʽ���Ź�
    kWDOG_Mode_Window,            // WDOG ����ģʽ
}WDOG_Mode_Type;


typedef struct __attribute__((__packed__))
{
    WDOG_Mode_Type      mode;                   //WDOG ����ģʽѡ��
    uint32_t            timeOutInMs;            //WDOG ���ι��ʱ��
    uint32_t            windowInMs;             //WDOG ������ʼʱ��
}WDOG_InitTypeDef;

//!< Callback Type
typedef void (*WDOG_CallBackType)(void);

//!< API functions
void WDOG_QuickInit(uint32_t timeInUs);
void WDOG_Init(WDOG_InitTypeDef* WDOG_InitStruct);
void WDOG_ITDMAConfig(uint8_t NewState);
void WDOG_CallbackInstall(WDOG_CallBackType AppCBFun);
uint32_t WDOG_GetResetCounter(void);
void WDOG_ClearResetCounter(void);
void WDOG_Refresh(void);
uint32_t WDOG_GetCurrentCounter(void);
uint8_t wDog_isr_usr();


#endif


