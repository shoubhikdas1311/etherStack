/*
 * Sys.h
 *
 *  Created on: Nov 19, 2019
 *      Author: SD
 */

#ifndef SYS_H_
#define SYS_H_

#include "SD_GPIO.h"

  /*!< Macro to enable all interrupts. */
#define EnableInterrupts __asm(" CPSIE i");

  /*!< Macro to disable all interrupts. */
#define DisableInterrupts __asm(" CPSID i");

void jiffiesInit();
void launchJiffies();

#endif /* SYS_H_ */
