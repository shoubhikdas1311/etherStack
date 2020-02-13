/*
 * File:        common.h
 * Purpose:     File to be included by all project files
 *
 * Notes:
 */
 

#ifndef _COMMON_H_
#define _COMMON_H_

/********************************************************************/

/*
 * Misc. Defines
 */
#ifdef	FALSE
#undef	FALSE
#endif
#define FALSE	(0)

#ifdef	TRUE
#undef	TRUE
#endif
#define	TRUE	(1)

#ifdef	NULL
#undef	NULL
#endif
#define NULL	(0)

#ifdef  ON
#undef  ON
#endif
#define ON      (1)

#ifdef  OFF
#undef  OFF
#endif
#define OFF     (0)
/*
 * Debug prints ON (#define) or OFF (#undef)
 */
#define DEBUG
#define DEBUG_PRINT

//#define JTAG_UART	UART2_BASE_PTR
//#define JTAG_UART	UART1_BASE_PTR
#define JTAG_UART	UART5_BASE_PTR


#define uint8	uint8_t
#define uint16	uint16_t

#define uint32	uint32_t

/* 
 * Include the generic CPU header file 
 */
#include "arm_cm4.h"

//#include "MK60D10.h"

#include "../Drivers/MK60N512.h"

/* 
 * Include common utilities
 */
#include "assert.h"
#include "io.h"
//#include "startup.h"
//#include "stdlib.h"


/********************************************************************/

#endif /* _COMMON_H_ */
