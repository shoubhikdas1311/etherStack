
#include "../common/common.h"
#include "SD_GPIO.h"
#include "SD_Delay.h"

/* ----------------------------------------------------------------------------
   -- TSI
   ---------------------------------------------------------------------------- */

/*! \addtogroup TSI_Peripheral TSI */
/*! \{ */

/*! TSI - Peripheral register structure */
typedef struct TSI0_MemMap {
  uint32_t GENCS;                                  /*!< General Control and Status Register, offset: 0x0 */
  uint32_t SCANC;                                  /*!< SCAN control register, offset: 0x4 */
  uint32_t PEN;                                    /*!< Pin enable register, offset: 0x8 */
  uint32_t STATUS;                                 /*!< Status Register, offset: 0xC */
  uint8_t RESERVED_0[240];
  uint32_t CNTR1;                                  /*!< Counter Register, offset: 0x100 */
  uint32_t CNTR3;                                  /*!< Counter Register, offset: 0x104 */
  uint32_t CNTR5;                                  /*!< Counter Register, offset: 0x108 */
  uint32_t CNTR7;                                  /*!< Counter Register, offset: 0x10C */
  uint32_t CNTR9;                                  /*!< Counter Register, offset: 0x110 */
  uint32_t CNTR11;                                 /*!< Counter Register, offset: 0x114 */
  uint32_t CNTR13;                                 /*!< Counter Register, offset: 0x118 */
  uint32_t CNTR15;                                 /*!< Counter Register, offset: 0x11C */
  uint32_t THRESHLD[16];                           /*!< Channel n threshold register, array offset: 0x120, array step: 0x4 */
} volatile *TSI0_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- TSI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*! \addtogroup TSI_Register_Accessor_Macros TSI - Register accessor macros */
/*! \{ */
#define TSI_THRESHLD_REG(base,index)             ((base)->THRESHLD[index])
#define TSI_BASE_PTR                            ((TSI0_MemMapPtr)0x40045000u)
#define TSI0_THRESHLD0                           TSI_THRESHLD_REG(TSI_BASE_PTR,0)
#define TSI0_THRESHLD1                           TSI_THRESHLD_REG(TSI_BASE_PTR,1)
#define TSI0_THRESHLD2                           TSI_THRESHLD_REG(TSI_BASE_PTR,2)
#define TSI0_THRESHLD3                           TSI_THRESHLD_REG(TSI_BASE_PTR,3)
#define TSI0_THRESHLD4                           TSI_THRESHLD_REG(TSI_BASE_PTR,4)
#define TSI0_THRESHLD5                           TSI_THRESHLD_REG(TSI_BASE_PTR,5)
#define TSI0_THRESHLD6                           TSI_THRESHLD_REG(TSI_BASE_PTR,6)
#define TSI0_THRESHLD7                           TSI_THRESHLD_REG(TSI_BASE_PTR,7)
#define TSI0_THRESHLD8                           TSI_THRESHLD_REG(TSI_BASE_PTR,8)
#define TSI0_THRESHLD9                           TSI_THRESHLD_REG(TSI_BASE_PTR,9)
#define TSI0_THRESHLD10                          TSI_THRESHLD_REG(TSI_BASE_PTR,10)
#define TSI0_THRESHLD11                          TSI_THRESHLD_REG(TSI_BASE_PTR,11)
#define TSI0_THRESHLD12                          TSI_THRESHLD_REG(TSI_BASE_PTR,12)
#define TSI0_THRESHLD13                          TSI_THRESHLD_REG(TSI_BASE_PTR,13)
#define TSI0_THRESHLD14                          TSI_THRESHLD_REG(TSI_BASE_PTR,14)
#define TSI0_THRESHLD15                          TSI_THRESHLD_REG(TSI_BASE_PTR,15)
#define TSI_SCANC_DELVOL_MASK                    0x70000u
#define TSI_SCANC_DELVOL_SHIFT                   16
#define TSI_SCANC_DELVOL(x)                      (((uint32_t)(((uint32_t)(x))<<TSI_SCANC_DELVOL_SHIFT))&TSI_SCANC_DELVOL_MASK)

#define ELECTRODE_RED  							0
#define ELECTRODE_GREEN  						1
#define ELECTRODE_YELLOW  						2
#define ELECTRODE_BLUE  						3

#define ELECTRODE_RED_TOUCH  					0x200
#define ELECTRODE_GREEN_TOUCH  					0x200
#define ELECTRODE_YELLOW_TOUCH  				0x200
#define ELECTRODE_BLUE_TOUCH  					0x200

#define ELECTRODE_RED_OVRRUN  					0xf000
#define ELECTRODE_GREEN_OVRRUN  				0xf000
#define ELECTRODE_YELLOW_OVRRUN  				0xf000
#define ELECTRODE_BLUE_OVRRUN  					0xf000

/* Number of scans needed for a touch to remain high to be considered valid */
#define DBOUNCE_COUNTS  						0x00000010

#define ELECTRODE_ENABLE_REG    				TSI0_PEN

//#ifdef CPU_MK60N512VMD100

  #define ELECTRODE_RED_COUNT  					(uint16)((TSI0_CNTR5>>16)&0x0000FFFF)
  #define ELECTRODE_GREEN_COUNT  				(uint16)((TSI0_CNTR7>>16)&0x0000FFFF)
  #define ELECTRODE_YELLOW_COUNT  				(uint16)((TSI0_CNTR9)&0x0000FFFF)
  #define ELECTRODE_BLUE_COUNT  				(uint16)((TSI0_CNTR9>>16)&0x0000FFFF)

  #define ELECTRODE_RED_OVERRUN  				TSI0_THRESHLD5
  #define ELECTRODE_GREEN_OVERRUN  				TSI0_THRESHLD7
  #define ELECTRODE_YELLOW_OVERRUN  			TSI0_THRESHLD8
  #define ELECTRODE_BLUE_OVERRUN  				TSI0_THRESHLD9

  #define ELECTRODE_RED_EN_MASK  				TSI_PEN_PEN5_MASK
  #define ELECTRODE_GREEN_EN_MASK  				TSI_PEN_PEN7_MASK
  #define ELECTRODE_YELLOW_EN_MASK  			TSI_PEN_PEN8_MASK
  #define ELECTRODE_BLUE_EN_MASK  				TSI_PEN_PEN9_MASK

//#else

//  #define ELECTRODE_RED_COUNT  					(uint16)((TSI0_CNTR9>>16)&0x0000FFFF)
//  #define ELECTRODE_GREEN_COUNT  				(uint16)((TSI0_CNTR11)&0x0000FFFF)
//  #define ELECTRODE_YELLOW_COUNT  				(uint16)((TSI0_CNTR11>>16)&0x0000FFFF)
//  #define ELECTRODE_BLUE_COUNT  				(uint16)((TSI0_CNTR13)&0x0000FFFF)

//  #define ELECTRODE_RED_OVERRUN(base)  			TSI0_THRESHLD5  
//  #define ELECTRODE_GREEN_OVERRUN(base)  		TSI_THRESHOLD_REG(base) 
//  #define ELECTRODE_YELLOW_OVERRUN(base)  		TSI_THRESHOLD_REG(base) 
//  #define ELECTRODE_BLUE_OVERRUN(base)  		TSI_THRESHOLD_REG(base) 

//  #define ELECTRODE_RED_EN_MASK  				TSI_PEN_PEN9_MASK
//  #define ELECTRODE_GREEN_EN_MASK  				TSI_PEN_PEN10_MASK
//  #define ELECTRODE_YELLOW_EN_MASK  			TSI_PEN_PEN11_MASK
//  #define ELECTRODE_BLUE_EN_MASK  				TSI_PEN_PEN12_MASK

//#endif


#define START_SCANNING  						TSI0_GENCS |= TSI_GENCS_STM_MASK
#define ENABLE_EOS_INT  						TSI0_GENCS |= (TSI_GENCS_TSIIE_MASK|TSI_GENCS_ESOR_MASK)
#define ENABLE_TSI      						TSI0_GENCS |= TSI_GENCS_TSIEN_MASK
#define DISABLE_TSI     						TSI0_GENCS &= ~TSI_GENCS_TSIEN_MASK


void TSI_Init(void);
void TSI_isr(void);
void TSI_SelfCalibration(void);