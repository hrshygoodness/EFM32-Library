/**************************************************************************//**
 * @file eeprom_emulation.h
 * @brief EEPROM Emulation Demo Application header file
 * @author Silicon Labs
 * @version 1.10
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#ifndef __EE_H
#define __EE_H

#include <stdbool.h>
#include "em_device.h"
#include "em_msc.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 ******************************   DEFINES   ************************************
 ******************************************************************************/


#if defined (_EFM32_GECKO_FAMILY) || defined(_EFM32_TINY_FAMILY)
#define PAGE_SIZE                        0x200
#elif defined(_EFM32_GIANT_FAMILY)
#define PAGE_SIZE                       ((FLASH_SIZE >= 0x80000) ? 0x1000 : 0x800)
#else
#pragma error "Unknown page size"
#endif


#define DEFAULT_NUMBER_OF_PAGES          2
#define MAX_NUMBER_OF_PAGES              (FLASH_SIZE/PAGE_SIZE)

#define SIZE_OF_DATA                     2                                        /* 2 bytes */
#define SIZE_OF_VIRTUAL_ADDRESS          2                                        /* 2 bytes */
#define SIZE_OF_VARIABLE                 (SIZE_OF_DATA + SIZE_OF_VIRTUAL_ADDRESS) /* 4 bytes */
   
#define MAX_ACTIVE_VARIABLES             (PAGE_SIZE / SIZE_OF_VARIABLE)

#define PAGE_STATUS_ERASED               0xFF
#define PAGE_STATUS_RECEIVING            0xAA
#define PAGE_STATUS_ACTIVE               0x00


/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/

/* Since the data to be written to flash must be read from ram, the data used to
 * set the pages' status, is explicitly written to the ram beforehand. */
static uint32_t pageStatusActiveValue    = ((uint32_t) PAGE_STATUS_ACTIVE << 24) | 0x00FFFFFF;
static uint32_t pageStatusReceivingValue = ((uint32_t) PAGE_STATUS_RECEIVING << 24) | 0x00FFFFFF;


/*******************************************************************************
 *******************************   ENUMS   *************************************
 ******************************************************************************/

typedef enum
{
  eePageStatusErased    = PAGE_STATUS_ERASED,
  eePageStatusReceiving = PAGE_STATUS_RECEIVING,
  eePageStatusActive    = PAGE_STATUS_ACTIVE,
} EE_PageStatus_TypeDef;


/*******************************************************************************
 ******************************   STRUCTS   ************************************
 ******************************************************************************/

typedef struct
{
  /* Each variable is assigned a unique virtual address automatically when first
   * written to, or when using the declare function. */
  uint16_t virtualAddress;
} EE_Variable_TypeDef;

typedef struct
{
  uint32_t *startAddress;
  uint32_t *endAddress;
} EE_Page_TypeDef;


/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

bool EE_Init(uint32_t);
bool EE_Format(uint32_t);
bool EE_Read(EE_Variable_TypeDef *var, uint16_t *readData);
void EE_Write(EE_Variable_TypeDef *var, uint16_t writeData);
bool EE_DeclareVariable(EE_Variable_TypeDef *var);
void EE_DeleteVariable(EE_Variable_TypeDef *var);
uint32_t EE_GetEraseCount(void);


/***************************************************************************//**
 * @brief
 *   Returns the page status of the given page.
 *
 * @param[in] page
 *   Pointer to the page whose status to be returned.
 *
 * @return
 *   Returns the status of the given page.
 ******************************************************************************/
__STATIC_INLINE EE_PageStatus_TypeDef EE_getPageStatus(EE_Page_TypeDef *page)
{
  return (EE_PageStatus_TypeDef)((*(page->startAddress) >> 24) & 0xFF);
}

/***************************************************************************//**
 * @brief
 *   Sets status of the given page to active.
 *
 * @param[in] page
 *   Pointer to the page whose status to be altered.
 *
 * @return
 *   Returns the status of the flash operation.
 ******************************************************************************/
__STATIC_INLINE msc_Return_TypeDef EE_setPageStatusActive(EE_Page_TypeDef *page)
{
  return MSC_WriteWord(page->startAddress, &pageStatusActiveValue, SIZE_OF_VARIABLE);
}

/***************************************************************************//**
 * @brief
 *   Sets status of the given page to receiving.
 *
 * @param[in] page
 *   Pointer to the page whose status to be altered.
 *
 * @return
 *   Returns the status of the flash operation.
 ******************************************************************************/
__STATIC_INLINE msc_Return_TypeDef EE_setPageStatusReceiving(EE_Page_TypeDef *page)
{
  return MSC_WriteWord(page->startAddress, &pageStatusReceivingValue, SIZE_OF_VARIABLE);
}

#ifdef __cplusplus
}
#endif

#endif /* __EE_H */
