/**************************************************************************//**
 * @file hijack.h
 * @brief Hijack demo for EFM32TG-STK3300
 * @version 1.05
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
#ifndef __HIJACK_H
#define __HIJACK_H

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup App_Notes
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup HiJack
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   MACROS   ************************************
 ******************************************************************************/


/*******************************************************************************
 ****************************   CONFIGURATION   ********************************
 ******************************************************************************/

  
/*******************************************************************************
 ******************************   TYPEDEFS   ***********************************
 ******************************************************************************/

typedef enum
{
  hijackEdgeModeRising = 0,
  hijackEdgeModeFalling = 1,
  hijackEdgeModeBoth = 2
} HIJACK_EdgeMode_t;


typedef enum
{
  hijackOutputModeSet = 0,
  hijackOutputModeClear,
  hijackOutputModeToggle
} HIJACK_OutputMode_t;

/** Task pointer type. */
typedef void (*HIJACK_TxDoneFuncPtr_t)(void);
/*******************************************************************************
 ******************************   PROTOTYPES   *********************************
 ******************************************************************************/
void HIJACK_Init(HIJACK_TxDoneFuncPtr_t pTxDone);
bool HIJACK_ByteTx(uint8_t byte);
void HIJACK_ByteRx(uint8_t *pByte);


/** @} (end addtogroup HiJack) */
/** @} (end addtogroup App_Notes) */

#ifdef __cplusplus
}
#endif

#endif /* __HIJACK_H */
