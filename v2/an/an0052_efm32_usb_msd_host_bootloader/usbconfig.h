/***************************************************************************//**
 * @file usbconfig.h
 * @brief USB protocol stack library, application supplied configuration options.
 * @author Silicon Labs
 * @version 1.04
 *******************************************************************************
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
#ifndef __USBCONFIG_H
#define __USBCONFIG_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USB_HOST            /* Compile stack for host mode. */

/****************************************************************************
**                                                                         **
** Specify number of host channels used (in addition to EP0).              **
**                                                                         **
*****************************************************************************/
#define NUM_HC_USED 2       /* Not counting default control ep which  */
                            /* is assigned to host channels 0 and 1   */

#ifndef NDEBUG
/* Debug USB API functions (illegal input parameters etc.)  */
#define DEBUG_USB_API       /* Uncomment to turn on  */

/*
 * Some utility functions in the API needs printf. These
 * functions have "print" in their name. This macro enables
 * these functions.
 */
#define USB_USE_PRINTF      /* Uncomment to enable   */
#endif

#ifdef GG_STK
/****************************************************************************
**                                                                         **
** Configure USB overcurrent flag                                          **
**                                                                         **
*****************************************************************************/
#define USB_VBUSOVRCUR_PORT       gpioPortF
#define USB_VBUSOVRCUR_PIN        6
#define USB_VBUSOVRCUR_POLARITY   USB_VBUSOVRCUR_POLARITY_LOW
#endif
  
#ifdef __cplusplus
}
#endif

#endif /* __USBCONFIG_H */
