/***************************************************************************//**
 * @file ssd2119.h
 * @brief SSD2119 LCD controller driver
 * @author Silicon Labs
 * @version 1.06
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
#ifndef __SSD2119_H
#define __SSD2119_H

#include <stdbool.h>
#include "em_part.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

#ifdef SSD2119_REGISTER_ACCESS_HOOKS
  #define LCD_DEVFUNC_WRITEREGISTER 0x30
  #define LCD_DEVFUNC_READREGISTER  0x31
  #define LCD_DEVFUNC_WRITEDATA     0x32
  #define LCD_DEVFUNC_READDATA      0x33
#endif

#define LCD_DEVFUNC_INITIALIZE    0x34
#define LCD_DEVFUNC_CONTRADDR     0x35

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* __SSD2119_H */
