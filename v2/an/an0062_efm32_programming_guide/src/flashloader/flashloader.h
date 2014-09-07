/**************************************************************************//**
 * @file flashloader.h
 * @brief Universal flashloader for EFM32 devices
 * @author Silicon Labs
 * @version 1.03
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
#ifndef FLASHLOADER_H
#define FLASHLOADER_H

#include <stdint.h>

#define STATE_LOCATION 0x20000010

/* This is the flashloader status, set by the flashloader itself. */
#define FLASHLOADER_STATUS_NOT_READY 0
#define FLASHLOADER_STATUS_READY     1

#define FLASHLOADER_STATUS_ERROR_HARDFAULT    100
#define FLASHLOADER_STATUS_ERROR_NMI          101

#define FLASHLOADER_STATUS_ERROR_TIMEOUT      102
#define FLASHLOADER_STATUS_ERROR_LOCKED       103
#define FLASHLOADER_STATUS_ERROR_INVALIDADDR  104
#define FLASHLOADER_STATUS_ERROR_UNALIGNED    105
#define FLASHLOADER_STATUS_ERROR_UNKNOWN      106

#define DEBUGGERCMD_NOT_CONNECTED   0
#define DEBUGGERCMD_NONE            1
#define DEBUGGERCMD_ERASE_PAGE      2
#define DEBUGGERCMD_WRITE_DATA1     3
#define DEBUGGERCMD_WRITE_DATA2     4
#define DEBUGGERCMD_MASS_ERASE      5


typedef struct
{
  volatile uint32_t flashLoaderStatus;  /* Set by the flashloader */
  volatile uint32_t debuggerStatus;     /* Set by debugger. */ 
  volatile uint32_t bufferSize;         /* Size of available buffers */
  volatile uint32_t bufferAddress1;     /* Address of buffer 1 */
  volatile uint32_t bufferAddress2;     /* Address of buffer 2 */
  volatile uint32_t writeAddress1;       /* Start address to write */
  volatile uint32_t writeAddress2;       /* Start address to write */
  volatile uint32_t numBytes1;          /* Number of bytes to write to flash from buffer 1 */
  volatile uint32_t numBytes2;          /* Number of bytes to write to flash from buffer 2 */
  volatile uint32_t pageSize;           /* Size of one flash page */
  volatile uint32_t sramSize;           /* Size of SRAM */
  volatile uint32_t flashSize;          /* Size of internal flash */
  volatile uint32_t partFamily;         /* EFM32 Device Family Id */
  volatile uint32_t prodRev;            /* Production rev. */
  
} flashLoaderState_TypeDef;

#endif