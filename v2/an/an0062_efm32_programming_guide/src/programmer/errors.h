/*******************************************************************************
 * @file errors.h
 * @brief Simple exception handler for C
 * @author Silicon Labs
 * @version 1.03
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
#ifndef _ERROR_H_
#define _ERROR_H_

#include <setjmp.h>

#define SWD_ERROR_OK                    1
#define SWD_ERROR_WAIT                  2
#define SWD_ERROR_FAULT                 3
#define SWD_ERROR_PROTOCOL              4
#define SWD_ERROR_PARITY                5
#define SWD_ERROR_MCU_LOCKED            6
#define SWD_ERROR_INVALID_IDR           7
#define SWD_ERROR_INVALID_IDCODE        8
#define SWD_ERROR_FLASH_WRITE_FAILED    9
#define SWD_ERROR_UNLOCK_FAILED         10
#define SWD_ERROR_AAP_EXTENSION_FAILED  11
#define SWD_ERROR_LOCK_FAILED           12
#define SWD_ERROR_CLR_DLW_FAILED        13
#define SWD_ERROR_MASS_ERASE_TIMEOUT    14
#define SWD_ERROR_VERIFY_FW_FAILED      15
#define SWD_ERROR_TIMEOUT_WAITING_RESET 16
#define SWD_ERROR_TARGET_NOT_HALTED     17
#define SWD_ERROR_FLASHLOADER_ERROR     18
#define SWD_ERROR_DEVICE_ERASE_FAILED   19
#define SWD_ERROR_TIMEOUT_HALT          20
#define SWD_ERROR_DEBUG_POWER           21

char *getErrorString(int errorCode);

/* Maximum number of nested TRY/CATCH blocks */
#define EXCEPTION_MAX_LEVEL 5

extern jmp_buf swdErrors[];
extern int swdErrorIndex;

/* Raise an error */
#define RAISE(x) longjmp(swdErrors[swdErrorIndex], x)

/* During debugging this macro can be used instead to generate a 
 * breakpoint at the point where an error occurs */
//#define RAISE(x) __asm("bkpt 1");

/* Start TRY/CATCH block */
#define TRY { int errorCode = setjmp(swdErrors[++swdErrorIndex]); \
  if ( errorCode == 0 ) { 

/* Start CATCH clause */
#define CATCH } else {
    
/* End TRY/CATCH block */
#define ENDTRY } swdErrorIndex--;}

#endif