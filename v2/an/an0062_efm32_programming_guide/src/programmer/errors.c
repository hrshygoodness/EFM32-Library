/*******************************************************************************
 * @file errors.c
 * @brief Simple exception model for C
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

/*******************************************************************
 * 
 * This file (and errors.h) implements a simple exception-like
 * system for handling errors in C. By using this we can avoid
 * checking for error return values on every function call.
 * The exception system is based on the library functions setjmp() 
 * and longjmp(). Four macros are defined, TRY, CATCH, ENDRY and RAISE(x). 
 *  
 * The macros are used like this: 
 * 
 * TRY
 *   someFunction();
 *   someOtherFunction();
 * CATCH
 *   printf("Error occurred: %s\n", getErrorString(errorCode));
 * ENDTRY
 * 
 * It is possible to raise an exception like this: 
 * 
 * void someFunction(void)
 * {
 *   .....
 *   if ( someErrorCondition )
 *   {
 *     RAISE(SWD_ERROR_SOME_ERROR);
 *   }
 *   ....
 * }
 * 
 * The error codes are defined in errors.h. After a call to RAISE()
 * The execution will jump to the CATCH block which will receive 
 * the error in a variable called errorCode. The getErrorString() 
 * function can be used to conveniently print an error message. 
 * 
 * TRY/CATCH blocks can be nested. I.e. someFunction() can 
 * include its own TRY/CATCH block. Any RAISE()'ed error will
 * return to the closest CATCH block. The maximum nesting level
 * is specified with EXCEPTION_MAX_LEVEL. 
 * 
 * IMPORTANT
 * 
 *   Do not RAISE() an error within a CATCH block. 
 * 
 *   Exiting early from a TRY block in any other way than
 *   RAISE() WILL break the system and lead to undefined 
 *   behavior. Therefore make sure to never have a return 
 *   statement within the TRY or CATCH blocks. 
 * 
 *****************************************************************/

#include "errors.h"


jmp_buf swdErrors[EXCEPTION_MAX_LEVEL];
int swdErrorIndex = -1;

char *getErrorString(int errorCode)
{
  switch (errorCode)
  {
  case SWD_ERROR_OK:
    return "No error.";
  case SWD_ERROR_WAIT:
    return "Timed out while waiting for WAIT response.";
  case SWD_ERROR_FAULT:
    return "Target returned FAULT response.";
  case SWD_ERROR_PROTOCOL:
    return "Protocol error. Target does not respond.";
  case SWD_ERROR_PARITY:
    return "Parity error.";
  case SWD_ERROR_MCU_LOCKED:
    return "MCU locked.";
  case SWD_ERROR_INVALID_IDR:
    return "Invalid IDR value.";
  case SWD_ERROR_FLASH_WRITE_FAILED:
    return "Write to flash failed.";
  case SWD_ERROR_UNLOCK_FAILED:
    return "Failed to unlock target.";
  case SWD_ERROR_AAP_EXTENSION_FAILED:
    return "Unable to access AAP registers. Is the reset pin connected?";
  case SWD_ERROR_LOCK_FAILED:
    return "Failed to lock target.";
  case SWD_ERROR_CLR_DLW_FAILED:
    return "Failed to clear Debug Lock Word.";
  case SWD_ERROR_MASS_ERASE_TIMEOUT:
    return "Timed out while waiting for Mass Erase to complete.";
  case SWD_ERROR_VERIFY_FW_FAILED:
    return "Application verification failed.";
  case SWD_ERROR_TIMEOUT_WAITING_RESET:
    return "Timeout while waiting for target to reset.";
  case SWD_ERROR_TARGET_NOT_HALTED:
    return "Target is halted.";
  case SWD_ERROR_FLASHLOADER_ERROR:
    return "Error in flashloader.";
  case SWD_ERROR_DEVICE_ERASE_FAILED:
    return "Device Erase failed."; 
  case SWD_ERROR_TIMEOUT_HALT:
    return "Timeout while waiting to halt target.";
  case SWD_ERROR_DEBUG_POWER:
    return "Failed to power up debug interface.";
  case SWD_ERROR_INVALID_IDCODE:
    return "Invalid IDCODE.";
  default:
    return "Unknown error.";
  }
}