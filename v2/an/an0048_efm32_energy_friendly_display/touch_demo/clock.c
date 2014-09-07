/**************************************************************************//**
 * @file clock.c
 * @brief Keeps track of the current time
 * @author Silicon Labs
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
#include <time.h>
#include "em_device.h"
#include "em_burtc.h"
#include "clock.h"

#define COUNTS_PER_SEC 256


/* The current time reference. Number of seconds since midnight
 * January 1, 1970.  */
static time_t startTime = 1357810550;


/* Init the BURTC to keep track of the current time. 
 * The time is found from the sum of the starTime and the 
 * counter value. No check for overflow is done here which 
 * means the clock will wrap around after roughly 194 days. 
 * See AN0041 for an example on how to handle overflows */
void initClock(void)
{
  
  RMU->CTRL &= ~RMU_CTRL_BURSTEN;
  
  /* Create burtcInit struct and fill with default values */ 
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

  burtcInit.enable       = true;
  burtcInit.mode         = burtcModeEM4;
  burtcInit.debugRun     = false;
  burtcInit.clkSel       = burtcClkSelLFRCO;
  burtcInit.clkDiv       = burtcClkDiv_128;
  burtcInit.timeStamp    = true;
  burtcInit.compare0Top  = false;
  burtcInit.lowPowerMode = burtcLPEnable;
  
  /* Initialize BURTC with burtcInit struct */
  BURTC_Init( &burtcInit );
}  

/* Get the current time */
struct tm *getTime(void)
{
  time_t curTime = startTime + BURTC->CNT / COUNTS_PER_SEC;
  return localtime(&curTime);
}

/* Get the current timer minus one second. 
 * This is used to make some of the animations look smooth */
struct tm *getTimeMinusOne(void)
{
  time_t curTime = startTime + BURTC->CNT / COUNTS_PER_SEC - 1;
  return localtime(&curTime);
}
