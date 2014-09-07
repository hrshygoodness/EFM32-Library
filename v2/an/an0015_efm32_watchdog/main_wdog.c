/*****************************************************************************
 * @file main_wdog.c
 * @brief Watchdog Demo Application
 * @author Silicon Labs
 * @version 1.08
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


#include "em_device.h"
#include "em_wdog.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_chip.h"
#include "segmentlcd.h"
#include "rtcdrv.h"


/* GLOBAL VARIABLES */

/* Defining the watchdog initialization data */
WDOG_Init_TypeDef init =
{
  .enable     = true,               /* Start watchdog when init done */
  .debugRun   = false,              /* WDOG not counting during debug halt */
  .em2Run     = true,               /* WDOG counting when in EM2 */
  .em3Run     = true,               /* WDOG counting when in EM3 */
  .em4Block   = false,              /* EM4 can be entered */
  .swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
  .lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
  .clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
  .perSel     = wdogPeriod_2k,      /* Set the watchdog period to 2049 clock periods (ie ~2 seconds)*/
};

unsigned long resetCause;
unsigned int  i;

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *
 * The purpose of this example is to demonstrate some basic functionality of the
 * WDOG, and while doing so write feedback to the LCD to make it easier to follow.
 * After initialization and corresponding enabling of the WDOG, the program
 * enters a loop where feedback is written to the LCD, while the WDOG timer is
 * cleared timely. When this is done some time, the system intentionally enters
 * a stalemate, and stops clearing the WDOG. The WDOG then generates a system
 * reset. The cause of the last reset is detected during initialization, and
 * the system now enters a stalemate without enabling the WDOG. When running
 * debug, the debugger is not able to keep track when the WDOG resets the system.
 * This will cause undefined behavior of the debug session.
 *
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Store the cause of the last reset, and clear the reset cause register */
  resetCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  /* Initialize LCD controller without boost */
  SegmentLCD_Init(false);

  /* Enabling clock to the interface of the low energy modules (including the Watchdog)*/
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Check if the watchdog triggered the last reset */
  if (resetCause & RMU_RSTCAUSE_WDOGRST)
  {
    /* Write feedback to lcd */
    SegmentLCD_Write("WDOG");
    /* Stay in this loop forever */
    while (1) ;
  }

  /* Initializing watchdog with choosen settings */
  WDOG_Init(&init);

  /* Do something for a while and make sure that the watchdog does not time out */
  SegmentLCD_Write("FEEDING");
  for (i = 0; i < 40; i++)
  {
    /* Processing takes place here */
    
    RTCDRV_Delay(50, false);
    WDOG_Feed();
  }

  /* Stop feeding the watchdog, and it will trigger a reset */
  SegmentLCD_Write("WAIT");

  /* Enter loop, and wait for wdog reset */
  while (1) ;
}
