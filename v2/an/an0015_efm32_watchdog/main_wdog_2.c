/*************************************************************************//**
 * @file main_wdog_2.c
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
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_wdog.h"
#include "em_rmu.h"
#include "em_chip.h"
#include "segmentlcd.h"


/* DEFINES */

#define WAKEUP_INTERVAL_MS    50


/* GLOBAL VARIABLES */

/* Set up RTC init struct */
RTC_Init_TypeDef rtcInit =
{
  .debugRun = false,
  .comp0Top = true,
  .enable   = true,
};

/* Defining the watchdog initialization data */
WDOG_Init_TypeDef init =
{
  .enable     = false,            /* Do not start watchdog when init done */
  .debugRun   = false,            /* WDOG not counting during debug halt */
  .em2Run     = true,             /* WDOG counting when in EM2 */
  .em3Run     = false,             /* WDOG counting when in EM3 */
  .em4Block   = false,            /* EM4 can be entered */
  .swoscBlock = true,             /* Block disabling LFRCO/LFXO in CMU */
  .lock       = false,            /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
  .clkSel     = wdogClkSelLFXO,   /* Select the 32.768kHZ LFXO oscillator */
  .perSel     = wdogPeriod_64k,   /* Set the watchdog period to 65537 clock periods (ie ~2 seconds)*/
};

unsigned long resetCause;
unsigned int  i;
uint32_t      rtcCountBetweenWakeup;


/**************************************************************************//**
 * @brief RTC Interrupt Handler. Clears interrupt flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *
 * The purpose of the RTC interrupt is to wake the CPU from deep sleep mode EM2
 * at given intervals. This is a way to save energy, while ensuring that the
 * CPU often enough checks if there are any other instructions ready to executed.
 *
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
}


/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *
 * This example is similar to the first, but in addition demonstrates more of
 * the functionality available for the WDOG. The functionality demonstrated are
 * the use of a clock source different from the ULFRCO, register and oscillator
 * locking  and a way to keep the system mainly in EM2 while the WDOG is still
 * running. A RTC clock is used to wake the system from EM2 regularly.
 *
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Store the cause of the last reset, and clear the reset cause register */
  resetCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  /* Starting LFRCO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  /* Enable the RTC clock */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Input RTC init struct in initialize funciton */
  RTC_Init(&rtcInit);

  /* Set RTC compare value */
  rtcCountBetweenWakeup = ((SystemLFRCOClockGet() * WAKEUP_INTERVAL_MS) / 1000);
  RTC_CompareSet(0, rtcCountBetweenWakeup);

  /* Enable RTC interrupt from COMP0 */
  RTC_IntEnable(RTC_IF_COMP0);

  /* Enable RTC interrupt vector in NVIC */
  NVIC_EnableIRQ(RTC_IRQn);

  /* Enable RTC */
  RTC_Enable(true);

  /* Starting LFXO and waiting in EM2 until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);
  while (!(CMU->STATUS & CMU_STATUS_LFXORDY))
  {
    EMU_EnterEM2(false);
  }

  /* Initialize LCD controller without boost*/
  SegmentLCD_Init(false);

  /* Check if the watchdog triggered the last reset */
  if (resetCause & RMU_RSTCAUSE_WDOGRST)
  {
    /* Write feedback to lcd */
    SegmentLCD_Write("WDOG");

    /* Stay in this loop forever */
    while (1)
    {
      EMU_EnterEM2(false);
    }
  }

  /* Initializing watchdog with choosen settings */
  WDOG_Init(&init);

  /* Enabling watchdog, since it was not enabled during initialization */
  WDOG_Enable(true);

  /* Locking watchdog register (reset needed to unlock) */
  WDOG_Lock();

  /* Oscillator is locked, and can not be disabled */
  CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

  /* Verifying that the LFXO is still running*/
  while (!(CMU->STATUS & CMU_STATUS_LFXOENS)) ;

  /* Register is locked, can not be modified */
  while (WDOG->SYNCBUSY & WDOG_SYNCBUSY_CTRL) ;
  WDOG->CTRL |= WDOG_CTRL_EM3RUN;

  /* Verifying that the EM3RUN bit is not set */
  while (WDOG->CTRL & WDOG_CTRL_EM3RUN) ;

  /* Do something for a while and make sure that the watchdog does not time out */
  SegmentLCD_Write("FEEDING");
  for (i = 0; i < 40; i++)
  {
    /* Enter EM2 while the watchdog is still counting */
    EMU_EnterEM2(false);

    /* Processing takes place here */

    WDOG_Feed();
  }

  /* Stop feeding the watchdog */
  SegmentLCD_Write("WAIT");

  /* Enter loop, and wait for wdog reset in EM2 */
  while (1)
  {
    EMU_EnterEM2(false);
  }
}
