/*****************************************************************************
 * @file main_rtc.c
 * @brief Real Time Counter Demo Application
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
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"

/* Defines*/
#define LFRCO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS            500
#define RTC_COUNT_BETWEEN_WAKEUP    (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)


/**************************************************************************//**
 * @brief RTC Interrupt Handler, clears the flag.
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
}

/**************************************************************************//**
 * @brief  Start LFRCO for RTC
 * Starts the low frequency RC oscillator (LFRCO) and routes it to the RTC
 *****************************************************************************/
void startLfxoForRtc(void)
{
  /* Starting LFRCO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  /* Routing the LFRCO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  
  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
}



/**************************************************************************//**
 * @brief  Setup RTC
 * On compare match with COMP0, clear counter and set interrupt
 *****************************************************************************/
void setupRtc(void)
{
  /* Configuring clocks in the Clock Management Unit (CMU) */
  startLfxoForRtc();
  
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;
  
  rtcInit.enable   = true;      /* Enable RTC after init has run */
  rtcInit.comp0Top = true;      /* Clear counter on compare match */
  rtcInit.debugRun = false;     /* Counter shall keep running during debug halt. */

  /* Setting the compare value of the RTC */
  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

  /* Enabling Interrupt from RTC */
  RTC_IntEnable(RTC_IFC_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);

  /* Initialize the RTC */
  RTC_Init(&rtcInit);
}



/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Setting up rtc */
  setupRtc();

  /* Disable LFB clock domains to save power */
  //CMU->LFCLKSEL &= ~_CMU_LFCLKSEL_LFB_MASK;   
  
  while (1)
  {
    /* Forever enter EM2. */
    EMU_EnterEM2(false);
  }
}
