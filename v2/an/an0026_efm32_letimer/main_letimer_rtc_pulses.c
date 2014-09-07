/*****************************************************************************
 * @file main_letimer_rtc_pulses.c
 * @brief LETIMER demo application, LETIMER triggered by RTC
 * @author Silicon Labs
 * @version 1.07
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
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_chip.h"

/* Maximum value for compare registers */
#define LETIMER_TOP 100
#define LETIMER_REPEAT 5
#define RTC_COMP 5

/**************************************************************************//**
 * @brief  RTC_setup
 * Configures and starts the RTC
 *****************************************************************************/
void RTC_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Each RTC tick is 1 second */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32768);
  
  /* Setting up RTC with compare value.
     Compare match will occur at 5 seconds */
  RTC_CompareSet(0, RTC_COMP);
  
  /* Enable RTC */
  RTC_Enable(true);
}

/**************************************************************************//**
 * @brief  LETIMER0_setup
 * Configures and starts the LETIMER
 *****************************************************************************/
void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  /* The CORELE clock is also necessary for the RTC and all
     low energy peripherals, but since this function
     is called before RTC_setup() the clock enable
     is only included here */
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PD6 as push pull so the
     LETIMER can override it */
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
  
  /* Set initial compare values for COMP0 */
  LETIMER_CompareSet(LETIMER0, 0, LETIMER_TOP);
  
  /* Set repetition values, LETIMER will count 5 times and
     thus, generate 5 pulses */
  LETIMER_RepeatSet(LETIMER0, 0, LETIMER_REPEAT);
  
  /* Route LETIMER to location 0 (OUT0 - PD6) and enable output */
  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC0;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = false,                  /* Don't start counting when init completed - only with RTC compare match */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = true,                   /* Start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAPulse,       /* Pulse output on output 0 */
  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
  .repMode        = letimerRepeatOneshot    /* Count while REP != 0 */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}



/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Align different chip revisions */
  CHIP_Init();

  /* Initialize LETIMER */
  LETIMER_setup();
  
  /* Initialize RTC */
  RTC_setup();
  
  while(1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(false);
  }
}
