/***************************************************************************//**
 * @file rtc.c
 * @brief Generates 1s ticks with RTC
 * @author Silicon Labs
 * @version 1.02
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
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"

/* This flags is set in the RTC interrupt handler to tell the
 * appliation that one second has passed */
volatile bool rtcTick = false;

/**********************************************************
 * Configures and starts the RTC. The RTC is configured
 * to generate an interrupt every second.
 **********************************************************/
void rtcStartTick(void)
{
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  /* Set prescaler to 1s per tick */
  CMU->LFAPRESC0 = (CMU->LFAPRESC0 & ~_CMU_LFAPRESC0_RTC_MASK) | CMU_LFAPRESC0_RTC_DIV32768;
  
  /* Wrap around at 1 */
  RTC->COMP0 = 0;
  
  /* Clear counter */
  RTC->CNT = 0;
  
  rtcTick = false;
  
  /* Enable interrupts */
  RTC->IEN = RTC_IEN_COMP0;
  NVIC_EnableIRQ(RTC_IRQn);
  
  /* Start RTC */
  RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_EN;
}

/**********************************************************
 * Stop the RTC
 **********************************************************/
void rtcStopTick(void)
{
  RTC->CTRL &= ~RTC_CTRL_EN;
  CMU_ClockEnable(cmuClock_RTC, false);
}

/**********************************************************
 * Interrupt handler for the RTC. Sets the rtcTick
 * flag.
 **********************************************************/
void RTC_IRQHandler(void)
{
  uint32_t flags = RTC->IF;
  RTC->IFC = flags;
  
  if ( flags & RTC_IF_COMP0 )
  {
    rtcTick = true;
  }
}

void rtcDelayMs(int ms)
{
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  CMU->LFAPRESC0 = (CMU->LFAPRESC0 & ~_CMU_LFAPRESC0_RTC_MASK) | CMU_LFAPRESC0_RTC_DIV1;
  
  rtcTick = false;
  
  RTC->CNT = 0;
  RTC->COMP0 = (ms * 32768) / 1000;
  
  /* Enable interrupts */
  RTC->IEN = RTC_IEN_COMP0;
  NVIC_EnableIRQ(RTC_IRQn);
  
  /* Start RTC */
  RTC->CTRL = RTC_CTRL_EN;
  
  while ( !rtcTick )
  {
    EMU_EnterEM2(true);
  }
  
  /* Stop RTC */
  RTC->CTRL &= ~RTC_CTRL_EN;
  CMU_ClockEnable(cmuClock_RTC, false);
}

