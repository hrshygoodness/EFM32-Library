/*******************************************************************************
 * @file delay.c
 * @brief Delay functions
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

#include "em_device.h"
#include "em_timer.h"
#include "em_cmu.h"

#define RTC_FREQ 32768
#define TIMER_FREQ 48000000

/**********************************************************
 * Enables clocks used for delay functions.
 **********************************************************/
void initDelay(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}


/**********************************************************
 * Delay a number of milliseconds
 **********************************************************/
void delayMs(int ms)
{
  uint32_t endValue = ms * RTC_FREQ / 1000;
  RTC->CNT = 0;
  
  RTC->CTRL |= RTC_CTRL_EN;
  
  while ( RTC->CNT < endValue );
  
  RTC->CTRL &= ~RTC_CTRL_EN;
}
  
/**********************************************************
 * Delay a number of microseconds
 **********************************************************/
void delayUs(int us)
{
  uint32_t endValue = us * (TIMER_FREQ / 1000000);
  TIMER0->CNT = 0;

  TIMER0->CMD = TIMER_CMD_START;
  
  while ( TIMER0->CNT < endValue );
  
  TIMER0->CMD = TIMER_CMD_STOP;
}
  
