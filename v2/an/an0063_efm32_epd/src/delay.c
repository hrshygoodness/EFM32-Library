/*******************************************************************************
 * @file delay.c
 * @brief Delay functions
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
#include "em_timer.h"
#include "em_cmu.h"
#include "em_emu.h"

#define RTC_FREQ 32768
#define TIMER_FREQ 14000000

#define TIMER_INST TIMER1
#define TIMER_CLK  cmuClock_TIMER1
#define TIMER_IRQHandler TIMER1_IRQHandler
#define TIMER_IRQn TIMER1_IRQn

/**********************************************************
 * Enables clocks used for delay functions.
 **********************************************************/
void delayInit(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(TIMER_CLK, true);
  
  RTC->IEN = RTC_IEN_COMP0;
  NVIC_EnableIRQ(RTC_IRQn);
  
  TIMER_INST->CC[0].CTRL |= TIMER_CC_CTRL_MODE_OUTPUTCOMPARE;
  TIMER_INST->IEN = TIMER_IEN_CC0;
  NVIC_EnableIRQ(TIMER_IRQn);
}




/**********************************************************
 * Delay a number of milliseconds
 **********************************************************/
void delayMs(int ms)
{
  uint32_t endValue = (ms * RTC_FREQ) / 1000;
  RTC->CNT = 0;
  
  RTC->COMP0 = endValue;
  
  RTC->CTRL |= RTC_CTRL_EN;
  
  while ( RTC->CNT < endValue )
  {
    EMU_EnterEM2(false);
  }
  
  RTC->CTRL &= ~RTC_CTRL_EN;
}

/**********************************************************
 * Start the RTC
 **********************************************************/
void rtcStart(void)
{
  RTC->CNT = 0;
  RTC->CTRL |= RTC_CTRL_EN;  
}

/**********************************************************
 * Stop the RTC
 **********************************************************/
void rtcStop(void)
{
  RTC->CTRL &= ~RTC_CTRL_EN;
}

/**********************************************************
 * Returns the number of milliseconds since the 
 * RTC was started
 **********************************************************/
uint32_t rtcGetMs(void)
{
  return (RTC->CNT * 1000) / RTC_FREQ;
}



  
/**********************************************************
 * Delay a number of microseconds
 **********************************************************/
void delayUs(int us)
{
  uint32_t endValue = us * (TIMER_FREQ / 1000000);
  
  /* Clear the counter */
  TIMER_INST->CNT = 0;

  /* Set the end value */
  TIMER_INST->CC[0].CCV = endValue;
  
  /* Start the timer */
  TIMER_INST->CMD = TIMER_CMD_START;
  
  /* Wait until end value is reached */
  while ( TIMER_INST->CNT < endValue )
  {
    /* Enter EM1 to save power */
    EMU_EnterEM1();
  }
  
  /* Stop the timer */
  TIMER_INST->CMD = TIMER_CMD_STOP;
}


void RTC_IRQHandler(void)
{
  /* Clear interrupt flags */
  RTC->IFC = RTC->IF;
}

void TIMER_IRQHandler(void)
{
   /* Clear interrupt flags */
  TIMER_INST->IFC = TIMER_INST->IF;
}
