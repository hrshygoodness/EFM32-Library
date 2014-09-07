/**************************************************************************//**
 * @file anim_timer.c
 * @brief Animation Timer. Used to time display updates. 
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
#include "em_device.h"
#include "em_letimer.h"
#include "em_cmu.h"
#include "anim_timer.h"

/* Flag to check when we should redraw a frame */
static volatile bool updateDisplay = true;


/* Starts the RTC and gives periodic interrupts which 
 * will set the updateDisplay flag. The interrupt 
 * period will be frameDelayMs */
void startAnimTimer(uint32_t frameDelayMs)
{   
  CMU_ClockEnable(cmuClock_RTC, true);  
  
  /* Set the COMP0 to match the given time */
  RTC->CNT = 0;
  RTC->COMP0 = (frameDelayMs * 32768) / 1000;
  
  /* Wrap around on COMP0 match */
  RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_EN;
  
  /* Enable interrupt */
  RTC->IEN = RTC_IEN_COMP0;
  NVIC_EnableIRQ(RTC_IRQn);
  
  updateDisplay = true;
}


/* Stop the animation timer */
void stopAnimTimer(void)
{
  RTC->CTRL = 0;
  CMU_ClockEnable(cmuClock_RTC, false);
}

void RTC_IRQHandler(void)
{
  RTC->IFC = RTC_IFC_COMP0;
  
  updateDisplay = true;
}

/* Code should call this function whenever the display has
 * been updated */
void displayUpdated(void)
{
  updateDisplay = false;
}

/* Code should check this function to see if an animation 
 * timer interrupt has occurred and it is time to 
 * update the display */
bool shouldUpdateDisplay(void)
{
  return updateDisplay;
}

/* Manually request a new display by calling this function */
void requestDisplayUpdate(void)
{
  updateDisplay = true;
}
