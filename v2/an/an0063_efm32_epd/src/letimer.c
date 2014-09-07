/*****************************************************************************
 * @file letimer.c
 * @brief Configures LETIMER to give periodic interrupts for slideshow
 * @author Silicon Labs
 * @version 1.02
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
#include "main.h"
#include "letimer.h"

#define LETIMER_FREQ (32768/1024)
#define DELAY_S      5

/**********************************************************
 * Configures LETIMER0 to overflow with an interrupt
 * at a regular interval. The overflow period is set
 * by DELAY_S. This function does not start the timer. 
 **********************************************************/
void letimerInit(void)
{
  CMU_ClockEnable(cmuClock_LETIMER0, true);
  
  /* Prescale clock */
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_1024);
  
  /* Make counter reload COMP0 value on underflow */
  LETIMER0->CTRL |= LETIMER_CTRL_COMP0TOP;
  
  /* Set the top value */
  LETIMER0->COMP0 =  (LETIMER_FREQ * DELAY_S) - 1;
  
  /* Enable interrupt on underflow (LETIMER counts down) */
  LETIMER0->IEN = LETIMER_IEN_UF;
  NVIC_EnableIRQ(LETIMER0_IRQn);
}


/**********************************************************
 * Starts LETIMER0
 **********************************************************/
void letimerStart(void)
{
  LETIMER_Enable(LETIMER0, true);
}

/**********************************************************
 * Stops LETIMER0
 **********************************************************/
void letimerStop(void)
{
  LETIMER_Enable(LETIMER0, false);
}

/**********************************************************
 * Interrupt handler for LETIMER0. Flips to the next
 * screen when triggered. 
 **********************************************************/
void LETIMER0_IRQHandler(void)
{
  /* Clear interrupt flags */
  LETIMER0->IFC = LETIMER0->IF;
  
  /* Switch to next screen */
  nextScreen();
}
