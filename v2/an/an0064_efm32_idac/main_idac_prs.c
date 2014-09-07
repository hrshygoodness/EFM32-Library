 /******************************************************************************
 * @file main_idac_prs.c
 * @brief IDAC PRS example
 * @author Silicon Labs
 * @version 1.01
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
 
#include <stdint.h>
#include <stdlib.h>
#include "em_device.h"
#include "em_idac.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_gpio.h"

/* Setting TOP value to 500 so that frequency is around 215Hz 
   F = HFPERCLK / ( 2^(PRESC + 1) x TOP + 1)
   where HFPERCLK = 14Mhz and PRESC = 6 */
#define TOP 500

/**************************************************************************//**
 * @brief  setupIdac
 * configures the IDAC for the PRS controlling output on/off
 *****************************************************************************/
void setupIdac()
{
  uint32_t idacStep = 0xF; /* Setting step to middle value */
  
  /* Enable IDAC clock */
  CMU_ClockEnable(cmuClock_IDAC0, true);
  
  IDAC_Init_TypeDef idacInit = 
  {
    .enable = true,
    .outMode = idacOutputPin,
    .prsEnable = true,
    .prsSel = idacPRSSELCh0,
    .sinkEnable = false,
  };
  
  IDAC_Init(IDAC0, &idacInit);
  
  /* Minimize Output Transition */
  IDAC_MinimalOutputTransitionMode(IDAC0, true);
  
  IDAC_RangeSet(IDAC0, idacCurrentRange2);
  IDAC_StepSet(IDAC0, idacStep);
}

/**************************************************************************//**
 * @brief  TIMER_setup
 * configures the TIMER to generate square waveform with 
 * frequency HFPERCLK / ( 2^(PRESC + 1) x TOP + 1)
 *****************************************************************************/
void setupTimer()
{
/* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionToggle,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCompare,
    .filter     = true,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);
  
  /* The PRS channel will follow CC0 out */
  TIMER0->CC[0].CTRL |= TIMER_CC_CTRL_PRSCONF_LEVEL;
  
  /* Set Top Value */
  TIMER_TopSet(TIMER0, TOP); 
  
  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale64,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
  
  /* Configure timer */
  TIMER_Init(TIMER0, &timerInit);  
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  
  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  
  /* Initialize IDAC */
  setupIdac();
  
  /* Initialize timer */
  setupTimer();
  
  /* Select TIMER0 as source and timer CC0 as signal */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0CC0, prsEdgeOff);

  /* Stay in this loop forever at end of program */
  while (1)
  {
    EMU_EnterEM1();    
  }
}
