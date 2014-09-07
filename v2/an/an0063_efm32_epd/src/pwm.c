/*****************************************************************************
 * @file pwm.c
 * @brief Pulse Width Modulation
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
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "config.h"

/* Define PWM frequency value. EPD requires between 100-300 kHz */
#define PWM_FREQ 200000

/* TIMER runs directly off HFCLK */
#define TIMER_FREQ 14000000

/* Configure paramters for PWM channel. These must match the pin
 * selected in config.h */
#define PWM_TIMER           TIMER0
#define PWM_TIMER_CLK       cmuClock_TIMER0 
#define PWM_TIMER_LOCATION  TIMER_ROUTE_LOCATION_LOC3
#define PWM_TIMER_CC        2
#define PWM_TIMER_CCPEN     TIMER_ROUTE_CC2PEN   

/**********************************************************
 * Configures TIMER for PWM output
 **********************************************************/
void pwmInit(void)
{  
  /* Enable clock for GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER */
  CMU_ClockEnable(PWM_TIMER_CLK, true);
   
  /* Configure TIMER CC pin */
  GPIO_PinModeSet(EPD_PIN_PWM, gpioModePushPull, 0);
  
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionClear,
    .cmoa       = timerOutputActionSet,
    .mode       = timerCCModeCompare,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false
  };
  
  /* Configure CC channel */
  TIMER_InitCC(PWM_TIMER, PWM_TIMER_CC, &timerCCInit);

  /* Route CC0 to location 4 (PD6) and enable pin */  
  PWM_TIMER->ROUTE = (PWM_TIMER_CCPEN | PWM_TIMER_LOCATION); 
  
  /* Set Top Value to set frequency */
  TIMER_TopSet(PWM_TIMER, TIMER_FREQ/PWM_FREQ);
  
  /* 50% duty-cycle */
  TIMER_CompareSet(PWM_TIMER, PWM_TIMER_CC, (TIMER_FREQ/PWM_FREQ)/2);

  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
  
  /* Configure PWM_TIMER registers */
  TIMER_Init(PWM_TIMER, &timerInit);
}

/************************************************
 * Starts PWM output 
 ************************************************/
void pwmEnable(void)
{
  TIMER_Enable(PWM_TIMER, true);
}

/************************************************
 * Stops PWM output 
 ************************************************/
void pwmDisable(void)
{
  TIMER_Enable(PWM_TIMER, false);
}
