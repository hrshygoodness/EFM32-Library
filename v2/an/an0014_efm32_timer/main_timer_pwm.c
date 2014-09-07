/*****************************************************************************
 * @file main_timer_pwm.c
 * @brief TIMER Pulse Width Modulation Demo Application
 * @author Silicon Labs
 * @version 1.09
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

/* Define PWM frequency value */
#define PWM_FREQ 10000

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  uint32_t compareValue;
  
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);

  compareValue = TIMER_CaptureGet(TIMER0, 0);
  /* increment duty-cycle or reset if reached TOP value */
  if( compareValue == TIMER_TopGet(TIMER0))
    TIMER_CompareBufSet(TIMER0, 0, 0);
  else
    TIMER_CompareBufSet(TIMER0, 0, ++compareValue);
}
/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();
    
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);
   
  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
  
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  /* Route CC0 to location 3 (PD1) and enable pin */  
  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3); 
  
  /* Set Top Value */
  TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ);
  
  /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
  TIMER_CompareBufSet(TIMER0, 0, 0);

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
  
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /* Configure timer */
  TIMER_Init(TIMER0, &timerInit);
   
  while(1)
  {
    /* Go to EM1 */
    EMU_EnterEM1();
  }
  
}

