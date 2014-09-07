/*****************************************************************************
 * @file main_timer_up_count.c
 * @brief TIMER Up Count Demo Application
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
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"

#if defined ( STK3700 )
  #define LED_PORT gpioPortE
  #define LED_PIN  2
#elif defined ( STK3300 )
  #define LED_PORT gpioPortD
  #define LED_PIN  7
#elif defined ( STKG8XX )
  #define LED_PORT gpioPortC
  #define LED_PIN  0
#else
  #error "undefined KIT"
#endif
/* 13671 Hz -> 14Mhz (clock frequency) / 1024 (prescaler)
  Setting TOP to 27342 results in an overflow each 2 seconds */
#define TOP 27342
/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
  
  /* Toggle LED ON/OFF */
  GPIO_PinOutToggle(LED_PORT, LED_PIN);
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
  
  /* Configure pin as push pull output for LED/pin drive */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPullDrive, 0);
  
  /* Select TIMER0 parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true, 
    .debugRun   = true, 
    .prescale   = timerPrescale1024, 
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
  
  /* Set TIMER Top value */
  TIMER_TopSet(TIMER0, TOP);
  
  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);
  
  while(1)
  {
    /* Go to EM1 */
    EMU_EnterEM1();
  } 
 
}

