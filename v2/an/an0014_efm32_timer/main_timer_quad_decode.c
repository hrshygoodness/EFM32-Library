/*****************************************************************************
 * @file main_timer_quad_decode.c
 * @brief TIMER Quadrature Decoder Demo Application
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

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* clear flag for TIMER0 CC0 interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_CC0);
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
  
  /* Enable clock for PRS module */
  CMU_ClockEnable(cmuClock_PRS, true);
  
  /* Set led drive pins (PC0 - PC3) as outputs */
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);
  
  /* Configure PB9 as an input for PB0 button with filter enable (out = 1)*/
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 1); 
  
  /* Configure PB10 as an input for PB1 button with filter enable (out = 1)*/
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 1); 
  
  /* Enable interrupts on Pins PB9 and PB10 */
  GPIO_IntConfig(gpioPortB, 9, false, false, false);
  GPIO_IntConfig(gpioPortB, 10, false, false, false);
  
  /* Enable PRS sense on GPIO and disable interrupt sense */
  GPIO_InputSenseSet(GPIO_INSENSE_PRS, _GPIO_INSENSE_RESETVALUE);
  
  /* Select GPIO as source and GPIOPIN9 as signal for PRS channel 0 */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN9, prsEdgeOff);
  
  /* Select GPIO as source and GPIOPIN10 as signal for PRS channel 1 */
  PRS_SourceSignalSet(1, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN10, prsEdgeOff);
  
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCapture, /* This is only to have interrupts on capture */
    .filter     = false,
    .prsInput   = true,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);
  
  /* Change PRS channel to 1 */
  timerCCInit.prsSel = timerPRSSELCh1;
 
  /* Configure CC channel 1 */
  TIMER_InitCC(TIMER0, 1, &timerCCInit);
  
  /* Select TIMER0 parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeQDec,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);
  
  /* Reset counter */ 
  TIMER0->CNT=0;
  
  /* ------------------------------------------------------- */
  /* THIS PART IS FOR INTERRUPT DRIVEN LED DISPLAY */
  /* ------------------------------------------------------- */

  /* Enable CC0 interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_CC0);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /* Create and initialize variables for led movement */
  uint32_t timerPrevCount = 0, timerCurrentCount;
  int8_t pin=0;
  
  while(1)
  {    
    /* Get the current counter value */
    timerCurrentCount = TIMER_CounterGet(TIMER0);
    
    /* Each time there is a capture it is verified if the counter value is 
       bigger or smaller than the previous count value to determine the direction */
    if(timerPrevCount < timerCurrentCount || ((timerPrevCount == 0xFFFF) && (timerCurrentCount == 0)))
    { 
      GPIO_PortOutClear(gpioPortC, 0xF);
      GPIO_PinOutSet(gpioPortC, pin);
      pin++;
      if(pin>3) 
        pin=0; 
    }else if(timerPrevCount > timerCurrentCount || ((timerPrevCount == 0) && (timerCurrentCount == 0xFFFF)))
    {
      GPIO_PortOutClear(gpioPortC, 0xF);
      GPIO_PinOutSet(gpioPortC, pin);
      pin--;
      if(pin<0) 
        pin=3;
    }
    /* Reading the current counter value to compare with during the next cycle */
    timerPrevCount = timerCurrentCount;
    /* Go to EM1 for energy saving */
    EMU_EnterEM1();
  }
  
}

