/*****************************************************************************
 * @file main_prs_pulse_width.c
 * @brief PRS demo application, pulse width capture width ACMP and TIMER
 * @author Silicon Labs
 * @version 1.07
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
#include "em_acmp.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "em_prs.h"
#include "em_timer.h"
#include "segmentlcd.h"


/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Clear flag for TIMER0 CC0 interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_CC0);
  
  /* Write capture value on LCD */
  SegmentLCD_Number(TIMER_CaptureGet(TIMER0, 0));
}

/**************************************************************************//**
 * @brief  ACMP_setup
 * Configures and starts the ACMP
 *****************************************************************************/
void ACMP_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PC4 as input with pull down for ACMP channel 4 input */
  GPIO_PinModeSet(gpioPortC, 4, gpioModeInputPullFilter, 0);

  /* Analog comparator parameters */
  const ACMP_Init_TypeDef acmpInit =
  {
    .fullBias                 = false,                  /* No full bias current*/
    .halfBias                 = false,                  /* No half bias current */
    .biasProg                 = 7,                      /* Biasprog current 1.4 uA */
    .interruptOnFallingEdge   = false,                  /* Disable interrupt for falling edge */
    .interruptOnRisingEdge    = false,                  /* Disable interrupt for rising edge */
    .warmTime                 = acmpWarmTime256,        /* Warm-up time in clock cycles, should be >140 cycles for >10us warm-up @ 14MHz */
    .hysteresisLevel          = acmpHysteresisLevel0,   /* Hysteresis level 0  - no hysteresis */
    .inactiveValue            = 0,                      /* Inactive comparator output value */
    .lowPowerReferenceEnabled = false,                  /* Low power reference mode disabled */
    .vddLevel                 = 32,                     /* Vdd reference scaling of 32 */
  };

  /* Init ACMP and set ACMP channel 4 as positive input
     and scaled Vdd as negative input */
  ACMP_Init(ACMP0, &acmpInit);
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel4);
  ACMP_Enable(ACMP0);
}

/**************************************************************************//**
 * @brief  TIMER0_setup
 * Configures the TIMER
 *****************************************************************************/
void TIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Select CC channel parameters */
  const TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventFalling,      /* Input capture event control */
    .edge       = timerEdgeFalling,       /* Input capture on falling edge */
    .prsSel     = timerPRSSELCh5,         /* Prs channel select channel 5*/
    .cufoa      = timerOutputActionNone,  /* No action on counter underflow */
    .cofoa      = timerOutputActionNone,  /* No action on counter overflow */
    .cmoa       = timerOutputActionNone,  /* No action on counter match */
    .mode       = timerCCModeCapture,     /* CC channel mode capture */
    .filter     = false,                  /* No filter */
    .prsInput   = true,                   /* CC channel PRS input */
    .coist      = false,                  /* Comparator output initial state */
    .outInvert  = false,                  /* No output invert */
  };
  
  /* Initialize TIMER0 CC0 channel */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);
  
  /* Select timer parameters */  
  const TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,                        /* Do not start counting when init complete */         
    .debugRun   = false,                        /* Counter not running on debug halt */  
    .prescale   = timerPrescale1024,            /* Prescaler of 1024 */
    .clkSel     = timerClkSelHFPerClk,          /* TIMER0 clocked by the HFPERCLK */
    .fallAction = timerInputActionStop,         /* Stop counter on falling edge */
    .riseAction = timerInputActionReloadStart,  /* Reload and start on rising edge */
    .mode       = timerModeUp,                  /* Counting up */
    .dmaClrAct  = false,                        /* No DMA */
    .quadModeX4 = false,                        /* No quad decoding */
    .oneShot    = false,                        /* Counting up constinuously */
    .sync       = false,                        /* No start/stop/reload by other timers */
  };
  
  /* Initialize TIMER0 */
  TIMER_Init(TIMER0, &timerInit);  
  
  /* PRS setup */
  /* Select ACMP as source and ACMP0OUT (ACMP0 OUTPUT) as signal */
  PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_ACMP0, PRS_CH_CTRL_SIGSEL_ACMP0OUT, prsEdgeOff);
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Align different chip revisions*/
  CHIP_Init();
  
  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Initialise the ACMP */
  ACMP_setup();

  /* Initialise the TIMER */
  TIMER_setup();  
  
  //------------------------------------------------------------
  // THE INTERRUPT IS SIMPLY TO DISPLAY THE CAPTURE ON THE LCD
  //------------------------------------------------------------
  /* Enable CC0 interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_CC0);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  //------------------------------------------------------------
  //------------------------------------------------------------
  
  while(1)
  {
    /* Enter EM1 while waiting for capture. */
    EMU_EnterEM1();
  }
}
