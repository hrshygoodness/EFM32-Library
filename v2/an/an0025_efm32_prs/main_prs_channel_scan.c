/*****************************************************************************
 * @file main_prs_channel_scan.c
 * @brief PRS demo application, scan a PRS channel using a TIMER
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
    .fullBias                 = false,                  /* no full bias current*/
    .halfBias                 = false,                  /* no half bias current */
    .biasProg                 = 7,                      /* Biasprog current 1.4 uA */
    .interruptOnFallingEdge   = false,                  /* disable interrupt for falling edge */
    .interruptOnRisingEdge    = false,                  /* disable interrupt for rising edge */
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
 * @brief  PRS_ScanChannel
 * Waits for activity on a selected PRS channel and writes on the LCD 
   when activity occurs
 *  
 * @param[in] timer  
 *   Pointer to TIMER peripheral register block.
 *
 * @param[in] prsCh
 * 	 PRS Channel to be monitored
 *
 * @param[in] edgeType
 *   Signal edge to be monitored/captured 
 *****************************************************************************/
void PRS_ScanChannel(TIMER_TypeDef *timer, TIMER_PRSSEL_TypeDef prsCh, TIMER_Edge_TypeDef edgeType)
{
  
/* enable the clock for the correct timer */  
#define TIMER_Clock(T) (T==TIMER0 ? cmuClock_TIMER0 : \
                          T==TIMER1 ? cmuClock_TIMER1 : 0)
  
  /* Enable necessary clocks */
  CMU_ClockEnable((CMU_Clock_TypeDef)TIMER_Clock(timer), true);
    
  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Select CC channel parameters */
  const TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventFalling,      /* input capture event control */
    .edge       = edgeType,               /* input capture on falling edge */
    .prsSel     = prsCh,                  /* prs channel select channel 5*/
    .cufoa      = timerOutputActionNone,  /* no action on counter underflow */
    .cofoa      = timerOutputActionNone,  /* no action on counter overflow */
    .cmoa       = timerOutputActionNone,  /* no action on counter match */
    .mode       = timerCCModeCapture,     /* CC channel mode capture */
    .filter     = false,                  /* no filter */
    .prsInput   = true,                   /* CC channel PRS input */
    .coist      = false,                  /* comparator output initial state */
    .outInvert  = false,                  /* no output invert */
  };
  
  /* Initialize TIMER0 CC0 */
  TIMER_InitCC(timer, 0, &timerCCInit);
  
  /* Select timer parameters */  
  const TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,                         /* start counting when init complete */         
    .debugRun   = false,                        /* counter not running on debug halt */  
    .prescale   = timerPrescale1024,            /* prescaler of 64 */
    .clkSel     = timerClkSelHFPerClk,          /* TIMER0 clocked by the HFPERCLK */
    .fallAction = timerInputActionNone,         /* stop counter on falling edge */
    .riseAction = timerInputActionNone,         /* reload and start on rising edge */
    .mode       = timerModeUp,                  /* counting up */
    .dmaClrAct  = false,                        /* no DMA */
    .quadModeX4 = false,                        /* no quad decoding */
    .oneShot    = false,                        /* counting up constinuously */
    .sync       = false,                        /* no start/stop/reload by other timers */
  };
  
  /* Initialize TIMER0 */
  TIMER_Init(timer, &timerInit);  
  
  /* Poll the Input Capture Valid flag in the Status register
      The program will hang at this point waiting for activity on this
	  channel */
  while(!((TIMER0->STATUS & _TIMER_STATUS_ICV0_MASK )>>16));  
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Align different chip revisions*/
  CHIP_Init();
  
  /* Initialise the ACMP */
  ACMP_setup();

  /* PRS setup */
  /* Enable clock for PRS and select ACMP as source and ACMP0OUT (ACMP0 OUTPUT) as signal 
     for channel 5 */
  CMU_ClockEnable(cmuClock_PRS, true);
  PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_ACMP0, PRS_CH_CTRL_SIGSEL_ACMP0OUT, prsEdgeOff);
  
  /* Start PRS scan 
     This function will halt the program while there is no PRS activity
     This function assumes that the PRS has been setup previously */
  PRS_ScanChannel(TIMER0, timerPRSSELCh5, timerEdgeFalling);
  
  /* Write PRS and channel number on the LCD to acknowledge PRS activity */  
  SegmentLCD_Write("PRS");
  SegmentLCD_Number((int)timerPRSSELCh5);
  
  while(1)
  {
    /* Enter EM1 while waiting for capture. */
    EMU_EnterEM1();
  }
}
