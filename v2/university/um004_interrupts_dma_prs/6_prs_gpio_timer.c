/**************************************************************************//**
 * @file
 * @brief PRS Example
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include "efm32.h"
#include "em_acmp.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "em_prs.h"
#include "em_timer.h"
#include "segmentlcd.h"

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 */
#define PB0_PORT                    gpioPortB
#define PB0_PIN                     9

#else

/* Defines for Push Button 0 */
#define PB0_PORT                    gpioPortD
#define PB0_PIN                     8

#endif

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
 * Configures the GPIO
 *****************************************************************************/
void GPIO_setup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);

  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, false);

  GPIO_InputSenseSet(GPIO_INSENSE_PRS, _GPIO_INSENSE_RESETVALUE);
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
    .eventCtrl = timerEventRising,        /* Input capture event control */
    .edge      = timerEdgeRising,         /* Input capture on rising edge */
    .prsSel    = timerPRSSELCh5,          /* Prs channel select channel 5*/
    .cufoa     = timerOutputActionNone,   /* No action on counter underflow */
    .cofoa     = timerOutputActionNone,   /* No action on counter overflow */
    .cmoa      = timerOutputActionNone,   /* No action on counter match */
    .mode      = timerCCModeCapture,      /* CC channel mode capture */
    .filter    = false,                   /* No filter */
    .prsInput  = true,                    /* CC channel PRS input */
    .coist     = false,                   /* Comparator output initial state */
    .outInvert = false,                   /* No output invert */
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
    .fallAction = timerInputActionReloadStart,  /* Reload and start on falling edge */
    .riseAction = timerInputActionStop,         /* Stop counter on rising edge */
    .mode       = timerModeUp,                  /* Counting up */
    .dmaClrAct  = false,                        /* No DMA */
    .quadModeX4 = false,                        /* No quad decoding */
    .oneShot    = false,                        /* Counting up constinuously */
    .sync       = false,                        /* No start/stop/reload by other timers */
  };

  /* Initialize TIMER0 */
  TIMER_Init(TIMER0, &timerInit);


  
 #if defined(_EFM32_GIANT_FAMILY)

  /* PRS setup */
  PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN9, prsEdgeOff);

#else

  /* PRS setup */
  PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_GPIOH, PRS_CH_CTRL_SIGSEL_GPIOPIN8, prsEdgeOff);
  
#endif
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Align different chip revisions*/
  CHIP_Init();

  /* Initialize the LCD */
  SegmentLCD_Init(false);

  /* Initialise the ACMP */
  GPIO_setup();

  /* Initialise the TIMER */
  TIMER_setup();

  /* Enable timer interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IEN_CC0);

  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);


  while (1)
  {
    /* Enter EM1 while waiting for capture. */
    EMU_EnterEM1();
  }
}
