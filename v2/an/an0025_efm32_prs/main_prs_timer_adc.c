/*****************************************************************************
 * @file main_prs_timer_adc.c
 * @brief PRS demo application, TIMER triggering ADC conversion
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
#include "em_adc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_lcd.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_chip.h"
#include "segmentlcd.h"


/**************************************************************************//**
 * @brief ADC0_IRQHandler
 * Interrupt Service Routine for ADC
 *****************************************************************************/
void ADC0_IRQHandler(void)
{ 
  /* Clear ADC0 single conversion complete interrupt flag */
  ADC_IntClear(ADC0, ADC_IEN_SINGLE);
  
  /* Read conversion result and show it on the LCD 
     Since Vdd was selected for both reference and 
     ADC input the result will be 255 which is the 
     maximum with 8 bit resolution */
  SegmentLCD_Number(ADC_DataSingleGet(ADC0));
}

/**************************************************************************//**
 * @brief  TIMER0_setup
 * Configures and starts the TIMER
 *****************************************************************************/
void TIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  
  /* Select timer parameters */  
  const TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,                         /* Start counting when init complete */         
    .debugRun   = false,                        /* Counter not running on debug halt */  
    .prescale   = timerPrescale1024,            /* Prescaler of 1024 -> overflow after aprox 4.70 seconds */
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
}

/**************************************************************************//**
 * @brief  ADC_setup
 * configures the ADC for single conversion
 *****************************************************************************/
void ADC_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  
  /* Default common configuration for ADC */
  const ADC_Init_TypeDef ADCInit = ADC_INIT_DEFAULT;
  
  /* Initialize ADC common parts for both single conversion and scan sequence */
  ADC_Init(ADC0, &ADCInit);
  
  /* Configuration for single sample conversion */
  ADC_InitSingle_TypeDef ADCInitSingle = 
  {
    .prsSel     = adcPRSSELCh5,     /* ACD triggered by PRS channel 0 */
    .acqTime    = adcAcqTime8,      /* aquisition time of 8 ADC clock cycles */
    .reference  = adcRefVDD,        /* Vdd as ADC reference */
    .resolution = adcRes8Bit,       /* 8 bit resolution */
    .input      = adcSingleInpVDD,  /* Vdd as ADC input, result will always be maximum */
    .diff       = false,            /* no differential input */
    .prsEnable  = true,             /* PRS enable */
    .leftAdjust = false,            /* right adjust result */
    .rep        = false             /* no repetition */
  };

  /* Initialize ADC single sample conversion */
  ADC_InitSingle(ADC0, &ADCInitSingle);
  
  /* Select TIMER0 as source and timer overflow as signal */
  PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Align different chip revisions */
  CHIP_Init();

  /* Initialize LCD */
  SegmentLCD_Init(false);  
  
  /* Initialize ADC */
  ADC_setup();
  
  /* Initialize TIMER */
  TIMER_setup();
  
  //------------------------------------------------------------------------
  // THE INTERRUPT IS SIMPLY TO PRESENT THE ADC CONVERSION RESULT ON THE LCD
  //------------------------------------------------------------------------
  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  
  /* Enable ADC interrupt vector in NVIC*/
  NVIC_EnableIRQ(ADC0_IRQn);
  //------------------------------------------------------------
  //------------------------------------------------------------
  
  while(1)
  {
    /* Enter EM1 */ 
    EMU_EnterEM1();
  }
}
