/******************************************************************************
 * @file main_adc_timer_prs.c
 * @brief ADC timer and PRS example
 * @author Silicon Labs
 * @version 1.10
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


#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_lcd.h"
#include "segmentlcd.h"

uint32_t adcResult;

/**************************************************************************//**
 * @brief ADC0_IRQHandler
 * Interrupt Service Routine for ADC
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Clear ADC0 interrupt flag */
  ADC0->IFC = 1;

  /* Read conversion result to clear Single Data Valid flag */
  adcResult = ADC_DataSingleGet(ADC0);
}

/***************************************************************************//**
* @brief
*   Configure ADC usage for this application.
*******************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(7000000, 0);

  /* WARMUPMODE must be set to Normal according to ref manual before */
  /* entering EM2. In this example, the warmup time is not a big problem */
  /* due to relatively infrequent polling. Leave at default NORMAL, */

  ADC_Init(ADC0, &init);

  /* Init for single conversion use. */
  singleInit.reference  = adcRefVDD;
  singleInit.input      = adcSingleInpCh5;
  singleInit.resolution = adcRes12Bit;
  singleInit.prsEnable  = true; /* Enable PRS for ADC */
  ADC_InitSingle(ADC0, &singleInit);

  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC0->IEN = ADC_IEN_SINGLE;

  /* Enable ADC interrupt vector in NVIC*/
  NVIC_EnableIRQ(ADC0_IRQn);
}

/***************************************************************************//**
* @brief
*   Configure Timer for this application.
*******************************************************************************/
static void TimerConfig(void)
{
  /* Use default timer settings */
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  /* Change prescaler to 64, gives roughly 3 overflows per
   * second at 14MHz with 0xffff as top value */
  timerInit.prescale = timerPrescale64;
  TIMER_Init(TIMER0, &timerInit);
}

/******************************************************************************
 * @brief  Main function
 * The main file starts a timer and uses PRS to trigger an ADC conversion.
 * It waits in EM1 until the ADC conversion is complete, then prints the
 * result on the lcd.
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Select TIMER0 as source and TIMER0OF (Timer0 overflow) as signal (rising edge) */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);

  ADCConfig();
  TimerConfig();

  /* Stay in this loop forever */
  while (1)
  {
    /* Enter EM1 and wait for timer triggered adc conversion */
    EMU_EnterEM1();

    /* Write result to LCD */
    SegmentLCD_Number(adcResult);

    /* Do other stuff */
    int i;
    for (i = 0; i < 10000; i++) ;
  }
}


