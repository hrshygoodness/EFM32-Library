/*****************************************************************************
 * @file main_dac_sine_generator.c
 * @brief Digital to Analog converter, sine generator example
 * @author Silicon Labs
 * @version 1.11
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
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dac.h"
#include "em_prs.h"
#include "em_timer.h"

/* Function prototypes */
void DAC_setup(void);
void TIMER_setup(void);
void SineFrequencySet(uint32_t freq);

/**************************************************************************//**
 * @brief  Main function
 * Setup of dac, prs and timer and set frequency, enter em1 while dac is working
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();


  /* Initialise the TIMER */
  TIMER_setup();

  /* Initialise the DAC */
  DAC_setup();

  /* Enable DAC channel 0, located on pin PB11 */
  DAC_Enable(DAC0, 0, true);

  /* Set the desired frequency in Hz, preferably 20 Hz - 15 kHz */
  SineFrequencySet(15000);

  /* Enter EM1 while the DAC, Timer and PRS are generating the sinewave */
  EMU_EnterEM1();

  /* Should never get here */
  return 0;
}


/**************************************************************************//**
 * @brief  Setup DAC
 * Configures and starts the DAC
 *****************************************************************************/
void DAC_setup(void)
{
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Enable the DAC clock */
  CMU_ClockEnable(cmuClock_DAC0, true);

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 1 MHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(1000000, 0);

  /* Disable PRS control of output driver. PRS is instead used to trigger new samples */
  init.outEnablePRS = false;

  /* Enable Sine Generator Mode */
  init.sineEnable = true;

  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);

  initChannel.prsEnable = true;
  initChannel.prsSel    = dacPRSSELCh0;

  DAC_InitChannel(DAC0, &initChannel, 0);
}

/**************************************************************************//**
 * @brief  Setup TIMER for sine output
 * Configures and starts the TIMER for sine generation triggering
 *****************************************************************************/
void TIMER_setup(void)
{
  /* Use default timer */
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  TIMER_Init(TIMER0, &timerInit);

  /* PRS setup */
  /* Select TIMER0 as source and TIMER0OF (Timer0 overflow) as signal */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);
}

/**************************************************************************//**
 * @brief  Calculate and set new timer top value
 * Calculates and sets new timer top value based on current hfperclk
 *****************************************************************************/
void SineFrequencySet(uint32_t freq)
{
  uint32_t TimerTopValue;
  /* Get peripheral clock frequency */
  uint32_t hfperFreq = CMU_ClockFreqGet(cmuClock_HFPER);

  /* Calculate new timer top value */
  TimerTopValue = (hfperFreq >> 4) / freq;

  if (TimerTopValue > 0xffff)
  {
    TimerTopValue = 0xffff;
  }

  TIMER_TopBufSet(TIMER0, TimerTopValue);
}
