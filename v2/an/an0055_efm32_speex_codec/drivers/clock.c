/**************************************************************************//**
 * @file clock.c
 * @brief Setup core and peripheral clocks for Speex Codec
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_cmu.h"
#include "clock.h"
#include "config.h"

/**************************************************************************//**
 * @brief Clock setup
 * Setup low/high frequency core clock and peripheral clocks 
 *****************************************************************************/
void initClock(void)
{
  /* Setup crystal frequency */
  SystemHFXOClockSet(SPEEX_HFXO_FREQ);   
  /* Use HFXO as core clock frequency */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  /* Disable HFRCO */
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

#if SPEEX_HFXO_FREQ == FREQ8M
#if SPEEX_BAND > NARROWBAND
#error CODEC or WIDEBAND or ULTRAWIDEBAND cannot run on 8MHz crystal 
#endif
  
#elif SPEEX_HFXO_FREQ == FREQ16M
#if SPEEX_BAND > WIDEBAND
#error CODEC or ULTRAWIDEBAND cannot run on 16MHz crystal 
#endif
  
#elif SPEEX_HFXO_FREQ == FREQ48M
  /* Scale the HFCLK to lower frequency, reset LE to DIV2 and HFLE to 0 */
#if SPEEX_BAND < WIDEBAND
  CMU_ClockDivSet(cmuClock_HF, 6);
  CMU->CTRL &= ~_CMU_CTRL_HFLE_MASK;
  CMU->HFCORECLKDIV = _CMU_HFCORECLKDIV_RESETVALUE;
#elif SPEEX_BAND < ULTRAWIDEBAND
  CMU_ClockDivSet(cmuClock_HF, 3);
  CMU->CTRL &= ~_CMU_CTRL_HFLE_MASK;
  CMU->HFCORECLKDIV = _CMU_HFCORECLKDIV_RESETVALUE;
#endif    
  
#elif SPEEX_HFXO_FREQ == FREQ32M
  /* Scale the HFCLK to lower frequency */
#if SPEEX_BAND < WIDEBAND
  CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_4);
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_4);
#elif SPEEX_BAND < ULTRAWIDEBAND
  CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_2);
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_2);
#else
#error CODEC or ULTRAWIDEBAND cannot run on 32MHz crystal 
#endif
#endif
  
  /* Enable clock for DMA */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_DMA;
  
  /* Enable clock for Peripherals */
#if SPEEX_BAND < NB8KCODEC
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_DAC0 + CMU_HFPERCLKEN0_GPIO + CMU_HFPERCLKEN0_PRS + TIMER_CLK;
#else
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC0 + CMU_HFPERCLKEN0_DAC0 + CMU_HFPERCLKEN0_GPIO + CMU_HFPERCLKEN0_PRS + TIMER_CLK;
#endif
}
