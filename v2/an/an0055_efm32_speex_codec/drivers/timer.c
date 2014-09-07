/**************************************************************************//**
 * @file timer.c
 * @brief Setup Timer for this application note
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

#include "em_device.h"
#include "em_prs.h"
#include "config.h"

/***************************************************************************//**
 * @brief
 *   Configure Timer for this application.
 *******************************************************************************/
void initTimer(void)
{
  /* Use default setting of CTRL register, configure PRESC with different */
  /* BASE_FREQ and system core clock frequency, the BASE_FREQ depends on */
  /* maximum sampling frequency of ADC and DAC */
  TIMER_USED->CTRL |= ((31 - __CLZ(SystemCoreClock/BASE_FREQ)) << _TIMER_CTRL_PRESC_SHIFT);
#if (SPEEX_BAND < WIDEBAND || SPEEX_BAND == NB8KCODEC)
  TIMER_USED->TOP = SAMPLE_8K;
#endif
}

/***************************************************************************//**
 * @brief
 *   Enable PRS and configure TIMER TOP value for different sampling frequency.
 * @param bufferSize - Buffer size to setup sampling frequency  
 *******************************************************************************/
void setTopValue(uint16_t bufferSize)
{
  /* Select TIMER as source and Timer overflow as signal */
  PRS_SourceSignalSet(DAC_PRS_CH, TIMER_PRS, TIMER_SRC, prsEdgeOff);

#if SPEEX_BAND == WIDEBAND  
  /* Setup sampling frequency for WB to play NB or WB */
  if (bufferSize < SAMPLE_SIZE)
  {
    TIMER_USED->TOP = SAMPLE_8K;
  }
  else
  {
    TIMER_USED->TOP = SAMPLE_16K;
  }
#elif SPEEX_BAND == ULTRAWIDEBAND
  /* Setup sampling frequency for UWB to play NB or WB or UWB */
  if (bufferSize < SAMPLE_SIZE/2)
  {
    TIMER_USED->TOP = SAMPLE_8K;
  }
  else if (bufferSize < SAMPLE_SIZE)
  {
    TIMER_USED->TOP = SAMPLE_16K;
  }
  else
  {
    TIMER_USED->TOP = SAMPLE_32K;
  }    
#endif
  /* Sampling frequency is fixed on codec, NB8K and NB */  
  TIMER_USED->CMD = TIMER_CMD_START;
}
