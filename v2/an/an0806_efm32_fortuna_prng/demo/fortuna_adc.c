/**************************************************************************//**
 * @file fortuna_adc.c
 * @brief Interface to ADC in order to collect entropy from ADC.
 * @author Silicon Labs
 * @version 1.03
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

#include <stdint.h>
#include <stdbool.h>

/* EMLIB headers */
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"

/* Fortuna headers */
#include "fortuna_adc.h"

/* Global variables for ADC */
static volatile bool  adcFinished;


/**************************************************************************//**
 * @brief   ADC interrupt Handler. Clears interrupt and then indicates data is
 *          ready by setting adcFinished to true.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Acknowledge interrupt. */
  ADC_IntClear(ADC0, ADC_IF_SINGLE);

  /*Indicate data is ready. */
  adcFinished = true;
}


/**************************************************************************//**
 * @brief   Configure ADC for sampling the temperature sensor.
 *****************************************************************************/
void AdcRandomInitialize(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Enable ADC clock. */
  CMU_ClockEnable(cmuClock_ADC0, true);

  init.timebase = ADC_TimebaseCalc(0);

  /* Set ADC clock to 4 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(4000000, 0);

  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V. */
  singleInit.reference = adcRef1V25;
  singleInit.input     = adcSingleInpTemp;
  singleInit.resolution = adcRes12Bit;
  singleInit.acqTime = adcAcqTime1;
  ADC_InitSingle(ADC0, &singleInit);

  /* Generate interrupt when data conversion is completed. */
  ADC_IntClear(ADC0, ADC_IFC_SINGLE);
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
}


/**************************************************************************//**
 * @brief Get a sample from the ADC.
 *****************************************************************************/
uint32_t AdcSampleGet(void)
{
  uint32_t  adcData;          /* Sample from ADC. */
  
  /* Clear flag which indicates whether the ADC irq handler has been called
     and ADC data is ready. */
  adcFinished = false;
  
  /* Trigger ADC conversion*/
  ADC_Start(ADC0, adcStartSingle);
  
  /* Stay in EM1 Energy mode until ADC finishes conversion. */
  while (!adcFinished)
  {
    /* Only ADC interrupt wakes-up EFM32 */
    EMU_EnterEM1();
  }
  
  /* ADC has finished conversion if it reached here. */
  adcData = ADC_DataSingleGet(ADC0);

  return adcData;
}
