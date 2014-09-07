/***************************************************************************//**
 * @file adc.c
 * @brief Setup ADC for audio input
 * @author Silicon Labs
 * @version 1.06
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_adc.h"
#include "config.h"

/***************************************************************************//**
 * @brief
 *   Configure ADC usage for this application.
 *******************************************************************************/
void initAdc(void)
{
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
  
  /* Init common issues for both single and scan mode */
  /* TIMERBASE should be set >= 1us for ADC warmup */
  /* When HFPERCLK is running at 48MHz, the maximum TIMERBASE */
  /* is (0x1f + 1) x 1/48M = 6.67us, less (1 - 0.67) = 0.33us if VDD as ref */
  /* If using internal ref, 6 x 0.67us = 4us, less (6 - 4) = 2us */
  /* Use acquisition time in channel configuration to compensate TIMEBASE */ 
  init.ovsRateSel = adcOvsRateSel16; 
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  ADC_Init(ADC0, &init);

  /* Init for single conversion use, enable PRS channel for ADC */
  /* Default reference voltage is 1.25V */
  /* Internal ref, compensate 2us = 24 ADC clock + 1 = 25 minimum */
  singleInit.acqTime = adcAcqTime32;
#if ADC_REF_SELECT == 1
  /* Set reference voltage to 2.5V */
  singleInit.reference = adcRef2V5;
#elif ADC_REF_SELECT == 2
  /* Set reference voltage to VDD */
  singleInit.reference = adcRefVDD;
  /* VDD ref, compensate 0.33us = 4 ADC clock + 1 = 5 minimum */
  singleInit.acqTime = adcAcqTime8;
#endif
  singleInit.prsSel = ADC_PRS_SEL;
  singleInit.leftAdjust = true;
  singleInit.input = ADC_CHANNEL;
#if ADC_OVS_16 == 1
  /* Use oversampling to 16 bit resolution */
  singleInit.resolution = adcResOVS;
#endif
  
  ADC_InitSingle(ADC0, &singleInit);
}
