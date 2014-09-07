/**************************************************************************//**
 * @file
 * @brief Exercise: ADC single conversion using emlib
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
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "rtcdrv.h"

/**************************************************************************//**
 * @brief  ADC Configuration
 *****************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Fill in your code here */
  (void)init;
  (void)singleInit;

}


/**************************************************************************//**
 * @brief  Main function
 * The ADC is set up in single conversion mode and samples VDD/3 10 times each
 * second, it sleeps in em2 between samples. The supply voltage (VDD) is then
 * calculated.
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  SegmentLCD_Init(false);

  uint32_t sample;
  double   voltage;

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  ADCConfig();

  /* Stay in this loop forever at end of program */
  while (1)
  {
    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);

    /* Calculate supply voltage */
    voltage = (sample * 1.25 * 3) / 4096;

    /* Write to LCD */
    char buffer[10];
    snprintf(buffer, 8, "%1.2f", voltage);
    SegmentLCD_Write(buffer);

    /* wait 100ms in EM2 before next conversion */
    RTCDRV_Trigger(100, NULL);
    EMU_EnterEM2(true);
  }
}



