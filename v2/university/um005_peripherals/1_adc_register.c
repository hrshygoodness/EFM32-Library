/**************************************************************************//**
 * @file
 * @brief ADC single conversion using registers
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

/******************************************************************************
 * @brief  Main function
 * The ADC is set up in single conversion mode and samples VDD/3 10 times each
 * second, it sleeps in em2 between samples. The supply voltage (VDD) is then
 * calculated.
 *
 * The register operations for the ADC are set directly in the Main function.
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

  /**** Configure ADC usage for this application. ****/

  /* Sets the ADCn_CTRL - Control Register. Look at register map for the ADC
   * in the reference manual. Only the bitfields that should have another value
   * than the default (reset value) are changed
   *
   * The high frequency peripheral clock used by the ADC is running at 14 MHz,
   * we therefore need 14 cycles to achieve the needed 1 us warmup time. The
   * warmup is set to TIMEBASE + 1 cycles, so we set TIMEBASE to be 13. We want
   * to run the ADC at 7 MHz, that means a prescale factor of 2, and that PRESC
   * should be 1. */
  ADC0->CTRL =
    (13 << _ADC_CTRL_TIMEBASE_SHIFT) |
    (1 << _ADC_CTRL_PRESC_SHIFT);

  /* Sets the ADCn_SINGLECTRL - Single Sample Control Register.
   * Defines that are already shifted are used here, so we only need combine
   * them with bitwise OR. Notice how these do not begin with _ like the
   * defines above. */
  ADC0->SINGLECTRL =
    ADC_SINGLECTRL_AT_32CYCLES |
    ADC_SINGLECTRL_INPUTSEL_VDDDIV3;

  /* Stay in this loop forever at end of program */
  while (1)
  {
    /* Start the sampling */
    ADC0->CMD = adcStartSingle;

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC0->SINGLEDATA;

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



