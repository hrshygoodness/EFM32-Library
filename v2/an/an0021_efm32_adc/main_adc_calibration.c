/******************************************************************************
 * @file main_adc_calibration.c
 * @brief ADC calibration example
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
#include "em_lcd.h"
#include "segmentlcd.h"
#include "rtcdrv.h"

/* Use a siprintf which don't support floating point formatting for GCC */
#if defined(__ICCARM__) || defined (__CC_ARM) || defined (__CROSSWORKS_ARM)
  #define SPRINTF                  sprintf
#else
  #define SPRINTF                  siprintf
#endif



/***************************************************************************//**
 * @brief
 *   Calibrate offset and gain for the specified reference.
 *   Supports currently only single ended gain calibration.
 *   Could easily be expanded to support differential gain calibration.
 *
 * @details
 *   The offset calibration routine measures 0 V with the ADC, and adjust
 *   the calibration register until the converted value equals 0.
 *   The gain calibration routine needs an external reference voltage equal
 *   to the top value for the selected reference. For example if the 2.5 V
 *   reference is to be calibrated, the external supply must also equal 2.5V.
 *
 * @param[in] adc
 *   Pointer to ADC peripheral register block.
 *
 * @param[in] ref
 *   Reference used during calibration. Can be both external and internal
 *   references.
 *
 * @return
 *   The final value of the calibration register, note that the calibration
 *   register gets updated with this value during the calibration.
 *   No need to load the calibration values after the function returns.
 ******************************************************************************/
uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref)
{
  int32_t  sample;
  uint32_t cal;

  /* Binary search variables */
  uint8_t high;
  uint8_t mid;
  uint8_t low;

  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(adc);

  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(7000000, 0);

  /* Set an oversampling rate for more accuracy */
  init.ovsRateSel = adcOvsRateSel4096;
  /* Leave other settings at default values */
  ADC_Init(adc, &init);

  /* Init for single conversion use, measure diff 0 with selected reference. */
  singleInit.reference = ref;
  singleInit.input     = adcSingleInpDiff0;
  singleInit.acqTime   = adcAcqTime16;
  singleInit.diff      = true;
  /* Enable oversampling rate */
  singleInit.resolution = adcResOVS;

  ADC_InitSingle(adc, &singleInit);

  /* ADC is now set up for offset calibration */
  /* Offset calibration register is a 7 bit signed 2's complement value. */
  /* Use unsigned indexes for binary search, and convert when calibration */
  /* register is written to. */
  high = 128;
  low  = 0;

  /* Do binary search for offset calibration*/
  while (low < high)
  {
    /* Calculate midpoint */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement and written to both scan and */
    /* single calibration registers */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
    cal     |= (mid - 63) << _ADC_CAL_SINGLEOFFSET_SHIFT;
    cal     |= (mid - 63) << _ADC_CAL_SCANOFFSET_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);

    /* Wait while conversion is active */
    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part of to repeat search */
    /* Calibration register has negative effect on result */
    if (sample < 0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample > 0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  /* Now do gain calibration, only input and diff settings needs to be changed */
  adc->SINGLECTRL &= ~(_ADC_SINGLECTRL_INPUTSEL_MASK | _ADC_SINGLECTRL_DIFF_MASK);
  adc->SINGLECTRL |= (adcSingleInpCh4 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (false << _ADC_SINGLECTRL_DIFF_SHIFT);

  /* ADC is now set up for gain calibration */
  /* Gain calibration register is a 7 bit unsigned value. */

  high = 128;
  low  = 0;

  /* Do binary search for gain calibration */
  while (low < high)
  {
    /* Calculate midpoint and write to calibration register */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK);
    cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
    cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);

    /* Wait while conversion is active */
    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part to repeat search */
    /* Compare with a value atleast one LSB's less than top to avoid overshooting */
    /* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
    /* applies to the 12 bit result value, if 0xffe is the top value in 12 bit, this */
    /* is in turn 0xffe0 in the 16 bit result. */
    /* Calibration register has positive effect on result */
    if (sample > 0xffd0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample < 0xffd0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  return adc->CAL;
}


/******************************************************************************
 * @brief  Main function
 * The two calibration routines are called in the right order, the offset needs
 * to be calibrated first. The new and the old calibrated value for the 1.25 V
 * reference is printed on the segment LCD for comparison. They should be
 * almost equal at room temperature.
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  uint8_t  offset_calibration_value;
  uint8_t  gain_calibration_value;
  uint32_t calibration_value;

  uint32_t old_gain_calibration_value =
    (DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_GAIN_MASK)
    >> _DEVINFO_ADC0CAL0_1V25_GAIN_SHIFT;

  uint32_t old_offset_calibration_value =
    (DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_OFFSET_MASK)
    >> _DEVINFO_ADC0CAL0_1V25_OFFSET_SHIFT;

  RTCDRV_Trigger(100, NULL);

  calibration_value = ADC_Calibration(ADC0, adcRef1V25);

  offset_calibration_value = (calibration_value & _ADC_CAL_SINGLEOFFSET_MASK) >> _ADC_CAL_SINGLEOFFSET_SHIFT;
  gain_calibration_value   = (calibration_value & _ADC_CAL_SINGLEGAIN_MASK) >> _ADC_CAL_SINGLEGAIN_SHIFT;

  /* Write new and old offset calibration value for 1.25 V reference to lcd. */
  /* Stay in this loop forever at end of program */

  char buf[10];
  while (1)
  {
    SPRINTF(buf, "O %X %X", (unsigned int) offset_calibration_value, (unsigned int) old_offset_calibration_value);
    SegmentLCD_Write(buf);

    /* Wait some seconds */
    RTCDRV_Trigger(2000, NULL);
    EMU_EnterEM2(true);

    /* Write new and old gain calibration value for 1.25 V reference to lcd. */
    SPRINTF(buf, "G %X %X", (unsigned int) gain_calibration_value, (unsigned int) old_gain_calibration_value);
    SegmentLCD_Write(buf);

    /* Wait some seconds */
    RTCDRV_Trigger(2000, NULL);
    EMU_EnterEM2(true);
  }
}



