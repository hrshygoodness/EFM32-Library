/*****************************************************************************
 * @file main_dac_calibration.c
 * @brief Digital to Analog converter, calibration routine
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
#include "em_dac.h"
#include "em_adc.h"


/* Function prototypes */
uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref);
uint32_t DAC_Calibration(DAC_TypeDef *dac, DAC_Ref_TypeDef ref);

/**************************************************************************//**
 * @brief  Main function
 * To calibrate the ADC properly, a reference voltage, equal to the voltage
 * of the efm32 reference that is calibrated, needs to be applied
 * externally to the ADC channel 4 input (configurable in the adc calibration
 * routine).
 *****************************************************************************/
int main(void)
{
  /* Align different chip revisions */
  CHIP_Init();

  /* Calibrate the DAC with the 1.25V reference */
  DAC_Calibration(DAC0, dacRef1V25);

  while (1) ;
}

/***************************************************************************//**
 * @brief
 *   Calibrate DAC offset and gain for the specified reference.
 *   Supports currently only single ended gain calibration.
 *   Could easily be expanded to support differential gain calibration.
 *
 * @details
 *   The offset calibration routine converts 0 in single-ended mode and measures
 *   it with the ADC. The calibration register is adjusted until the output
 *   equals 0. This is done for both DAC channels.
 *   The gain calibration routine converts the maximum value in single ended mode
 *   and measures the output with the ADC. The calibration register is adjusted
 *   until the output equals the reference voltage minus a few LSB's.
 *
 * @param[in] dac
 *   Pointer to DAC peripheral register block.
 *
 * @param[in] ref
 *   Reference used during calibration. Can be both bandgap and VDD.
 *
 * @return
 *   The final value of the calibration register, note that the calibration
 *   register gets updated with this value during the calibration.
 *   No need to load the calibration values after the function returns.
 ******************************************************************************/
uint32_t DAC_Calibration(DAC_TypeDef *dac, DAC_Ref_TypeDef ref)
{
  int32_t  sample;
  uint32_t cal_reg, ADC_calibration_value;
  int8_t   cal_value, cal_value_signmagnitude;

  /* Enable the DAC and ADC clock */
  CMU_ClockEnable(cmuClock_DAC0, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* First make sure the ADC is calibrated, an external reference voltage */
  /* should be connected to adcSingleInpCh4, PD4 */
  /* For ADC calibration the same reference voltage as the DAC is used, */

  switch (ref)
  {
  case _DAC_CTRL_REFSEL_1V25:
    ADC_calibration_value = ADC_Calibration(ADC0, adcRef1V25);
    break;

  case _DAC_CTRL_REFSEL_2V5:
    ADC_calibration_value = ADC_Calibration(ADC0, adcRef2V5);
    break;

  case _DAC_CTRL_REFSEL_VDD:
    ADC_calibration_value = ADC_Calibration(ADC0, adcRefVDD);
    break;
  }

  /* Setup the DAC to output 0 V to the ADC with the selected reference */
  /* Single ended mode, both channels */

  DAC_Init_TypeDef dac_init = {
    dacRefresh8,                 /* Refresh every 8 prescaled cycles. */
    ref,                         /* reference. */
    dacOutputPinADC,             /* Output to adc only. */
    dacConvModeContinuous,       /* Continuous mode. */
    0,                           /* No prescaling. */
    false,                       /* Do not enable low pass filter. */
    false,                       /* Do not reset prescaler on ch0 start. */
    false,                       /* DAC output enable always on. */
    false,                       /* Disable sine mode. */
    false                        /* Single ended mode. */
  };

  /* Set prescaler to 500kHz, doesn't really matter */
  dac_init.prescale = DAC_PrescaleCalc(500000, 0);

  DAC_InitChannel_TypeDef dac_initChannel = {
    true,               /* enable channel when init done. */
    false,              /* Disable PRS triggering. */
    false,              /* Channel not refreshed automatically. */
    dacPRSSELCh0        /* Select PRS ch0 (if PRS triggering enabled). */
  };


  /* Reset the DAC */
  DAC_Reset(dac);

  /* Init the DAC */
  DAC_Init(DAC0, &dac_init);

  /* Init the two channels */
  DAC_InitChannel(DAC0, &dac_initChannel, 0);
  DAC_InitChannel(DAC0, &dac_initChannel, 1);

  /* Write 0 to the DAC output */
  dac->CH0DATA = 0;
  dac->CH1DATA = 0;

  /* Now setup the ADC to measure the DAC output */
  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(ADC0);

  ADC_Init_TypeDef       adc_init = {
    adcOvsRateSel4096,             /* max oversampling. */
    adcLPFilterBypass,             /* No input filter selected. */
    adcWarmupNormal,               /* ADC shutdown after each conversion. */
    _ADC_CTRL_TIMEBASE_DEFAULT,    /* Use HW default value. */
    _ADC_CTRL_PRESC_DEFAULT,       /* Use HW default value. */
    false                          /* Do not use tailgate. */
  };

  ADC_InitSingle_TypeDef adc_singleInit = {
    adcPRSSELCh0,               /* PRS ch0 (if enabled). */
    adcAcqTime16,               /* 16 ADC_CLK cycle acquisition time. */
    adcRef1V25,                 /* 1.25V internal reference. */
    adcResOVS,                  /* oversampling bit resolution. */
    adcSingleInpDACOut0,        /* DAC CH0 input selected. */
    false,                      /* Single ended input. */
    false,                      /* PRS disabled. */
    false,                      /* Right adjust. */
    false                       /* Deactivate conversion after one scan sequence. */
  };

  /* Init common settings for both single conversion and scan mode */
  adc_init.timebase = ADC_TimebaseCalc(0);

  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  adc_init.prescale = ADC_PrescaleCalc(7000000, 0);

  /* Set the correct reference according to what reference the DAC uses */
  switch (ref)
  {
  case _DAC_CTRL_REFSEL_1V25:
    adc_singleInit.reference = adcRef1V25;
    break;

  case _DAC_CTRL_REFSEL_2V5:
    adc_singleInit.reference = adcRef2V5;
    break;

  case _DAC_CTRL_REFSEL_VDD:
    adc_singleInit.reference = adcRefVDD;
    break;
  }

  ADC_Init(ADC0, &adc_init);

  ADC_InitSingle(ADC0, &adc_singleInit);

  /* Set adc calibration register back to the value that was found during calibration */
  /* This is necessary since the adc-init routine loads factory-calibration again. */

  ADC0->CAL = ADC_calibration_value;

  /* Now do the calibration */
  /* The routine first adjust the calibration register until the voltage is */
  /* above 0 V, then steps down until the result is just above 0 */
  /* For higher speed calibration this routine could be modified to do a binary */
  /* search like the ADC-calibration routine does. */
  /* Oversampling is used to get higher accuracy */

  /* Cal register for offset is sign magnitude, use signed integer in calibration,*/
  /* and convert to sign magnitude before writing the register */
  cal_value = -31;
  sample    = 4095;

  /* Keep going until we are closer than 1 lsb with 12 bits. */
  /* The sample result is 16 bits because of the oversampling */
  while (sample > 15)
  {
    if (cal_value < 0)
    {
      /* Convert to sign magnitued value with 6 bits */
      cal_value_signmagnitude  = (-cal_value) | 0x20;
      cal_value_signmagnitude &= 0x3f;
    }
    else
    {
      /* Positive numbers are the same for sign magnitude and two's complement */
      cal_value_signmagnitude = cal_value;
    }

    /* Write the new calibration value to the calibration register */
    cal_reg  = dac->CAL & ~(_DAC_CAL_CH0OFFSET_MASK);
    cal_reg |= cal_value_signmagnitude << _DAC_CAL_CH0OFFSET_SHIFT;
    dac->CAL = cal_reg;

    /* Do a conversion */
    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);
    cal_value++;
  }

  /* Now do calibration of channel 1 */
  /* Switch ADC input to DAC1 */
  uint32_t tmp;
  tmp              = ADC0->SINGLECTRL & ~(_ADC_SINGLECTRL_INPUTSEL_MASK);
  tmp             |= ADC_SINGLECTRL_INPUTSEL_DAC0OUT1;
  ADC0->SINGLECTRL = tmp;

  cal_value = -31;

  /* Keep going until we are closer than 1 lsb with 12 bits. */
  /* The sample result is 16 bits because of the oversampling */
  sample = 4095;
  while (sample > 15)
  {
    if (cal_value < 0)
    {
      /* Convert to sign magnitued value with 6 bits */
      cal_value_signmagnitude  = ((-cal_value) | 0x20);
      cal_value_signmagnitude &= 0x3f;
    }
    else
    {
      /* Positive numbers are the same for sign magnitude and two's complement */
      cal_value_signmagnitude = cal_value;
    }

    cal_reg  = dac->CAL & ~(_DAC_CAL_CH1OFFSET_MASK);
    cal_reg |= cal_value_signmagnitude << _DAC_CAL_CH1OFFSET_SHIFT;
    dac->CAL = cal_reg;

    /* Do a conversion */
    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);
    cal_value++;
  }

  /* Now do the gain calibration */
  /* To calibrate the gain, first set the DAC to output the highest voltage */
  /* This is then measured with the ADC and the calibration register is changed */
  /* until the ADC reads almost the highest output value */
  /* Oversampling is used to get higher accuracy. */

  /* The gain is common for both channels, use the last DAC channel. */
  dac->CH1DATA = 4095;

  /* Cal value is now unsigned, higher result leads to lower DAC output. */
  /* Start with lowest output and decrease value during calibration to   */
  /* avoid overflow. */
  cal_value = 0x7F;

  /* Set sample to 0 and continue until the adc-value is close to the top. */
  sample = 0;
  while (sample < 0xFFEC)
  {
    cal_reg  = dac->CAL & ~(_DAC_CAL_GAIN_MASK);
    cal_reg |= cal_value << _DAC_CAL_GAIN_SHIFT;
    dac->CAL = cal_reg;

    /* Do a conversion */
    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);

    /* Decrement cal value */
    cal_value--;
  }

  return dac->CAL;
}

/***************************************************************************//**
 * @brief
 *   Calibrate ADC offset and gain for the specified reference.
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

  /* Init common issues for both single conversion and scan mode */
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
    if (sample > 0xffe0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample < 0xffe0)
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
