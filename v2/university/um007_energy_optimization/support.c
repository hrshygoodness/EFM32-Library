/**************************************************************************//**
 * @file
 * @brief Temperature Support Functions
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

#include "efm32.h"

/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @note See section 2.3.4 in the reference manual for details on this
 * calculation
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees Celsius.
 *****************************************************************************/
float convertToCelsius(int32_t adcSample)
{
  float temp;

  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);
  /* Factory calibration value from device information page. */
  int32_t cal_value_0 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                         >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -3.85;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));

  return temp;
}

/**************************************************************************//**
 * @brief Convert ADC sample values to fahrenheit
 * @param adcSample Raw value from ADC to be converted to fahrenheit
 * @return The temperature in degrees Fahrenheit
 *****************************************************************************/
float convertToFahrenheit(uint32_t adcSample)
{
  float celsius;
  float fahrenheit;
  celsius = convertToCelsius(adcSample);

  fahrenheit = (celsius * (9.0 / 5.0)) + 32.0;

  return fahrenheit;
}
