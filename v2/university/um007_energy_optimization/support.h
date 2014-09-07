/******************************************************************************
 * @file
 * @brief ADC conversion using Timer and polling
 * @author Energy Micro AS
 * @version 1.00
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

#ifndef __SUPPORT_H
#define __SUPPORT_H

float convertToCelsius(uint32_t adcSample);
float convertToFahrenheit(uint32_t adcSample);

#endif
