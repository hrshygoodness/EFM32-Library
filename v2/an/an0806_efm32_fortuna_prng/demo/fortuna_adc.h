/**************************************************************************//**
 * @file fortuna_adc.h
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

#ifndef __FORTUNA_ADC_H
#define __FORTUNA_ADC_H

/* Configure ADC for sampling the temperature sensor. */
void     AdcRandomInitialize(void);

/* Get a sample from the ADC. */
uint32_t AdcSampleGet(void);


#endif /*__FORTUNA_ADC_H */
