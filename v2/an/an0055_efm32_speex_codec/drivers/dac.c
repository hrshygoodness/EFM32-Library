/**************************************************************************//**
 * @file dac.c
 * @brief Setup DAC for audio output
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

#include "em_dac.h"
#include "config.h"

/***************************************************************************//**
 * @brief
 *   Configure DAC usage for this application.
 *******************************************************************************/
void initDac(void)
{
  /* Use default settings */
  DAC_Init_TypeDef init = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Default reference voltage is 1.25V */
#if DAC_REF_SELECT == 1
  /* Set reference voltage to 2.5V */
  init.reference = dacRef2V5;
#elif DAC_REF_SELECT == 2
  /* Set reference voltage to VDD */
  init.reference = dacRefVDD;
#endif
  
  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to DAC_CLOCK. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(DAC_CLOCK, 0);
  
  /* Initialize the DAC. */
  DAC_Init(DAC0, &init);

  /* Enable PRS CH to trigger samples at the right time with the timer */
  initChannel.prsSel = DAC_PRS_SEL;

  /* Initialize DAC channel */
  DAC_InitChannel(DAC0, &initChannel, DAC_CHANNEL);
}
