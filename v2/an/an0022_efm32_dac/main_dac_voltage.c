/*****************************************************************************
 * @file main_dac_voltage.c
 * @brief Digital to Analog converter, output voltage example
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
#include "em_emu.h"
#include "em_dac.h"


/* Function prototypes */
void DAC_setup(void);
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch);

/**************************************************************************//**
 * @brief  Main function
 * Setup of dac and calculating output value, enter em1 while dac is working
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  uint32_t DAC_Value;

  /* Initialise the DAC */
  DAC_setup();

  /* Enable DAC channel 0, located on pin PB11 */
  DAC_Enable(DAC0, 0, true);

  /* Calculate output to 0.5 V. */
  DAC_Value = (uint32_t)((0.5 * 4096) / 1.25);

  /* Write the new value to DAC register */
  DAC_WriteData(DAC0, DAC_Value, 0);

  /* Enter EM1 while the DAC is doing continuous conversions. */
  EMU_EnterEM1();

  /* Should never get here */
  return 0;
}


/**************************************************************************//**
 * @brief  Setup DAC
 * Configures and starts the DAC
 *****************************************************************************/
void DAC_setup(void)
{
  /* Use default settings */
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Enable the DAC clock */
  CMU_ClockEnable(cmuClock_DAC0, true);

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 500kHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the current value actually is. */
  init.prescale = DAC_PrescaleCalc(500000, 0);

  /* Set reference voltage to 1.25V */
  init.reference = dacRef1V25;

  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);
  DAC_InitChannel(DAC0, &initChannel, 0);
}


/**************************************************************************//**
 * @brief  Write DAC conversion value
 *****************************************************************************/
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch)
  {
    dac->CH0DATA = value;
  }
  else
  {
    dac->CH1DATA = value;
  }
}

