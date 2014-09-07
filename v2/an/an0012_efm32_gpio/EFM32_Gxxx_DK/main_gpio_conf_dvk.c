/*****************************************************************************
 * @file main_gpio_conf_dvk.c
 * @brief GPIO Demo Application
 * @author Silicon Labs
 * @version 1.07
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
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_chip.h"

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();
  
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PD with alternate drive strength of 20mA */
  GPIO_DriveModeSet(gpioPortD, gpioDriveModeHigh);
  
  /* Configure PC12 as input with filter*/
  GPIO_PinModeSet(gpioPortC, 12, gpioModeInput, 0);
  
  /* Configure PD8 as push pull output */
  GPIO_PinModeSet(gpioPortD, 8, gpioModePushPullDrive, 0);
  
  while(1)
  {
    /* If PC12 is high, drive high PD8, else drive low */
    if(GPIO_PinInGet(gpioPortC, 12)) 
      GPIO_PinOutSet(gpioPortD, 8);   /* Drive high PD8 */ 
    else 
      GPIO_PinOutClear(gpioPortD, 8); /* Drive low PD8 */
  }
  
 
}

