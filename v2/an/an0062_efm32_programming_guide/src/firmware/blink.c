/**************************************************************************//**
 * @file blink.c
 * @brief Sample firmware for use with EFM32 Demo Programmer
 * @author Silicon Labs
 * @version 1.03
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
#include "em_gpio.h"

/******************************************************************************
 * 
 * This firmware will blink an LED on any EFM32 starter kit. 
 * 
 ******************************************************************************/ 



/* Make binary larger so we can measure transfer speed more accurately. 
 * This requires the firmware_end section to be defined in linker file, 
 * see blink.icf.
 */
#pragma location = "firmware_end"
const uint32_t firmwareEnd = 0;

int main(void)
{
  /* Make loop variable volatile so compiler does not optimize away the delay loop */
  volatile int i;
    
  /* Get device family from DI page */
  uint32_t deviceFamily = (DEVINFO->PART & _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;
  
  GPIO_Port_TypeDef ledPort;
  int ledPin;
  uint32_t gpioClock;
  
  /* Retrieve the correct pin to toggle for each kit. 
   * We also need to get the correct bit-field to set
   * in HFPERCLKEN0 in order to turn on the GPIO clock. 
   */
  switch (deviceFamily) {
  case _DEVINFO_PART_DEVICE_FAMILY_GG:
  case _DEVINFO_PART_DEVICE_FAMILY_LG:
  case _DEVINFO_PART_DEVICE_FAMILY_WG:
    ledPort = gpioPortE;
    ledPin = 2;
    gpioClock = 1 << 13;
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_TG:
    ledPort = gpioPortD;
    ledPin = 7;
    gpioClock = 1 << 6;
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_G:
    ledPort = gpioPortC;
    ledPin = 0;
    gpioClock = 1 << 12;
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_ZG:
    ledPort = gpioPortC;
    ledPin = 10;
    gpioClock = 1 << 7;
    break;
  default:
    ledPort = gpioPortE;
    ledPin = 2;
    gpioClock = 1 << 13;
    break;
  }
     
  /* Enable clock to GPIO */
  CMU->HFPERCLKEN0 |= gpioClock;
    
  /* Set LED pin to push-pull */
  GPIO_PinModeSet(ledPort, ledPin, gpioModePushPull, 0);
      
  while(1) 
  {
    /* Toggle pin */
    GPIO_PinOutToggle(ledPort, ledPin);
    
    /* Simple delay */
    for ( i=0; i<100000; i++ );
  }
}