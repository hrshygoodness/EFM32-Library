/******************************************************************************
 * @file 2_leds.c
 * @brief Blinking LEDs with STK example
 * @author Silicon Labs
 * @version 1.18
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
#include "em_gpio.h"
#include "em_chip.h"

#if defined ( STK3200 )
  #define LED_PORT    gpioPortC
  #define LED_PIN     10
  #define PB0_PORT    gpioPortC
  #define PB0_PIN     8
#elif defined ( STK3700 )
  #define LED_PORT    gpioPortE
  #define LED_PIN     2
  #define PB0_PORT    gpioPortB
  #define PB0_PIN     9
#elif defined ( STK3300 )
  #define LED_PORT    gpioPortD
  #define LED_PIN     7
  #define PB0_PORT    gpioPortD
  #define PB0_PIN     8
#elif defined ( STKG8XX )
  #define LED_PORT    gpioPortC
  #define LED_PIN     0
  #define PB0_PORT    gpioPortB
  #define PB0_PIN     9
#else
  #error "undefined KIT"
#endif

/******************************************************************************
 * @brief Delay function
 *****************************************************************************/
void Delay(uint16_t milliseconds)
{
  (void) milliseconds;
  /* ENTER YOUR CODE HERE */
}

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();  
  
  /* ENTER YOUR CODE HERE */

  while(1);
  
}
