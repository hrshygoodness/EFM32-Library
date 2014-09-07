/******************************************************************************
 * @file 5_rtc_delay.c
 * @brief RTC Delay Exercise
 * @author Silicon Labs
 * @version 1.01
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
#include "em_emu.h"

// Uncomment the following lines for STK3700
#define LED_PORT 	gpioPortE
#define LED_PIN  	2
#define PB0_PORT 	gpioPortB
#define PB0_PIN 	9

// Uncomment the following lines for STK3300
// #define LED_PORT 	gpioPortD
// #define LED_PIN  	7
// #define PB0_PORT 	gpioPortD
// #define PB0_PIN 	8

// Uncomment the following lines for STKG8XX
// #define LED_PORT 	gpioPortC
// #define LED_PIN  	0
// #define PB0_PORT 	gpioPortB
// #define PB0_PIN		9


#define RTC_FREQ 32768



void initRTC(void)
{
  /* ENTER YOUR CODE HERE */

  /* Configure to overflow every 50ms */

  /* Configure and enable RTC. Fill in this struct */
  RTC_Init_TypeDef rtcInit;
  rtcInit.comp0Top = ;
  rtcInit.debugRun = ;
  rtcInit.enable   = ;
  RTC_Init(&rtcInit);

  /* Enable overflow interrupt */
}


int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Enable clock for GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure LED pin as push/pull output */
  GPIO_PinModeSet(LED_PORT,         /* Port */
                  LED_PIN,          /* Pin */
                  gpioModePushPull, /* Mode */
                  0 );              /* Output value */

  /* WRITE YOUR CODE HERE */

  /* Start LFXO and wait until it is stable */

  /* Select LFXO as the clock source for RTC clock domain */

  /* Enable clock to RTC */

  /* Enable clock to the interface of the low energy modules */


  /* Configure RTC */
  initRTC();

  /* Stay in this loop forever */
  while (1) {
    /* Enter EM2 */
    EMU_EnterEM2(false);
  }
}
