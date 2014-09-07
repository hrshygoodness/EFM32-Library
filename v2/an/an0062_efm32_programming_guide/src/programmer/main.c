/**************************************************************************//**
 * @file main.c
 * @brief Entry point for EFM32 Programmer
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
#include <stdint.h>
#include <stdbool.h>
#include <setjmp.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "dap.h"
#include "utils.h"
#include "state_machine.h"
#include "em_ebi.h"
#include "bsp.h"
#include "delay.h"
#include "kits.h"

#include "errors.h"


/**********************************************************
 * Sets up pins and clocks used for by application. 
 **********************************************************/
void init(void)
{
  /* Select 48 MHz external crystal oscillator */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Enable clock to GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Power up trace and debug clocks. Needed for DWT. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
  /* Enable DWT cycle counter. Used to measure transfer speed. */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  
  /* Set up TIMER and RTC for delay functions */
  initDelay();
  
  /* Set up correct pins and user feedback for kit */
  initKit();
}

int main(void)
{
  /* Handle chip erratas */
  CHIP_Init();
  
  /* Initalize clocks and peripherals */
  init();
  
  /* Printf statements are output on SWO */
  printf("EFM32 Demo Programmer started\n");
  
  /* Enter main loop. See state_machine.c */
  stateLoop(); 

  /* This is never reached */
  return 0;
}
