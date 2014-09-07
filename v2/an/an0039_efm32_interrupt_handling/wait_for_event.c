/***************************************************************************//**
 * @file wait_for_event.c
 * @brief Wait-for-Event (WFE) Example for EFM32.
 * @author Silicon Labs
 * @version 1.05
 *******************************************************************************
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
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_chip.h"

#if defined ( STK3700 )
  #define LED_PORT 	gpioPortE
  #define LED_PIN  	2
  #define PB0_PORT 	gpioPortB
  #define PB0_PIN 	9
#elif defined ( STK3300 )
  #define LED_PORT 	gpioPortD
  #define LED_PIN  	7
  #define PB0_PORT 	gpioPortD
  #define PB0_PIN 	8
#else
  #error "undefined KIT"
#endif

/**************************************************************************//**
 * @brief Configure GPIOs for LED and push button (with interrupt)
 *****************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure LED_PIN as push pull */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull,0);

  /* Configure Push button 0 as input */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 0);
  
  /* Set falling edge interrupt for Push button 0 */
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Set the clock frequency to fastest available on HFRCO*/
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  
  /* Configure gpio */
  gpioSetup();
 
  /* Set the SEVONPEND bit. Will trigger an event on pending interrupt. */
  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
  
  /* Deep Sleep when waiting for event, i.e. go to EM3 */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  /* Set event and clear it to put the event register bit in a known state
    (cleared) before going to sleep by waiting for the next event */
  __SEV();
  __WFE();
  
  while(1)
  { 
    /* Go to sleep while waiting for event (set by IRQ pending bit) */
    /* Since IRQ is not enabled, no ISR will be entered */
    __WFE();
    
    /* Toggle LED */
    GPIO->P[LED_PORT].DOUTTGL = 1<<LED_PIN;
    
    /* Clear the interrupt flag */
    GPIO_IntClear(1 << PB0_PIN);
    
	#if defined ( STK3700 )
		/* Clear pending GPIO ODD IRQ */
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);	
	#elif defined ( STK3300 )
		/* Clear pending GPIO EVEN IRQ */
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	#endif
  }
  
}  
