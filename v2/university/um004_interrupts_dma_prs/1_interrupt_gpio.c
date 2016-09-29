/**************************************************************************//**
 * @file
 * @brief GPIO Interrupt Example
 * @author Energy Micro AS
 * @version 1.0.0
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
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

/* Since the code can be run on both Giant Gecko and Tiny Gecko we need the following
   if statement since the pins for Push Button 0 and Push Button 1 are different for 
   the two chips */

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortB
#define PB0_PIN     9

/* Defines for the LED */
#define LED_PORT    gpioPortE
#define LED_PIN     2

#else 

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortD
#define PB0_PIN     8

/* Defines for the LED */
#define LED_PORT    gpioPortD
#define LED_PIN     7

#endif 



bool enableLed = 0;

/**************************************************************************//**
 * @brief GPIO Handler
 * Interrupt Service Routine 
 *****************************************************************************/

void GPIO_IRQHandler(void)
{
  /* Clear flag for Push Button 0  interrupt. The interrupt is called as
   * long as the interrupt flag is not cleared, so failing to do so here would
   * lock the program at the first interrupt.
   *
   * All the ports share a total of 16 interrupts - one per pin number - i.e.
   * pin 9 of port A and D share one interrupt, so to clear interrupts produced by
   * either one of them we have to clear bit 9 */
  GPIO_IntClear(1 << PB0_PIN);

  /* Toggle value */
  enableLed = !enableLed;

  if (enableLed)
  {
    /* Turn on LED */
    GPIO_PinOutSet(LED_PORT, LED_PIN);
  }
  else
  {
    /* Turn off LED )*/
    GPIO_PinOutClear(LED_PORT, LED_PIN);
  }
}

/**************************************************************************//**
 * @brief GPIO_EVEN Handler
 * Interrupt Service Routine for even GPIO pins
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
    GPIO_IRQHandler();
}

/**************************************************************************//**
 * @brief GPIO_ODD Handler
 * Interrupt Service Routine for odd GPIO pins
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
    GPIO_IRQHandler();
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Enable clock for GPIO module, we need this because
   *  the button and the LED are connected to GPIO pins. */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure push button 1 as an input,
   * so that we can read its value. */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);

  /* Configure LED as a push pull, so that we can
   * set its value - 1 to turn on, 0 to turn off */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

  /* Enable GPIO_ODD interrupt vector in NVIC. We want Push Button 1 to
   * send an interrupt when pressed. GPIO interrupts are handled by either
   * GPIO_ODD or GPIO_EVEN, depending on wheter the pin number is odd or even,
   * PB1 is therefore handled by GPIO_EVEN for Tiny Gecko (D8) and by GPIO_ODD for 
   * Giant Gecko (B9). */
  
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif

  /* Configure PD8 (Push Button 1) interrupt on falling edge, i.e. when it is
   * pressed, and rising edge, i.e. when it is released. */
  GPIO_IntConfig(PB0_PORT, PB0_PIN, true, true, true);

  while (1)
  {
    /* Go to EM3 */
    EMU_EnterEM3(true);
  }
}
