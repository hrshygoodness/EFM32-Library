/**************************************************************************//**
 * @file
 * @brief Exercise 1
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 & 1 */
#define PB0_PORT                    gpioPortB
#define PB0_PIN                     9
#define PB1_PORT                    gpioPortB
#define PB1_PIN                     10

/* Defines for the LED */
#define LED_PORT                    gpioPortE
#define LED_PIN                     2


#else

/* Defines for Push Button 0 & 1 */
#define PB0_PORT                    gpioPortD
#define PB0_PIN                     8
#define PB1_PORT                    gpioPortB
#define PB1_PIN                     11

/* Defines for the LED */
#define LED_PORT                    gpioPortD
#define LED_PIN                     7

#endif

/* Drive LED? */
bool enable_led = false;

/**************************************************************************//**
 * @brief Toggle LED if both PB1 and PB2 is pushed
 * Shared by GPIO Even and GPIO Odd interrupts handlers
 *****************************************************************************/
void toggle()
{
  if (!GPIO_PinInGet(PB0_PORT, PB0_PIN) && !GPIO_PinInGet(PB1_PORT, PB1_PIN))
  {
    enable_led = !enable_led;

    if (enable_led)
    {
      /* Turn off LED (pin D7)*/
      GPIO_PinOutSet(LED_PORT, LED_PIN);
    }
    else
    {
      /* Turn off LED (pin D7)*/
      GPIO_PinOutClear(LED_PORT, LED_PIN);
    }
  }
}



/**************************************************************************//**
 * @brief GPIO Even Interrupt Handler
 * Triggered by Push Button 0
 *****************************************************************************/
void GPIO_IRQHandler_1(void)
{
  GPIO_IntClear(1 << PB0_PIN);

  toggle();
}

/**************************************************************************//**
 * @brief GPIO Odd Interrupt Handler
 * Triggered by Push Button 1
 *****************************************************************************/
void GPIO_IRQHandler_2(void)
{
  GPIO_IntClear(1 << PB1_PIN);

  toggle();
}

/**************************************************************************//**
 * @brief GPIO Even Interrupt Handler
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void){
  
#if defined(_EFM32_GIANT_FAMILY)
  GPIO_IRQHandler_2();
#else
  GPIO_IRQHandler_1();
#endif
  
}

/**************************************************************************//**
 * @brief GPIO Odd Interrupt Handler
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void){
  
#if defined(_EFM32_GIANT_FAMILY)
  GPIO_IRQHandler_1();
#else
  GPIO_IRQHandler_2();
#endif
  
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

  /* Configure pin PD8 (push button 1) as an input,
   * so that we can read its value. */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 1);

  /* Configure PD7 (LED) as a push pull, so that we can
   * set its value - 1 to turn on, 0 to turn off */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

  /* Enable GPIO_ODD interrupt vector in NVIC. We want Push Button 1 to
   * send an interrupt when pressed. GPIO interrupts are handled by either
   * GPIO_ODD or GPIO_EVEN, depending on wheter the pin number is odd or even,
   * D8 is therefore handled by GPIO_EVEN. */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Configure PD8 (Push Button 1) interrupt on falling edge, i.e. when it is
   * pressed, and rising edge, i.e. when it is released. */
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);

  while (1)
  {
    /* Go to EM3 */
    EMU_EnterEM3(true);
  }
}
