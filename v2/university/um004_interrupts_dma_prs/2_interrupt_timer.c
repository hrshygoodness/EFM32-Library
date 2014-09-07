/**************************************************************************//**
 * @file
 * @brief RTC Interrupt Example
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
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_rtc.h"

/* Defines for the LED */
#if defined(_EFM32_GIANT_FAMILY)

#define LED_PORT    gpioPortE
#define LED_PIN     2

#else

#define LED_PORT     gpioPortD
#define LED_PIN      7

#endif

/* Defines for the RTC */
#define LFXO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS          1000
#define RTC_COUNT_BETWEEN_WAKEUP    ((LFXO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)

/**************************************************************************//**
 * @brief RTC Handler
 * Interrupt Service Routine for Real Time Counter
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);

  /* Toggle the LED */
  GPIO_PinOutToggle(LED_PORT, LED_PIN);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Starting low frequency crystal oscillator and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Routing the LFXO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  const RTC_Init_TypeDef rtcInit =
  {
    .enable   = true,
    .debugRun = false,
    .comp0Top = true,
  };

  RTC_Init(&rtcInit);


  /* Set compare register 0 to the first compare value, i.e. when to send out
   * the interrupt */
  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

  /* Enable interrupt for compare register 0 */
  RTC_IntEnable(RTC_IFC_COMP0);

  /* Enabling interrupt from RTC */
  NVIC_EnableIRQ(RTC_IRQn);

  /* Enable clock for GPIO module (because we are using the LED */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure LED as a push pull, so that we can set its value */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

  while (1)
  {
    /* Go to EM2 (RTC needs at least EM2 to operate,
     * so lower energy modes are not possible) */
    EMU_EnterEM2(true);
  }
}
