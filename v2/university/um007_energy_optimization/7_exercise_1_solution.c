/**************************************************************************//**
 * @file
 * @brief Exercise 1 - Improved Version
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

#include <stdlib.h>
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_lcd.h"
#include "em_rtc.h"
#include "em_gpio.h"

/* Drivers */
#include "segmentlcd.h"

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortB
#define PB0_PIN     9

#else 

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortD
#define PB0_PIN     8

#endif

/* Time of the stopwatch */
uint32_t time;

/* Enter your code here */

/* States */
bool enableStopwatch = false;
bool startup         = true;

/**************************************************************************//**
 * @brief GPIO interrupt handler
 * Manage start/stop of stopwatch
 *****************************************************************************/
void GPIO_IRQHandler(void)
{
  GPIO_IntClear(1 << PB0_PIN);

  if (!enableStopwatch)
  {
    /* Reset time */
    time = 0;

    /* Prescale the RTC clock and set to overflow every .1 seconds */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);
    RTC_CompareSet(0, 3276);

    /* Start the RTC */
    RTC_Enable(true);

    /* Enable the timer  */
    enableStopwatch = true;

  }
  else
  {
    /* Disable the RTC, and clear existing interrupts */
    RTC_Enable(false);
    RTC_IntClear(RTC_IFC_COMP0);

    /* Disable stopwatch */
    enableStopwatch = false;

    /* Prescale the RTC and set comparison */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32768);
    RTC_CompareSet(0, 3);

    /* The result will be displayed untill the next RTC overflow */
    RTC_Enable(true);
  }
}

/**************************************************************************//**
 * @brief GPIO Even interrupt handler
 * Manage start/stop of stopwatch
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_IRQHandler();
}

/**************************************************************************//**
 * @brief GPIO Odd interrupt handler
 * Manage start/stop of stopwatch
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_IRQHandler();
}


/**************************************************************************//**
 * @brief Configure GPIO
 *****************************************************************************/
void gpio_setup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure interrupt when Push Button 0 is pressed */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif
}


/**************************************************************************//**
 * @brief RTC interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);

  /* Interrupt after delay at startup */
  if (startup)
  {
    startup = false;
    RTC_Enable(false);
    return;
  }

  /* Increment the time while the stopwatch is active, otherwise prepare to
   * enter EM3 after having displayed the result for some time */
  if (enableStopwatch)
  {
    time++;
  }
  else
  {
    RTC_Enable(false);
  }
}

/**************************************************************************//**
 * @brief Configure RTC
 *****************************************************************************/
void rtc_setup(void)
{
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

  /* Set overflow to 0.1 seconds - this will be used for the startup delay */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);
  RTC_CompareSet(0, 3276);

  /* Enable interrupt for compare register 0 */
  RTC_IntEnable(RTC_IFC_COMP0);

  /* Enabling interrupt from RTC */
  NVIC_EnableIRQ(RTC_IRQn);
}

/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Enable the HFPER clock */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Configure RTC and GPIO */
  rtc_setup();
  gpio_setup();

  /* Ensure that RTC and GPIO has the same priority so that they do not
   * interrupt each other */
  NVIC_SetPriority(GPIO_ODD_IRQn, 0);
  NVIC_SetPriority(RTC_IRQn, 0);

  /* Wait for settings to be properly set before entering EM3 - RTC interrupt */
  EMU_EnterEM2(true);


  /* Stay in this loop forever */
  while (1)
  {
  
    SegmentLCD_Number(time);

    /* LCD and RTC require at least EM2 */
    EMU_EnterEM2(true);

  }
}