/**************************************************************************//**
 * @file
 * @brief Exercise 1 - Naive Version
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


/**************************************************************************//**
 * @brief Configure GPIO
 *****************************************************************************/
void gpio_setup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
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

  /* Set overflow every 0.1 seconds */
  RTC_CompareSet(0, 3276);
}

/**************************************************************************//**
 * @brief SWO Setup
 * Enables code view in energyAware Profiler
 *****************************************************************************/
void setupSWO(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;
  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  /* Enable code view */
  setupSWO();

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Enable the HFPER clock */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Configure RTC and GPIO */
  rtc_setup();
  gpio_setup();

  /* Stay in this loop forever */
  while (1)
  {
    /* Wait for Push Button 1 to be pressed */
    while (GPIO_PinInGet(PB0_PORT, PB0_PIN)) ;
    /* and released, so that we do not exit the while loop below immediately */
    while (!GPIO_PinInGet(PB0_PORT, PB0_PIN)) ;

    /* Reset time */
    time = 0;

    /* Disable LCD */
    LCD_Enable(true);

    /* Update time until Push Button 1 is pressed again */
    while (1)
    {
      if (RTC_CounterGet() == 0)
      {
        SegmentLCD_Number(++time);
      }

      if (!GPIO_PinInGet(PB0_PORT, PB0_PIN))
        break;
    }

    /* Delay while showing the result on the LCD */
    for (uint32_t delay = 0; delay < 30; delay++)
    {
      /* Wait for the RTC to overflow once */
      while (RTC_CounterGet() != 0) ;
      while (RTC_CounterGet() == 0) ;
    }

    /* Disable LCD */
    LCD_Enable(false);
  }
}
