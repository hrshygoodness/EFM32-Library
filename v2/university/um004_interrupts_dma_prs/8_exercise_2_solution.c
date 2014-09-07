/**************************************************************************//**
 * @file
 * @brief Exercise 2
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
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "em_rtc.h"

/* Drivers */
#include "segmentlcd.h"

/* Timing defines */
#define LFXO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS          1000
#define RTC_COUNT_BETWEEN_WAKEUP    ((LFXO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 & 1 */
#define PB0_PORT                    gpioPortB
#define PB0_PIN                     9
#define PB1_PORT                    gpioPortB
#define PB1_PIN                     10

#else

/* Defines for Push Button 0 & 1 */
#define PB0_PORT                    gpioPortD
#define PB0_PIN                     8
#define PB1_PORT                    gpioPortB
#define PB1_PIN                     11

#endif

/* The time of the stopwatch*/
uint32_t time = 10;
uint32_t old_time;

/* Increment the stopwatch? */
bool enableCount = false;

/* Which buttons are pressed? */
bool pb0_is_pushed = false;
bool pb1_is_pushed = false;

/* Timer status */
bool toggleTimer = false;
bool enableTimer = false;

/**************************************************************************//**
 * @brief Button Handler
 * Shared by GPIO Even and GPIO Odd interrupt handlers
 *****************************************************************************/
void button_handler()
{
  uint32_t interrupt_source = GPIO_IntGet();

  /* Both buttons were pushed - toggle the timer */
  if (pb1_is_pushed && pb0_is_pushed)
  {
    toggleTimer = true;
  }

  if (toggleTimer)
  {
    /* Wait for both buttons to be released */
    if (!pb1_is_pushed && !pb0_is_pushed)
    {
      if (enableTimer)
      {
        SegmentLCD_Write("STOP");
        time = old_time;
        SegmentLCD_Number(time);
        /* Disable RTC */
        RTC_Enable(false);
      }
      else
      {
        SegmentLCD_Write("START");
        old_time = time;
        /* Enable RTC */
        RTC_Enable(true);
      }
      enableTimer = !enableTimer;
      toggleTimer = false;
      return;
    }
  }
  else
  {
    /* If only Push Button 0 was pressed - decrease the time */
    if (!pb0_is_pushed && interrupt_source & 1 << PB0_PIN)
    {
      if (time > 0)
        time--;
      SegmentLCD_Number(time);
    }
    /* If only Push Button 1 was pressed - increase the time */
    else if (!pb1_is_pushed && interrupt_source & 1 << PB1_PIN)
    {
      if (time < 9999)
        time++;
      SegmentLCD_Number(time);
    }
  }
}


/**************************************************************************//**
 * @brief GPIO Interrupt Handler
 *****************************************************************************/
void GPIO_IRQHandler_1(void)
{
  pb0_is_pushed = !pb0_is_pushed;
  button_handler();
  GPIO_IntClear(1 << PB0_PIN);
}

/**************************************************************************//**
 * @brief GPIOInterrupt Handler
 *****************************************************************************/
void GPIO_IRQHandler_2(void)
{
  pb1_is_pushed = !pb1_is_pushed;
  button_handler();
  GPIO_IntClear(1 << PB1_PIN);
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
 * @brief Real Time Counter Interrupt Handler
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);

  time--;
  SegmentLCD_Number(time);
  if (time == 0)
  {
    SegmentLCD_Write("DONE");
    time = old_time;

    /* Disable RTC */
    RTC_Enable(false);
    enableTimer = false;
  }
  SegmentLCD_Number(time);
}

/**************************************************************************//**
 * @brief Initialize Real Time Counter
 *****************************************************************************/
void initRTC()
{
  /* Starting LFXO and waiting until it is stable */
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

  /* Set comapre value for compare register 0 */
  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

  /* Enable interrupt for compare register 0 */
  RTC_IntEnable(RTC_IFC_COMP0);

  /* Enabling Interrupt from RTC */
  NVIC_EnableIRQ(RTC_IRQn);

  RTC_Enable(false);
}

/**************************************************************************//**
 * @brief Initialize General Purpuse Input/Output
 *****************************************************************************/
void initGPIO()
{
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure pin PB9/PD8 (Push Button 1) and PB10/PB11 (Push Button 2) as an input,
   * so that we can read their values. */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 1);

  /* Enable GPIO_ODD and GPIO_EVEN interrupts in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Configure interrupts on falling edge for pins B9/D8 (Push Button 1),
   * B10/B11 (Push Button 2) and D3 */
  GPIO_IntConfig(PB0_PORT, PB0_PIN, true, true, true);
  GPIO_IntConfig(PB1_PORT, PB1_PIN, true, true, true);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  SegmentLCD_Init(true);

  initRTC();
  initGPIO();

  /* Initial LCD content */
  SegmentLCD_Number(time);

  while (1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(true);
    /* Wait for interrupts */
  }
}
