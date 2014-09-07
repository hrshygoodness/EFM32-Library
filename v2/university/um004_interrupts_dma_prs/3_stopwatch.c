/**************************************************************************//**
 * @file
 * @brief Stopwatch Example
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
#include "em_lcd.h"
#include "segmentlcd.h"
#include "em_rtc.h"


/* Defines for the RTC */
#define LFXO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS          10
#define RTC_COUNT_BETWEEN_WAKEUP    ((LFXO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)

/* The time of the stopwatch*/
uint32_t time = 0;

/* Increment the stopwatch? */
bool enableCount = false;

/* Display the gecko on the LCD? */
bool enableGecko = false;

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

/**************************************************************************//**
 * @brief GPIO Interrupt Handler
 *****************************************************************************/
void GPIO_IRQHandler_1(void)
{
  /* Clear flag for Push Button 1 (pin D8) interrupt */
  GPIO_IntClear(1 << PB0_PIN);

  /* Toggle enableCount to start/pause the stopwatch */
  if (enableCount)
  {
    enableCount = false;
    SegmentLCD_Write("Pause");
  }
  else
  {
    /* The interrupt handler is called to compute the proper time
     * for the next interrupt, since enableCount is 0, the time will not
     * be increased. */
    RTC_IRQHandler();

    SegmentLCD_Write("Start");
    enableCount = true;
  }
}

/**************************************************************************//**
 * @brief GPIO Interrupt Handler
 *****************************************************************************/
void GPIO_IRQHandler_2(void)
{
  /* Get the interrupt source, either Push Button 2 (pin B11) or pin D3 */
  uint32_t interrupt_source = GPIO_IntGet();

  /* Push Button 2 (pin B11) */
  if (interrupt_source & (1 << PB1_PIN))
  {
    GPIO_IntClear(1 << PB1_PIN);
    SegmentLCD_Write("Clear");
    time        = 0;
    enableCount = false;
    SegmentLCD_Number(time);
  }

  /* Pin D3 - channel 3 => 2^3 */
  if (interrupt_source & (1 << 3))
  {
    /* Operations can be made atomic, i.e. it cannot be interrupted by
     * interrupts with higher priorities, by disabling iterrupts. Uncomment
     * __disable_irq(); and __enable_irq(); to see how the update of the time
     * is delayed by the dummy loop below.*/
    /* __disable_irq(); */

    /* Toggle enableGecko */
    if (enableGecko)
      enableGecko = false;
    else
      enableGecko = true;

    SegmentLCD_Symbol(LCD_SYMBOL_GECKO, enableGecko);

    /* This dummy loop is intended to illustrate the different levels of
     * priority. The timer will continue to update the LCD display, but
     * interrupts produced by Push Button 1 and 2 will not be served until
     * after this function has finished. */
    for (uint32_t tmp = 0; tmp < 2000000; tmp++) ;

    GPIO_IntClear(1 << 3);

    /* Enable interrupts again */
    /* __enable_irq(); */
  }
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

  /* Increase the timer when appropriate. */
  if (enableCount)
    time++;

  /* Only update the number if necessary */
  if (enableCount)
    /* Set lower priority interrupt which will process data */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}



/**************************************************************************//**
 * @brief Software Interrupt Handler
 *****************************************************************************/
void PendSV_Handler(void)
{
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
}

/**************************************************************************//**
 * @brief Initialize General Purpuse Input/Output
 *****************************************************************************/
void initGPIO()
{
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure pin PD8/PB9 (Push Button 0) and PB11/PB10 (Push Button 1) as an input,
   * so that we can read their values. */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 1);

  /* Configure pin PD3 as an input. Unlike Push Button 0 and 1 this pin
   * does not have a pull down associated with it. We therefore need to set the
   * mode to InputPull, in order to set a default value of 1 with PinOutSet(). */
  GPIO_PinModeSet(gpioPortD, 3, gpioModeInputPull, 1);
  GPIO_PinOutSet(gpioPortD, 3);

  /* Configure PC0 as a push pull for LED drive */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

  /* Enable GPIO_ODD and GPIO_EVEN interrupts in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(PendSV_IRQn);

  /* Set priorities - 0 is the highest, 7 is the lowest */
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_SetPriority(GPIO_ODD_IRQn, 2);
  NVIC_SetPriority(GPIO_EVEN_IRQn, 1);
#else
  NVIC_SetPriority(GPIO_EVEN_IRQn, 2);
  NVIC_SetPriority(GPIO_ODD_IRQn, 1);
#endif
  NVIC_SetPriority(RTC_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 3);

  /* Configure interrupts on falling edge for pins D8/B9 (Push Button 0),
   * B11/B10 (Push Button 1) and D3 */
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);
  GPIO_IntConfig(gpioPortD, 3, false, true, true);
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
  SegmentLCD_Write("Welcome");
  SegmentLCD_Number(0);

  while (1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(true);
    /* Wait for interrupts */
  }
}
