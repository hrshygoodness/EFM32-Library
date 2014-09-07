/**************************************************************************//**
 * @file
 * @brief Energy Modes Exercise Support Function
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
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "bsp.h"

/* Header file for support functions */
#include "4_exercise.h"

#if defined(_EFM32_GIANT_FAMILY)
  /* Defines for Push Button 0 */
  #define PB0_PORT                    gpioPortB
  #define PB0_PIN                     9
#else
  /* Defines for Push Button 0 */
  #define PB0_PORT                    gpioPortD
  #define PB0_PIN                     8
#endif

/**************************************************************************//**
 * @brief RTC Handler - simply clear the interrupt
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
}

/**************************************************************************//**
 * @brief Configure RTC
 *****************************************************************************/
void rtc_setup(void)
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

  /* Produce an interrupt every second */
  RTC_CompareSet(0, 32768);

  /* Enable interrupt for compare register 0 */
  RTC_IntEnable(RTC_IFC_COMP0);

  /* Enabling interrupt from RTC */
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * @brief USART1 Handler - simply clear the interrupt
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  /* Clear interrupt source */
  USART_IntClear(USART1, USART_IF_TXC);
}

/**************************************************************************//**
 * @brief Configure USART
 *****************************************************************************/
void usart_setup(void)
{
  USART_TypeDef           *usart = USART1;
  USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Output on pin D0 */
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1);

  /* Enable pins at default location */
  usart->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC1;

  /* Configure USART for basic async operation */
  init.enable   = usartDisable;
  init.baudrate = 9600;
  USART_InitAsync(usart, &init);

  /* Enable interrupts */
  NVIC_EnableIRQ(USART1_TX_IRQn);
  USART_IntEnable(USART1, USART_IF_TXC);

  /* Finally enable it */
  USART_Enable(usart, usartEnable);
}

/**************************************************************************//**
 * @brief GPIO Even Handler - simply clear the interrupt
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Clear interrupt source */
  GPIO_IntClear(1 << PB0_PIN);
}

/**************************************************************************//**
 * @brief GPIO Even Handler - simply clear the interrupt
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Clear interrupt source */
  GPIO_IntClear(1 << PB0_PIN);
}

/**************************************************************************//**
 * @brief Configure GPIO
 *****************************************************************************/
void gpio_setup(void)
{
  BSP_LedsInit();

  /* Configure interrupt for Push Button 0 (D8) */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);
}
