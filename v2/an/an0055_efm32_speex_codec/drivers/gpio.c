/*************************************************************************//**
 * @file gpio.c
 * @brief Setup GPIO for GG STK
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_gpio.h"

static volatile uint8_t keyCode;

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB9)
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  if ((GPIO_IntGet() & (1 << 9)))
  {
    /* Acknowledge interrupt */
    GPIO_IntClear(1 << 9);
    keyCode = 1;
  }
}

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB10)
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  if ((GPIO_IntGet() & (1 << 10)))
  {
    /* Acknowledge interrupt */
    GPIO_IntClear(1 << 10);
    keyCode = 2;
  }
} 

/**************************************************************************//**
 * @brief Setup GPIO interrupt to change demo mode
 *****************************************************************************/
void initGpio(void)
{
  /* Configure PB9 as input and enable interrupt */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Configure PB10 as input and enable interrupt  */
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInputPull, 1); 
  GPIO_IntConfig(gpioPortB, 10, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 3, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief Check which key press
 *****************************************************************************/
uint8_t keyCheck(void)
{
  uint8_t i;
  
  i = keyCode;
  keyCode = 0;
  return i;
}
