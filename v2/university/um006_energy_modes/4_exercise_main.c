/**************************************************************************//**
 * @file
 * @brief Exercise
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
#include "em_emu.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_gpio.h"

/* Header file for support functions */
#include "4_exercise.h"

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip revision alignment and errata fixes */
  CHIP_Init();

  /* Initialize peripherals */
  usart_setup();
  gpio_setup();
  /* The RTC is using LFXO */
  rtc_setup();

  /* Disable interrupts  */
  NVIC_DisableIRQ(RTC_IRQn);
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
#endif
  NVIC_DisableIRQ(USART1_TX_IRQn);


  /* Enable RTC interrupts - produces an interrupt every second */
  NVIC_EnableIRQ(RTC_IRQn);


  /* Enter the lowest possible energy mode here */



  /* Disable RTC interrupts */
  NVIC_DisableIRQ(RTC_IRQn);


  /* Enter the lowest possible energy mode here */


  /* Do some computations */
  uint32_t mult[10][10];
  for (uint32_t i = 0; i < 10; i++)
  {
    for (uint32_t j = 0; j < 10; j++)
    {
      mult[i][j] = (i + 1) * (j + 1);
    }
  }


  /* Enable USART interrupts */
  NVIC_EnableIRQ(USART1_TX_IRQn);

  /* Send a byte with the UART - produces an interrupt when done */
  USART1->TXDATA = mult[0][6] + '0';


  /* Enter the lowest possible energy mode here */



  /* Disable USART interrupts */
  NVIC_DisableIRQ(USART1_TX_IRQn);

  /* Enable GPIO interrupts - produced when Push Button 0 is pressed */
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif


  /* Enter the lowest possible energy mode here */



 /* Disable GPIO */
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
#endif

  /* Enter the lowest possible energy mode here */



  /* Stay in the loop forever */
  while (1) ;
}
