/**************************************************************************//**
 * @file usart.c
 * @brief USART example
 * @author Silicon Labs
 * @version 1.14
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "em_device.h"
#include "usart.h"
#include "em_gpio.h"

/******************************************************************************
 * @brief sends data using USART2
 * @param txBuffer points to data to transmit
 * @param bytesToSend bytes will be sent
 *****************************************************************************/
void USART2_sendBuffer(char* txBuffer, int bytesToSend)
{
  USART_TypeDef *uart = USART2;
  int           ii;

  /* Sending the data */
  for (ii = 0; ii < bytesToSend;  ii++)
  {
    /* Waiting for the usart to be ready */
    while (!(uart->STATUS & USART_STATUS_TXBL)) ;

    if (txBuffer != 0)
    {
      /* Writing next byte to USART */
      uart->TXDATA = *txBuffer;
      txBuffer++;
    }
    else
    {
      uart->TXDATA = 0;
    }
  }

  /*Waiting for transmission of last byte */
  while (!(uart->STATUS & USART_STATUS_TXC)) ;
}



