/**************************************************************************//**
 * @file usart.c
 * @brief USART for bootloader debugging
 * @author Silicon Labs
 * @version 1.04
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

/**************************************************************************//**
 * @brief Transmit buffer to USART
 * @param Data character to transmit
 * @return Character sent
 *****************************************************************************/
int RETARGET_WriteChar(char c)
{
  /* Add CR if \n */
  if (c == '\n')
  {
    while (!(USART_USED->STATUS & USART_STATUS_TXBL))
      ;
    USART_USED->TXDATA = '\r';
  }
  while (!(USART_USED->STATUS & USART_STATUS_TXBL))
    ;
  USART_USED->TXDATA = c;

  return c;
}

/**************************************************************************//**
 * @brief Dummy function for retargetio.c
 *****************************************************************************/
int RETARGET_ReadChar(void)
{
  int i = 0;

  return i;
}
