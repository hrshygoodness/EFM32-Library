/**************************************************************************//**
 * @file usart.c
 * @brief USART code for the EFM32 bootloader
 * @author Silicon Labs
 * @version 1.68
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
#include "config.h"

/***************************************************************************//**
 * @brief
 *   Prints an int in hex.
 *
 * @param integer
 *   The integer to be printed.
 ******************************************************************************/
__ramfunc void USART_printHex(uint32_t integer)
{
  uint8_t c;
  int i, digit;

  for (i = 0; i <= 7; i++)
  {
    digit = integer >> 28;
    c = digit + 0x30;
    if (digit >= 10)
    {
      c += 7;
    }
    USART_txByte(c);
    integer <<= 4;
  }
}

/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc uint8_t USART_rxByte(void)
{
  uint32_t timer = 1000000;
  while (!(BOOTLOADER_USART->STATUS & USART_STATUS_RXDATAV) && --timer ) ;
  if (timer > 0)
  {
    return((uint8_t)(BOOTLOADER_USART->RXDATA & 0xFF));
  }
  else
  {
    return 0;
  }
}


/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc void USART_txByte(uint8_t data)
{
  /* Check that transmit buffer is empty */
  while (!(BOOTLOADER_USART->STATUS & USART_STATUS_TXBL)) ;

  BOOTLOADER_USART->TXDATA = (uint32_t) data;
}

/**************************************************************************//**
 * @brief Transmit null-terminated string to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc void USART_printString(uint8_t *string)
{
  while (*string != 0)
  {
    USART_txByte(*string++);
  }
}

/**************************************************************************//**
 * @brief Intializes BOOTLOADER_USART
 *
 * @param clkdiv
 *   The clock divisor to use.
 *****************************************************************************/
void USART_init(uint32_t clkdiv)
{
  /* Configure BOOTLOADER_USART */
  /* USART default to 1 stop bit, no parity, 8 data bits, so not
   * explicitly set */

  /* Set the clock division */
  BOOTLOADER_USART->CLKDIV = clkdiv;


  /* Enable RX and TX pins and set location 0 */
  BOOTLOADER_USART->ROUTE = BOOTLOADER_USART_LOCATION |
                 USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

  /* Clear RX/TX buffers */
  BOOTLOADER_USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

  /* Enable RX/TX */
  BOOTLOADER_USART->CMD = USART_CMD_RXEN | USART_CMD_TXEN;
}
