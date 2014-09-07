/**************************************************************************//**
 * @file spi.c
 * @brief SPI implementation of Board Control interface
 *        This implementation use the USART1 SPI interface to control board
 *        control registers.
 * @author Silicon Labs
 * @version 1.09
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

#include <stdio.h>
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "spi.h"

/**************************************************************************//**
 * @brief  Initializes SPI interface for access the U-Force transceiver board.
 *****************************************************************************/
void spiInit(void)
{
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

  /* Enable module clocks */
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* To avoid false start, configure output US1_TX as high on PD0 */
  /* GPIO pins used, please refer to DVK user guide. */
  /* Configure SPI pins */
  GPIO_PinModeSet(PORT_SPI_TX, PIN_SPI_TX, gpioModePushPull, 1);
  GPIO_PinModeSet(PORT_SPI_RX, PIN_SPI_RX, gpioModeInput, 0);
  GPIO_PinModeSet(PORT_SPI_CLK, PIN_SPI_CLK, gpioModePushPull, 0);
  /* Keep CS high to not activate slave */
  GPIO_PinModeSet(PORT_SPI_CS, PIN_SPI_CS, gpioModePushPull, 1);

  /* Reset USART just in case */
  USART_Reset(USART_USED);

  /* Enable clock for USART1 */
  CMU_ClockEnable(USART_CLK, true);

  /* Configure to use SPI master with manual CS */
  /* For now, configure SPI for worst case 32MHz clock in order to work for all */
  /* configurations. */
  init.refFreq  = 16000000;
  init.baudrate = 2000000;
  init.msbf     = true;
  USART_InitSync(USART_USED, &init);  
  
  /* Module USART1 is configured to location 1 */
  USART_USED->ROUTE = (USART_USED->ROUTE & ~_USART_ROUTE_LOCATION_MASK) |
                      USART_ROUTE_LOCATION_LOC1;

  /* Enable signals TX, RX, CLK, CS */
  USART_USED->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN |
                       USART_ROUTE_CLKPEN;

  USART_USED->CTRL |= USART_CTRL_CLKPOL;
  USART_USED->CTRL |= USART_CTRL_CLKPHA;

  /*Reset*/
  GPIO_PinModeSet(PORT_SPI_RESET, PIN_SPI_RESET, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  Disables GPIO pins and USART from FPGA register access
 *****************************************************************************/
void spiDisable(void)
{
  USART_Reset(USART_USED);

  /* Route setup must be reset separately */
  USART_USED->ROUTE = _USART_ROUTE_RESETVALUE;

  /* Disable SPI pins */
  GPIO_PinModeSet(PORT_SPI_TX, PIN_SPI_TX, gpioModeDisabled, 0);
  GPIO_PinModeSet(PORT_SPI_RX, PIN_SPI_RX, gpioModeDisabled, 0);
  GPIO_PinModeSet(PORT_SPI_CLK, PIN_SPI_CLK, gpioModeDisabled, 0);
  GPIO_PinModeSet(PORT_SPI_CS, PIN_SPI_CS, gpioModeDisabled, 0);

  /* Disable USART clock - we can't disable GPIO or HFPER as we don't know who else
   * might be using it */
  CMU_ClockEnable(USART_CLK, false);
}

/**************************************************************************//**
 * @brief  Perform SPI Transfer
 *****************************************************************************/
uint16_t spiAccess(uint8_t spiaddr, uint8_t spidata)
{
  uint16_t readData;

  /* Write SPI address */
  USART_Tx(USART_USED, spiaddr);

  /* Wait for transmission to finish */
  while (!(USART_USED->STATUS & USART_STATUS_TXC)) ;

  /* Just ignore data read back */
  readData   = USART_Rx(USART_USED);
  readData <<= 8;

  /* Write SPI data */
  USART_Tx(USART_USED, spidata);

  /* Wait for transmission to finished */
  while (!(USART_USED->STATUS & USART_STATUS_TXC)) ;

  readData |= USART_Rx(USART_USED);

  return(readData);
}

/**************************************************************************//**
 * @brief  Perform SPI Transfer
 *****************************************************************************/
void spiSendByte(uint8_t spidata)
{
  /* Write SPI address */
  USART_Tx(USART_USED, spidata);

  /* Wait for transmission to finish */
  while (!(USART_USED->STATUS & USART_STATUS_TXC)) ;

  /* Just ignore data read back */
  USART_Rx(USART_USED);
}

/**************************************************************************//**
 * @brief  Perform SPI Transfer
 *****************************************************************************/
uint8_t spiGetByte(void)
{
  /* Write SPI address */
  USART_Tx(USART_USED, 0x00);

  /* Wait for transmission to finish */
  while (!(USART_USED->STATUS & USART_STATUS_TXC)) ;

  /* Return the data read from SPI buffer */
  return(USART_Rx(USART_USED));
}
