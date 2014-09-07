/**************************************************************************//**
 * @file uart.c
 * @brief Booatloader UART communication
 * @author Silicon Labs
 * @version 1.03
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
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "uart.h"
#include "bootloader_config.h"



/******************************************************************************
 * Enables UART for the bootloader.
 *****************************************************************************/
void BLUART_init(void)
{
  /* Enable the required clocks */
  CMU_ClockEnable(UART_CLOCK, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure GPIO */
  GPIO_PinModeSet(TXPORT, TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(RXPORT, RXPIN, gpioModeInput, 0);
                  
  /* Configure USART peripheral. Default configuration 
   * is fine. We only need to set the baud rate.  */
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
  uartInit.baudrate = BOOTLOADER_BAUDRATE;
  USART_InitAsync(UART, &uartInit);
  
  /* Enable RX and TX and set location */
  UART->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | UART_LOC;
}

/******************************************************************************
 * Receives one byte from UART. This function will stall until
 * a byte is available. 
 *****************************************************************************/
RAMFUNC  uint8_t BLUART_receive(void)
{
  while ( !(UART->STATUS & USART_STATUS_RXDATAV) );
  return UART->RXDATA;
}

/******************************************************************************
 * Send one byte over UART.
 *****************************************************************************/
RAMFUNC  void BLUART_send(uint8_t data)
{
  while ( !(UART->STATUS & USART_STATUS_TXBL) );
  UART->TXDATA = data;
}

/******************************************************************************
 * Send a string over UART.
 *****************************************************************************/
RAMFUNC void BLUART_sendString(char *str)
{            
  while (1) {
    if ( *str == 0 ) 
      break;
    BLUART_send((uint8_t)*str++);
  }
  
  while ( !(UART->STATUS & USART_STATUS_TXC) );
}




