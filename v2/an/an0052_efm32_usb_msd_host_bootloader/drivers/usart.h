/**************************************************************************//**
 * @file usart.h
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
#ifndef __USART_H
#define __USART_H

#include <stdint.h>
#include "config.h"

#ifndef GG_STK
#define USART_BAUD      0x1900
#define USART_USED      UART1
#define USART_CLK       CMU_HFPERCLKEN0_UART1
#define USART_PORTNUM   1
#define USART_TXPORT    gpioPortB
#define USART_TXPIN     9
#define USART_LOCATION  USART_ROUTE_LOCATION_LOC2
#else
#define USART_BAUD      0x1900
#define USART_USED      USART0
#define USART_CLK       CMU_HFPERCLKEN0_USART0
#define USART_PORTNUM   2
#define USART_TXPORT    gpioPortC
#define USART_TXPIN     0
#define USART_LOCATION  USART_ROUTE_LOCATION_LOC5
#endif

#endif
