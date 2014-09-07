/**************************************************************************//**
 * @file config.h
 * @brief Bootloader Configuration.
 *    This file defines how the bootloader is set up.
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
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "em_device.h"

/** Frequency of the LF clock */
#define LFRCO_FREQ           (32768)

/** Number of seconds before autobaud times out and restarts the bootloader */
#define AUTOBAUD_TIMEOUT     30

/** Number of milliseconds between each consecutive polling of the SWD pins */
#define PIN_LOOP_INTERVAL    250

/** The size of the bootloader flash image */
#define BOOTLOADER_SIZE      0x800

/** USART used for communication. */
#define BOOTLOADER_USART           LEUART0
#define BOOTLOADER_USART_CLOCK     0
#define BOOTLOADER_LEUART_CLOCK    CMU_LFBCLKEN0_LEUART0
#define BOOTLOADER_USART_LOCATION  LEUART_ROUTE_LOCATION_LOC3

/** TIMER1 is used for autobaud. The channel and location must match the
 * RX line of BOOTLOADER_USART for this to work properly. */
#define AUTOBAUD_TIMER             TIMER0
#define AUTOBAUD_TIMER_CHANNEL     1
#define AUTOBAUD_TIMER_LOCATION    TIMER_ROUTE_LOCATION_LOC5
#define AUTOBAUD_TIMER_IRQn        TIMER0_IRQn
#define AUTOBAUD_TIMER_CLOCK       CMU_HFPERCLKEN0_TIMER0

/** USART used for debugging. */
#define DEBUG_USART                USART1
#define DEBUG_USART_CLOCK          CMU_HFPERCLKEN0_USART1
#define DEBUG_USART_LOCATION       USART_ROUTE_LOCATION_LOC2

/** Check for falling edge. Use if UART overlaps with GPIO */
#define USART_OVERLAPS_WITH_BOOTLOADER

/** This function sets up the GPIO setting for the debug output. */
static __INLINE void CONFIG_DebugGpioSetup(void)
{
  /* Use USART1 location 2/3
   * 0 : TX - Pin D7, RX - Pin D6

   * Configure GPIO pin (TX) as push pull (TX) and (RX) as input
   * To avoid false start, set output level to high
   */
  GPIO->P[3].DOUT = (1 << 7);
  GPIO->P[3].MODEL = GPIO_P_MODEL_MODE7_PUSHPULL | GPIO_P_MODEL_MODE6_INPUT;
}

/** This function sets up GPIO for the USART used in the bootloader. */
static __INLINE void CONFIG_UsartGpioSetup(void)
{
  /*
   * LEUART0, location 3:
   * TX: F0, RX: F1
   */
  GPIO->P[5].MODEL = GPIO_P_MODEL_MODE0_PUSHPULL | GPIO_P_MODEL_MODE1_INPUT;
}

#endif
