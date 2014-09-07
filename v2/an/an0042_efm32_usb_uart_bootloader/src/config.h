/**************************************************************************//**
 * @file config.h
 * @brief Bootloader Configuration.
 *    This file defines how the bootloader is set up.
 * @author Silicon Labs
 * @version 1.10
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
#ifndef CONFIG_H
#define CONFIG_H

/************ DEBUG #define's ******************************************/
/** This #define pulls in printf etc. Undef when making release build. */
//#define BL_DEBUG

/** This #define simulates that the SWDCLK pin is pulled high.
 ** Undef when making release build.                                   */
//#define SIMULATE_SWDCLK_PIN_HI
/************ DEBUG #define's end **************************************/


/** Define USB endpoint addresses */
#define EP_DATA_OUT  0x01       /* Endpoint for USB data reception.       */
#define EP_DATA_IN   0x81       /* Endpoint for USB data transmission.    */
#define EP_NOTIFY    0x82       /* The notification endpoint (not used).  */

/** Number of milliseconds between each consecutive polling of the SWD pins */
#define PIN_LOOP_INTERVAL    250

/** USART used for communication. */
#define BOOTLOADER_USART          USART0
#define BOOTLOADER_USART_CLOCK    CMU_HFPERCLKEN0_USART0
#define BOOTLOADER_USART_LOCATION USART_ROUTE_LOCATION_LOC0

/** TIMER1 is used for autobaud. The channel and location must match the
 ** RX line of BOOTLOADER_USART for this to work properly. */
#define AUTOBAUD_TIMER            TIMER1
#define AUTOBAUD_TIMER_CHANNEL    1
#define AUTOBAUD_TIMER_LOCATION   TIMER_ROUTE_LOCATION_LOC1
#define AUTOBAUD_TIMER_IRQn       TIMER1_IRQn
#define AUTOBAUD_TIMER_CLOCK      CMU_HFPERCLKEN0_TIMER1
#define AUTOBAUD_TIMER_INT_MASK   TIMER_IFC_CC1
#define AUTOBAUD_TIMER_ROUTE      TIMER_ROUTE_CC1PEN
#define AUTOBAUD_TIMER_IRQHANDLER TIMER1_IRQHandler

#define SWDCLK_PIN_IS_HI()        ( ( GPIO->P[5].DIN & 0x1 ) == 0x1 )
#define SWDCLK_PIN_IS_LO()        ( ( GPIO->P[5].DIN & 0x1 ) == 0x0 )

/** The size of the bootloader flash image */
#define BOOTLOADER_SIZE      (16*1024)            /* 16 KB */

/** The maximum flash size of any EFM32 part */
#define MAX_SIZE_OF_FLASH         (1024*1024)     /* 1 MB */

/** The size of a mass erase block */
#define MASSERASE_BLOCK_SIZE      (512*1024)      /* 512 KB */

/** This function sets up GPIO for the USART used in the bootloader. */
__STATIC_INLINE void CONFIG_UsartGpioSetup(void)
{
  /* Use USART0 location 0
   * 0 : TX - Pin E10, RX - Pin E11
   * Configure GPIO pins LOCATION 1 as push pull (TX)
   * and input (RX)
   * To avoid false start, configure output as high
   */
  GPIO->P[4].DOUT = (1 << 10);
  GPIO->P[4].MODEH = GPIO_P_MODEH_MODE10_PUSHPULL | GPIO_P_MODEH_MODE11_INPUT;
}

#endif
