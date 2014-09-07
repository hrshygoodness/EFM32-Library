/***************************************************************************//**
 * @file usbconfig.h
 * @brief USB protocol stack library, application supplied configuration options.
 * @author Silicon Labs
 * @version 1.02
 *******************************************************************************
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
#ifndef __USBCONFIG_H
#define __USBCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

  
/* Compile stack for host mode. */
#define USB_HOST     

/* Number of host channels used in addition to 0 and 1 
 * which are used for the control endpoint EP0 */  
#define NUM_HC_USED 2
   
/* Select TIMER0 to be used by the USB stack. This timer
 * must not be used by the application. */
#define USB_TIMER USB_TIMER0   
  

  
/********************************************************** 
 * Define the endpoint addresses. Bits [3:0] define
 * endpoint number. Bit 7 defines direction (1 = IN). 
 **********************************************************/
  
/* Endpoint for USB data IN  (device to host).    */  
#define EP_IN             0x81  
  
/* Endpoint for USB data OUT (host to device).    */
#define EP_OUT            0x01  


/********************************************************** 
 * VBUS overcurrent configuration
 **********************************************************/   
   
#ifdef STK
  #define USB_VBUSOVRCUR_PORT       gpioPortF   
  #define USB_VBUSOVRCUR_PIN        6               
  #define USB_VBUSOVRCUR_POLARITY   USB_VBUSOVRCUR_POLARITY_LOW 
#else /* DK */
  #define USB_VBUSOVRCUR_PORT       gpioPortE       
  #define USB_VBUSOVRCUR_PIN        2               
  #define USB_VBUSOVRCUR_POLARITY   USB_VBUSOVRCUR_POLARITY_LOW   
#endif
  

  
/********************************************************** 
 * Debug Configuration. Enable the stack to output
 * debug messages to a console. This example is
 * configured to output messages over UART.
 **********************************************************/

/* Enable debug output from the stack */
#define DEBUG_USB_API

/* Enable printf calls in stack */
#define USB_USE_PRINTF   

/* Function declaration for the low-level printing of a 
 * character. This function must be implemented by the 
 * application. */
int RETARGET_WriteChar(char c);
#define USER_PUTCHAR  RETARGET_WriteChar      


/********************************************************** 
 * Power saving configuration. Select low frequency 
 * clock and power saving mode.
 **********************************************************/

/* Select the clock used when USB is in low power mode */
#define USB_USBC_32kHz_CLK   USB_USBC_32kHz_CLK_LFXO

   
   
   
/* The address to give the attached device */
#define DEVICE_ADDR 1

/* Size of read buffer in bytes */
#define READ_BUFFER_SIZE 500
   

#ifdef __cplusplus
}
#endif

#endif /* __USBCONFIG_H */

