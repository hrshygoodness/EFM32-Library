/***************************************************************************//**
 * @file usbconfig.h
 * @brief USB protocol stack library, application supplied configuration options.
 * @author Silicon Labs
 * @version 1.01
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __USBCONFIG_H
#define __USBCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB_DEVICE        /* Compile stack for device mode. */
  

#define DEBUG_USB_API
#define USB_USE_PRINTF

int RETARGET_WriteChar(char c);
#define USER_PUTCHAR  RETARGET_WriteChar

/* Use TIMER1 for usb, TIMER0 used for smart card clk. */
#define USB_TIMER USB_TIMER1

/****************************************************************************
**                                                                         **
** Specify number of endpoints used (in addition to EP0).                  **
**                                                                         **
*****************************************************************************/
#define NUM_EP_USED 3

#ifdef __cplusplus
}
#endif

#endif /* __USBCONFIG_H */
