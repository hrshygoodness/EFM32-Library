/**************************************************************************//**
 * @file bootloader_config.h
 * @brief Configuration file for the AES Bootloader
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
#ifndef _BOOTLOADER_CONFIG_H
#define _BOOTLOADER_CONFIG_H


#include "em_device.h"

/************************************************
 * Static configuration. Do not modify
 ***********************************************/


/* Firmware Status */
#define FW_NOT_VERIFIED    0xffffffff
#define FW_VERIFIED        0x55555555
#define FW_DELETED         0x00000000

#define WORDS_PER_PAGE (FLASH_PAGE_SIZE / 4)

#define AES_BLOCKSIZE 16
#define AES_KEYSIZE 32

/* The space reserved for meta information before
 * each firmware image. The vector table will be placed
 * at FIRMWARE_START_ADDRESS + FIRMWARE_HEADER_SIZE. */
#define FIRMWARE_HEADER_SIZE 0x100

/* Address of Debug Lock Word. See reference manual */
#define DEBUG_LOCK_WORD    (0x0FE04000 + (127 * 4))



/**************************************************
 * Bootloader configuration. Modify this to fit
 * the device and application.
 *************************************************/

/* Configure UART peripheral */
#define UART USART1
#define UART_CLOCK cmuClock_USART1

/* Configure pins and location for UART. */
#define RXPORT gpioPortD
#define RXPIN 1
#define TXPORT gpioPortD
#define TXPIN 0

#define UART_LOC USART_ROUTE_LOCATION_LOC1

/* The baudrate to use for bootloader */
#define BOOTLOADER_BAUDRATE 115200

/* Which pin to pull high to enter 'bootloader mode' */
#define BOOTLOADER_PIN gpioPortD, 8

/**************************************************
 * Firmware configuration. Modify this section
 * to change location and size of the firmware
 * location(s). 
 *************************************************/

/* Set to true if you want to use a temporary storage when
 * uploading a new firmware image. If set to false the 
 * old firmware will be overwritten directly */
#define USE_TEMP_STORAGE false



/* Where to place the new firmware image. The boot address will 
 * be at FIRMWARE_START_ADDRESS + FIRMWARE_HEADER_SIZE */
#define FIRMWARE_START_ADDRESS 0x00002800
#define FIRMWARE_END_ADDRESS   FLASH_SIZE - 1

/* Where to to put the temporary storage used when 
 * uploading a new firmware image */
#define TEMP_START_ADDRESS 0x00064000
#define TEMP_END_ADDRESS   0x000c3fff



/* Firmware header struct. Gives information
 * about the firmware image */
typedef struct _FirmwareHeader 
{    
  /* Size in bytes of the binary image */
  uint32_t size;
  
  /* 128 bit hash */
  uint8_t hash[AES_BLOCKSIZE];
  
  /* Verified field. Will be set to one of the Firmware Status values defined above. */
  uint32_t verified;
  
} FirmwareHeader;

#endif
