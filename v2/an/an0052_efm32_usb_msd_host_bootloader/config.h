/**************************************************************************//**
 * @file config.h
 * @brief USB MSD Host Bootloader Configuration.
 *    This file defines how the bootloader is set up.
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
#ifndef CONFIG_H
#define CONFIG_H

/* Timeout in second for USB flash drive plug-in */
#define USB_WAIT_TIMEOUT      2

/* The size of the bootloader flash image */
#define BOOTLOADER_SIZE       (16*1024)         /* 16 KB */

/* The maximum flash size of any EFM32 part */
#define MAX_SIZE_OF_FLASH     (1024*1024)       /* 1 MB */

/* The size of a mass erase block */
#define MASSERASE_BLOCK_SIZE  (512*1024)        /* 512 KB */  

/* Buffer size for flash write */
#define BUFFERSIZE            (4*1024)          /* 4 KB */

/* Text file with binary file name */
#define TEXT_FILENAME         "binfname.txt"

/* Default binary file for firmware upgrade if default text file is not found */
#define BIN_FILENAME          "efmimage.bin"

/* Remove debug output function if NOUART is defined */
#ifdef NOUART
#define printf(...)
#endif

#endif
