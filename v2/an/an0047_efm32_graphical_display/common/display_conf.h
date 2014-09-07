/***************************************************************************//**
 * @file display_conf.h
 * @brief Display config
 * @author Silicon Labs
 * @version 1.06
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
#ifndef DISPLAY_CONF_H
#define DIPSLAY_CONF_H

/* Physical display size */
#define LCD_WIDTH   320
#define LCD_HEIGHT  240

/* 16-bit color per pixel */
#define BYTES_PER_PIXEL 2

/* The size in bytes of a full frame buffer */
#define FRAME_BUFFER_SIZE (LCD_WIDTH * LCD_HEIGHT * BYTES_PER_PIXEL)

/* Start address of video memory points to beginning of PSRAM block */
#define VRAM_ADDR_START  EBI_BankAddress(EBI_BANK2)


#endif
