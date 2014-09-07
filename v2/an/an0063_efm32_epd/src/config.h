/*****************************************************************************
 * @file config.h
 * @brief Configuration 
 * @author Silicon Labs
 * @version 1.02
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

/* Pin mappings. These can be changed, but note 
 * that the MOSI, CLK, and PWM must match the
 * configured USART and TIMER */
#define EPD_PIN_MOSI       gpioPortD,0
#define EPD_PIN_CLK        gpioPortD,2
#define EPD_PIN_CS         gpioPortC,5
#define EPD_PIN_RESET      gpioPortD,5
#define EPD_PIN_BORDER     gpioPortB,12
#define EPD_PIN_PWM        gpioPortD,3
#define EPD_PIN_BUSY       gpioPortC,6
#define EPD_PIN_PANEL_ON   gpioPortD,6
#define EPD_PIN_DISCHARGE  gpioPortD,7
#define EPD_PIN_PANEL_VDD  gpioPortC,4


/* This value must be sent over SPI before any register number */
#define REGISTER_HEADER 0x70

/* This value must be sent over SPI before any register data */
#define DATA_HEADER     0x72
 
/* 12 MHz maximum SPI frequency. If the HF clock is to slow
 * to achieve this frequency (2 * SPI_FREQ), the divisor
 * will be set to zero, making the SPI run as fast as possible
 * on the current clock. */
#define SPI_FREQ 12000000

/* Default is 2" panel. Change this line if using another display */
#define PANEL_200

#if defined(PANEL_144)
#define PANEL_WIDTH   128
#define PANEL_HEIGHT   96
#elif defined(PANEL_200)
#define PANEL_WIDTH   200
#define PANEL_HEIGHT   96
#elif defined(PANEL_270)
#define PANEL_WIDTH   264
#define PANEL_HEIGHT  176
#endif
   
/* Size of one local frame buffer (in bytes) */
#define FRAME_SIZE        (PANEL_HEIGHT * PANEL_WIDTH / 8)
   
/* Size of one frame buffer in EPD format (in bytes). Includes dummy bytes even though 
 * they are not needed for all displays. Actual size is in config struct. */
#define PANEL_FRAME_SIZE  (PANEL_HEIGHT * (PANEL_WIDTH / 4 + PANEL_HEIGHT / 4 + 1))

/* Size of one line in EPD format (in bytes). Includes dummy byte even though 
 * it is not needed for all displays. Actual size is in config struct. */   
#define PANEL_LINE_SIZE   (PANEL_WIDTH / 4 + PANEL_HEIGHT / 4 + 1)
   
/* This macro takes a value and rounds it up to the next value which
 * is divisible by 32. It is used to extend the emWin frame buffer
 * since the current version of the driver only supports displays
 * with dimensions that are divisble by 32. */
#define EMWIN_SIZE(x) (x % 32 == 0 ? x : ((x/32) + 1) * 32)
   
/* Size of the emWin frame buffer (in bytes) */
#define EMWIN_FRAME_SIZE ((EMWIN_SIZE(PANEL_HEIGHT) * EMWIN_SIZE(PANEL_WIDTH))/8)


#endif 
