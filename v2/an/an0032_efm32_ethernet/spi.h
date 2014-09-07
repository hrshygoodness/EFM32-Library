/***************************************************************************//**
 * @file spi.h
 * @brief SPI functions declaration.
 * @author Silicon Labs
 * @version 1.09
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
#ifndef __SPI_H
#define __SPI_H

#include <stdbool.h>
#include "em_device.h"
#include "em_bitband.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup EFM32_Library
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup GPIO
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/* USART used for SPI access */
#define USART_USED        USART1
#define USART_CLK         cmuClock_USART1

/* GPIO pins used for SPI communication. */
#define PIN_SPI_TX        0
#define PORT_SPI_TX       gpioPortD
#define PIN_SPI_RX        1
#define PORT_SPI_RX       gpioPortD
#define PIN_SPI_CLK       2
#define PORT_SPI_CLK      gpioPortD
#define PIN_SPI_CS        3
#define PORT_SPI_CS       gpioPortD
#define PIN_SPI_RESET     4
#define PORT_SPI_RESET    gpioPortD


/* USART read/write cycles */
#define USART_READ     1
#define USART_WRITE    0

#define CS_LOW()        GPIO->P[3].DOUT &= ~(1 << 3);
#define CS_HIGH()       GPIO->P[3].DOUT |= (1 << 3);

#define MOSI_LOW()      GPIO->P[3].DOUT &= ~(1 << 0);
#define MOSI_HIGH()     GPIO->P[3].DOUT |= (1 << 0);

#define SCK_LOW()       GPIO->P[3].DOUT &= ~(1 << 2);
#define SCK_HIGH()      GPIO->P[3].DOUT |= (1 << 2);

#define RESET_LOW()     GPIO->P[3].DOUT &= ~(1 << 4);
#define RESET_HIGH()    GPIO->P[3].DOUT |= (1 << 4);

/* #define CS_HIGH()                GPIO->P[3].DOUT &= ~(1 << 3); */
/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void spiInit(void);
void spiDisable(void);
uint16_t spiAccess(uint8_t spiaddr, uint8_t spidata);
void spiSendByte(uint8_t spidata);
uint8_t spiGetByte(void);

/** @} (end addtogroup GPIO) */
/** @} (end addtogroup EFM32_Library) */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
