/*****************************************************************************
 * @file spi.c
 * @brief SPI
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
#include "em_device.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "config.h"
#include "delay.h"
#include "spi.h"


/************************************************
 * Configures USART1 for SPI communication with 
 * the EPD COG driver
 ************************************************/
void spiInit(void)
{
  USART_InitSync_TypeDef spiInit = USART_INITSYNC_DEFAULT;
  
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* MOSI */
  GPIO_PinModeSet(EPD_PIN_MOSI, gpioModePushPull, 1);
  
  /* CS */
  GPIO_PinModeSet(EPD_PIN_CS, gpioModePushPull, 0);
  
  /* CLK */
  GPIO_PinModeSet(EPD_PIN_CLK, gpioModePushPull, 0);
  
  /* BUSY */
  GPIO_PinModeSet(EPD_PIN_BUSY, gpioModeInput, 0);
  
  /* Only enable TX output (no RX) */
  spiInit.enable = usartEnableTx;
  
  /* Set SPI speed */
  spiInit.baudrate = SPI_FREQ;
  
  /* Send most significant bit first */
  spiInit.msbf = true;
  
  /* Set USART1 registers */
  USART_InitSync(USART1, &spiInit);
  
  /* Route to location 1 and enable TX pin (PD0) */
  USART1->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;
}

/************************************************
 * Sends multiple bytes to the COG over SPI. 
 * The protocol requires that we first send the
 * register we write to followed by 1 or more
 * bytes of data. A register header must 
 * be sent before the register number and a data
 * header must be sent before the data. 
 * 
 * The protocol also requires that we wait 
 * on the busy pin between each byte while 
 * sending data. 
 * 
 * @param registerNum
 *   The COG register to write to
 *  
 * @param data
 *   Pointer to the data
 *   
 * @param dataLen
 *   Length of data in bytes
 ************************************************/
void spiSend(uint8_t registerNum, uint8_t *data, uint8_t dataLen)
{
  int i;

  /* Assert CS */
  GPIO_PinOutClear(EPD_PIN_CS);
  
  /* Send the register number */
  USART_Tx(USART1, REGISTER_HEADER);
  USART_Tx(USART1, registerNum);
  
  /* Deassert CS when transfer is complete */
  while ( !(USART1->STATUS & USART_STATUS_TXC) );
  GPIO_PinOutSet(EPD_PIN_CS);

  /* Wait 10 µs before sending data. Required by protocol */  
  delayUs(10);
  
  /* Assert CS */
  GPIO_PinOutClear(EPD_PIN_CS);
  
  /* Send data header */
  USART_Tx(USART1, DATA_HEADER);
  
  /* Send all data bytes */
  for ( i=0; i<dataLen; i++ )
  {
    /* COG will assert BUSY if it needs more time. 
     * wait until it is cleared. */
    while ( GPIO_PinInGet(EPD_PIN_BUSY) );
    
    /* Send data byte */
    USART_Tx(USART1, data[i]);
  }
  
  /* Deassert CS when transfer is complete */
  while ( !(USART1->STATUS & USART_STATUS_TXC) );
  GPIO_PinOutSet(EPD_PIN_CS);
}

/************************************************
 * Helper function to send 1 byte of data. 
 * Will call spiSend(). 
 * 
 * @param registerNum
 *   COG register to write to
 *  
 * @param data
 *   Data byte to write
 ************************************************/
void spiSend1(uint8_t registerNum, uint8_t data)
{
  spiSend(registerNum, &data, 1);
}


/************************************************
 * Helper function to send 2 bytes of data. 
 * Will call spiSend(). Bits are sent MSB first
 * as they are written by the calling function. 
 * 
 * @param registerNum
 *   COG register to write to
 *  
 * @param data
 *   Data byte to write
 ************************************************/
void spiSend2(uint8_t registerNum, uint16_t data)
{
  uint8_t data8[2];
  
  data8[0] = ((uint8_t *)&data)[1];
  data8[1] = ((uint8_t *)&data)[0];
  
  spiSend(registerNum, data8, 2);
}
