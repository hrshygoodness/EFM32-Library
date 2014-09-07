/**************************************************************************//**
 * @file spi.c
 * @brief SPI
 * @author Silicon Labs
 * @version 1.14
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
#include "spi.h"
#include "em_gpio.h"
#include "spi_project.h"


/* Buffer pointers and indexes */
char* slaveTxBuffer;
int slaveTxBufferSize;
volatile int slaveTxBufferIndex;
char* slaveRxBuffer;
int slaveRxBufferSize;
volatile int slaveRxBufferIndex;
char* masterRxBuffer;
int masterRxBufferSize;
volatile int masterRxBufferIndex;



/**************************************************************************//**
 * @brief Setup a USART as SPI
 * @param spiNumber is the number of the USART to use (e.g. 1 USART1)
 * @param location is the GPIO location to use for the device
 * @param master set if the SPI is to be master
 *****************************************************************************/
void SPI_setup(uint8_t spiNumber, uint8_t location, bool master)
{
  USART_TypeDef *spi;

  /* Determining USART */
  switch (spiNumber)
  {
  case 0:
    spi = USART0;
    break;
  case 1:
    spi = USART1;
    break;
  case 2:
    spi = USART2;
    break;
  default:
    return;
  }

  /* Setting baudrate */
  spi->CLKDIV = 128 * (SPI_PERCLK_FREQUENCY / SPI_BAUDRATE - 2);

  /* Configure SPI */
  /* Using synchronous (SPI) mode*/
  spi->CTRL = USART_CTRL_SYNC;
  /* Clearing old transfers/receptions, and disabling interrupts */
  spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  spi->IEN = 0;
  /* Enabling pins and setting location */
  spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN | (location << 8);
  
  /* Set GPIO config to slave */
  GPIO_Mode_TypeDef gpioModeMosi = gpioModeInput;
  GPIO_Mode_TypeDef gpioModeMiso = gpioModePushPull;
  GPIO_Mode_TypeDef gpioModeCs   = gpioModeInput;
  GPIO_Mode_TypeDef gpioModeClk  = gpioModeInput;
  
  /* Set to master and to control the CS line */
  if (master)
  {
    /* Enabling Master, TX and RX */
    spi->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
    spi->CTRL |= USART_CTRL_AUTOCS;
    
    /* Set GPIO config to master */
    gpioModeMosi = gpioModePushPull;
    gpioModeMiso = gpioModeInput;
    gpioModeCs   = gpioModePushPull;
    gpioModeClk  = gpioModePushPull;
  }
  else
  {
    /* Enabling TX and RX */
    spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  }

  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration */
  switch(spiNumber)
  {
    case 0: switch(location)
            {
              case 0: /* IO configuration (USART 0, Location #0) */
                      GPIO_PinModeSet(gpioPortE, 10, gpioModeMosi, 0); /* MOSI */
                      GPIO_PinModeSet(gpioPortE, 11, gpioModeMiso, 0); /* MISO */
                      GPIO_PinModeSet(gpioPortE, 13, gpioModeCs,   0); /* CS */
                      GPIO_PinModeSet(gpioPortE, 12, gpioModeClk,  0); /* Clock */
                      break;
              case 1: /* IO configuration (USART 0, Location #1) */
                      GPIO_PinModeSet(gpioPortE, 7, gpioModeMosi, 0);  /* MOSI */ 
                      GPIO_PinModeSet(gpioPortE, 6, gpioModeMiso, 0);  /* MISO */
                      GPIO_PinModeSet(gpioPortE, 4, gpioModeCs,   0);  /* CS */
                      GPIO_PinModeSet(gpioPortE, 5, gpioModeClk,  0);  /* Clock */
                      break;
              case 2: /* IO configuration (USART 0, Location #2) */
                      GPIO_PinModeSet(gpioPortC, 11, gpioModeMosi, 0); /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 10, gpioModeMiso, 0); /* MISO */
                      GPIO_PinModeSet(gpioPortC,  8, gpioModeCs,   0); /* CS */
                      GPIO_PinModeSet(gpioPortC,  9, gpioModeClk,  0); /* Clock */
                      break;
            default: break;
            }
            break;
    case 1: switch(location)
            {
              case 0: /* IO configuration (USART 1, Location #0) */
                      GPIO_PinModeSet(gpioPortC, 0, gpioModeMosi, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 1, gpioModeMiso, 0);  /* MISO */
                      GPIO_PinModeSet(gpioPortB, 8, gpioModeCs,   0);  /* CS */
                      GPIO_PinModeSet(gpioPortB, 7, gpioModeClk,  0);  /* Clock */
                      break;
              case 1: /* IO configuration (USART 1, Location #1) */
                      GPIO_PinModeSet(gpioPortD, 0, gpioModeMosi, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortD, 1, gpioModeMiso, 0);  /* MISO */
                      GPIO_PinModeSet(gpioPortD, 3, gpioModeCs,   0);  /* CS */
                      GPIO_PinModeSet(gpioPortD, 2, gpioModeClk,  0);  /* Clock */
                      break;              
            default: break;
            }
            break;
    case 2: switch(location)
            {
              case 0: /* IO configuration (USART 2, Location #0) */
                      GPIO_PinModeSet(gpioPortC, 2, gpioModeMosi, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortC, 3, gpioModeMiso, 0);  /* MISO */
                      GPIO_PinModeSet(gpioPortC, 5, gpioModeCs,   0);  /* CS */
                      GPIO_PinModeSet(gpioPortC, 4, gpioModeClk,  0);  /* Clock */
                      break;
              case 1: /* IO configuration (USART 2, Location #1) */
                      GPIO_PinModeSet(gpioPortB, 3, gpioModeMosi, 0);  /* MOSI */
                      GPIO_PinModeSet(gpioPortB, 4, gpioModeMiso, 0);  /* MISO */
                      GPIO_PinModeSet(gpioPortB, 6, gpioModeCs,   0);  /* CS */
                      GPIO_PinModeSet(gpioPortB, 5, gpioModeClk,  0);  /* Clock */
                      break;              
            default: break;
            }
            break;
    default: break;  
  }
}



/**************************************************************************//**
 * @brief USART1 RX IRQ Handler Setup
 * @param receiveBuffer points to where to place recieved data
 * @param receiveBufferSize indicates the number of bytes to receive
 *****************************************************************************/
void SPI1_setupRXInt(char* receiveBuffer, int receiveBufferSize)
{
  USART_TypeDef *spi = USART1;

  /* Setting up pointer and indexes */
  slaveRxBuffer      = receiveBuffer;
  slaveRxBufferSize  = receiveBufferSize;
  slaveRxBufferIndex = 0;

  /* Clear RX */
  spi->CMD = USART_CMD_CLEARRX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
  spi->IEN |= USART_IEN_RXDATAV;
}



/**************************************************************************//**
 * @brief USART1 TX IRQ Handler Setup
 * @param transmitBuffer points to the data to send
 * @param transmitBufferSize indicates the number of bytes to send
 *****************************************************************************/
void SPI1_setupTXInt(char* transmitBuffer, int transmitBufferSize)
{
  USART_TypeDef *spi = USART1;

  /* Setting up pointer and indexes */
  slaveTxBuffer      = transmitBuffer;
  slaveTxBufferSize  = transmitBufferSize;
  slaveTxBufferIndex = 0;

  /* Clear TX */
  spi->CMD = USART_CMD_CLEARTX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);
  spi->IEN |= USART_IEN_TXBL;
}



/**************************************************************************//**
 * @brief Setting up RX interrupts from USART2 RX
 * @param receiveBuffer points to where received data is to be stored
 * @param bytesToReceive indicates the number of bytes to receive
 *****************************************************************************/
void SPI2_setupRXInt(char* receiveBuffer, int bytesToReceive)
{
  USART_TypeDef *spi = USART2;

  /* Setting up pointer and indexes */
  masterRxBuffer      = receiveBuffer;
  masterRxBufferSize  = bytesToReceive;
  masterRxBufferIndex = 0;

  /* Flushing rx */
  spi->CMD = USART_CMD_CLEARRX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART2_RX_IRQn);
  NVIC_EnableIRQ(USART2_RX_IRQn);
  spi->IEN = USART_IEN_RXDATAV;
}



/**************************************************************************//**
 * @brief USART1 IRQ Handler Setup
 * @param receiveBuffer points to where received data is to be stored
 * @param receiveBufferSize indicates the number of bytes to receive
 * @param transmitBuffer points to the data to send
 * @param transmitBufferSize indicates the number of bytes to send
 *****************************************************************************/
void SPI1_setupSlaveInt(char* receiveBuffer, int receiveBufferSize, char* transmitBuffer, int transmitBufferSize)
{
  SPI1_setupRXInt(receiveBuffer, receiveBufferSize);
  SPI1_setupTXInt(transmitBuffer, transmitBufferSize);
}



/**************************************************************************//**
 * @brief USART1 RX IRQ Handler
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  USART_TypeDef *spi = USART1;
  uint8_t       rxdata;

  if (spi->STATUS & USART_STATUS_RXDATAV)
  {
    /* Reading out data */
    rxdata = spi->RXDATA;

    if (slaveRxBufferIndex < slaveRxBufferSize)
    {
      /* Store Data */
      slaveRxBuffer[slaveRxBufferIndex] = rxdata;
      slaveRxBufferIndex++;
    }
  }
}



/**************************************************************************//**
 * @brief USART1 TX IRQ Handler
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  USART_TypeDef *spi = USART1;

  if (spi->STATUS & USART_STATUS_TXBL)
  {
    /* Checking that valid data is to be transferred */
    if (slaveTxBuffer != 0)
    {
      /* Writing new data */
      spi->TXDATA = slaveTxBuffer[slaveTxBufferIndex];
      slaveTxBufferIndex++;
      /*Checking if more data is to be transferred */
      if (slaveTxBufferIndex == slaveTxBufferSize)
      {
        slaveTxBuffer = 0;
      }
    }
    else
    {
      /* Sending 0 if no data to send */
      spi->TXDATA = 0;
    }
  }
}



/**************************************************************************//**
 * @brief USART2 RX IRQ Handler
 *****************************************************************************/
void USART2_RX_IRQHandler(void)
{
  USART_TypeDef *spi = USART2;
  uint8_t       rxdata;

  if (spi->STATUS & USART_STATUS_RXDATAV)
  {
    /* Reading out data */
    rxdata = spi->RXDATA;

    if (masterRxBuffer != 0)
    {
      /* Store Data */
      masterRxBuffer[masterRxBufferIndex] = rxdata;
      masterRxBufferIndex++;

      if (masterRxBufferIndex == masterRxBufferSize)
      {
        masterRxBuffer = 0;
      }
    }
  }
}


