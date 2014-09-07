/**************************************************************************//**
 * @file
 * @brief SPI Interface for Ethernet controller; Micrel KSZ8851SNL
 * @version 3.20.5
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#include "ethspi.h"
#include "em_device.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include <stdio.h>

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
static void ETHSPI_SetChipSelect(bool value);
static uint8_t ETHSPI_XferSpi(uint8_t data);

/** Bytes to SPI packet byte enable bits conversion table */
static const uint8_t bitEnable[] = { 0x00, 0x1, 0x3, 0x7, 0xf };

/** SPI init structure */
static const USART_InitSync_TypeDef initSpi =
{ usartEnable,    /* Enable RX/TX when init completed. */
  48000000,       /* Use 48MHz reference clock */
  7000000,        /* 7 Mbits/s. */
  usartDatabits8, /* 8 databits. */
  true,           /* Master mode. */
  true,           /* Send most significant bit first. */
  usartClockMode0,
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_TINY_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  false,
  usartPrsRxCh0,
  false,
#endif
};
/** @endcond */

/**************************************************************************//**
 * @brief ETHSPI_Init
 *    Initialize SPI interface to Ethernet controller.
 * @note To enable access, be sure to call the functions
 *            BSP_PeripheralAccess(BSP_ETH, enable);
 *    before using this interface.
 *****************************************************************************/
void ETHSPI_Init(void)
{
  USART_TypeDef *spi;

  /* Enabling clock to USART */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(ETH_USART_CLK, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup SPI at USART0 */
  spi = ETH_USART_USED;
  /* Configure SPI */
  USART_Reset(spi);
  /* Initialize USART1, in SPI master mode. */
  USART_InitSync(spi, &initSpi);
  /* Enabling pins and setting location, SPI CS not enable */
  spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;
  /* Enabling TX and RX */
  spi->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  /* Clear previous interrupts */
  spi->IFC = _USART_IFC_MASK;

  /* IO configuration (USART 1, Location #1) */
  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief Performs the actual transfer of a byte
 *
 * @param[in] data actual data to be passed towards spi
 *
 * @return actual data to be received from spi
 *****************************************************************************/
static uint8_t ETHSPI_XferSpi(uint8_t data)
{
  USART_TypeDef *spi = USART1;

  spi->TXDATA = data;
  while (!(spi->STATUS & USART_STATUS_TXC))
  {
  }
  return (uint8_t)(spi->RXDATA);
}


/**************************************************************************//**
 * @brief Sets/resets the ChipSelect of the SPI
 *
 * @param[in] value if true CS will be set, and reseted otherwise
 *****************************************************************************/
static void ETHSPI_SetChipSelect(bool value)
{
  if (value)
  {
    /* Enable chip select */
    GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN);
  }
  else
  {
    /* Disable chip select */
    GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN);
  }
}


/**************************************************************************//**
 * @brief Read ethernet controller register
 * @param[in] reg Register to read
 * @param[in] numBytes Number of bytes to read, should be 1-4
 * @param[out] data Pointer to element where data should be put
 *****************************************************************************/
void ETHSPI_ReadRegister(uint8_t reg, int numBytes, void *data)
{
  uint8_t first, second;
  uint8_t *rxBuffer = (uint8_t *) data;

  EFM_ASSERT(reg > 0 && reg < 0xFF);
  EFM_ASSERT(numBytes > 0 && numBytes < 5);
  EFM_ASSERT(data != NULL);

  /* First Opcode:00, ByteEnable and MSB of register address */
  first  = bitEnable[numBytes] << (SHIFT_VAL + (reg & REG_MASK));
  first |= ((reg & ADDRESS_MS2B_MASK) >> ADDRESS_MS2B_POS);

  /* Second LSB of register address, last 4 LSB is ignored */
  second = (reg << SHIFT_VAL);

  /* Enable chip select */
  ETHSPI_SetChipSelect(true);

  ETHSPI_XferSpi(first);
  ETHSPI_XferSpi(second);

  /* Read data back */
  while (numBytes--)
  {
    *rxBuffer++ = ETHSPI_XferSpi(BOGUS_BYTE);
  }
  /* Disable chip select */
  ETHSPI_SetChipSelect(false);
}


/**************************************************************************//**
 * @brief Write ethernet controller register
 * @param[in] reg Register to read
 * @param[in] numBytes Number of bytes to read, should be 1-4
 * @param[out] data Pointer to element where data should be put
 *****************************************************************************/
void ETHSPI_WriteRegister(uint8_t reg, int numBytes, void *data)
{
  uint8_t first, second;
  uint8_t *txBuffer = (uint8_t *) data;

  EFM_ASSERT(reg > 0 && reg < 0xFF);
  EFM_ASSERT(numBytes > 0 && numBytes < 5);
  EFM_ASSERT(data != NULL);

  /* First Opcode:01, ByteEnable and MSB of register address */
  first  = OPCODE_REG_WRITE | (bitEnable[numBytes] << (SHIFT_VAL + (reg & REG_MASK)));
  first |= ((reg & ADDRESS_MS2B_MASK) >> ADDRESS_MS2B_POS);
  /* Second LSB of register address, last 4 LSB is ignored */
  second = (reg << SHIFT_VAL);

  /* Enable chip select */
  ETHSPI_SetChipSelect(true);
  ETHSPI_XferSpi(first);
  ETHSPI_XferSpi(second);

  /* Write Packet data */
  while (numBytes--)
  {
    ETHSPI_XferSpi(*txBuffer++);
  }
  /* Disable chip select */
  ETHSPI_SetChipSelect(false);
}


/**************************************************************************//**
 * @brief Start reading from the ethernet controller FIFO
 *****************************************************************************/
void ETHSPI_StartReadFIFO(void)
{
  /* Enable chip select */
  ETHSPI_SetChipSelect(true);
  /* Send Read command */
  ETHSPI_XferSpi(OPCODE_FIFO_READ);
}


/*************************************************************************//**
* @brief Start writing to the ethernet controller FIFO
*****************************************************************************/
void ETHSPI_StartWriteFIFO(void)
{
  /* Enable chip select */
  ETHSPI_SetChipSelect(true);
  /* Send Write command */
  ETHSPI_XferSpi(OPCODE_FIFO_WRITE);
}


/*************************************************************************//**
* @brief Stop read/write the ethernet controller FIFO
*****************************************************************************/
void ETHSPI_StopFIFO(void)
{
  /* Disable chip select */
  ETHSPI_SetChipSelect(false);
}


/*************************************************************************//**
* @brief Continue reading ethernet controller FIFO
* @param[in]  numBytes Number of bytes to read, 1-12K
* @param[out] data Actual bytes to read
*****************************************************************************/
void ETHSPI_ReadFifoContinue(int numBytes, uint8_t *data)
{
  EFM_ASSERT(numBytes >= 0 && numBytes < 12000);
  EFM_ASSERT(data != NULL);

  uint8_t *rxBuffer = (uint8_t *) data;
  while (numBytes--)
  {
    *rxBuffer++ = ETHSPI_XferSpi(BOGUS_BYTE);
  }
}


/*************************************************************************//**
* @brief Continue writing ethernet controller FIFO
* @param[in] numBytes Number of bytes to write, 1-12K
* @param[in] data Actual bytes to write
*****************************************************************************/
void ETHSPI_WriteFifoContinue(int numBytes, uint8_t *data)
{

  EFM_ASSERT(numBytes >= 0 && numBytes < 12000);
  EFM_ASSERT(data != NULL);

  uint8_t *txBuffer = (uint8_t *) data;
  while (numBytes--)
  {
    ETHSPI_XferSpi(*txBuffer++);
  }
}
