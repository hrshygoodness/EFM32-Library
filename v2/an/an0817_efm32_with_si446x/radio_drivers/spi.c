/*!
 * File:
 *  spi.c
 *  author: Shaoxian Luo
 *
 * Description:
 *  This file is the driver of the spi which is used for the communication with the radio
 *
 * Silicon Laboratories Confidential
 * Copyright 2013 Silicon Laboratories, Inc.
 */

#include "bsp_def.h"
 
/*!
 * This function is used to initialize the SPI port.
 *
 *  @return None
 */
void vSpiInitialize(void)
{
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

  /* Configure SPI */
  USART_Reset(USART1);

  init.baudrate     = 5000000;
  init.databits     = usartDatabits8;
  init.msbf         = 1;
  init.master       = 1;
  init.clockMode    = usartClockMode0;
  init.prsRxEnable  = 0;
  init.autoTx       = 0;

  USART_InitSync(USART1, &init);

  /* IO configuration (USART 1, Location #1) */
  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, 1);
}

/*!
* This function is used to read/write one byte from/to SPI port (target: EzRadioPRO).
 *
 * @param[in] biDataIn    Data to be sent.
 * @return  Read value of the SPI port after writing on it.
 */   
uint8_t bSpiReadWrite(uint8_t biDataIn)
{  
  USART_TypeDef *spi = USART1;
 
  spi->TXDATA = biDataIn;
  while (!(spi->STATUS & USART_STATUS_TXC))
  {
  }
  return (uint8_t)(spi->RXDATA);
}

/*!
 * This function is used to send data over SPI port (target: EzRadioPRO).no response expected.
 *
 *  @param[in] biDataInLength  The length of the data.
 *  @param[in] *pabiDataIn     Pointer to the first element of the data.
 *
 *  @return None
 */
void vSpiWriteDataBurst(uint8_t biDataInLength, uint8_t *pabiDataIn)
{
  while (biDataInLength--) {
    bSpiReadWrite(*pabiDataIn++);
  }
}

/*!
 * This function is used to read data from SPI port.(target: EzRadioPRO).
 *
 *  \param[in] biDataOutLength  The length of the data.
 *  \param[out] *paboDataOut    Pointer to the first element of the response.
 *
 *  \return None
 */
void vSpiReadDataBurst(uint8_t biDataOutLength, uint8_t *paboDataOut)
{
  // send command and get response from the radio IC
  while (biDataOutLength--) {
    *paboDataOut++ = bSpiReadWrite(0xFF);
  }
}

/**
 *  Pull down nSEL of the selected device.
 *
 *  @param[in] qiSelect Device select.
 *
 *  @note Input: qiSelect <br> 0 - \b DUT <br> 1 - \b LCD <br>
 *
 ******************************************************************************/
void vSpi_ClearNsel(eSpi_Nsel qiSelect)
{
  switch (qiSelect)
  {
      case eSpi_Nsel_RF:
        GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN);
        break;
      default:
        break;
  }
}

/**
 *  Pull-up nSEL of the selected device.
 *
 *  @param[in] qiSelect Device select.
 *
 *  @note Input: qiSelect <br> 0 - \b DUT <br> 1 - \b LCD <br>
 *
 ******************************************************************************/
void vSpi_SetNsel(eSpi_Nsel qiSelect)
{
  switch (qiSelect)
  {
      case eSpi_Nsel_RF:
        GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN);
        break;
      default:
        break;
  }
}

