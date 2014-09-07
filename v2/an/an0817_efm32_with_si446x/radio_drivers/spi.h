/*!
 * File:
 *  spi.h
 *  author: Shaoxian Luo
 *
 * Description:
 *  This file is the driver of the spi which is used for the communication with the radio
 *
 * Silicon Laboratories Confidential
 * Copyright 2013 Silicon Laboratories, Inc.
 */
#ifndef __SPI_H__
#define __SPI_H__

/*! SPI device select enum */

typedef enum
{
  eSpi_Nsel_RF,
} eSpi_Nsel;

#ifdef EFM32LG990F256
#define SPI_CS_PORT     gpioPortD
#define SPI_MOSI_PORT   gpioPortD 
#define SPI_MISO_PORT   gpioPortD 
#define SPI_CLK_PORT    gpioPortD 

#define SPI_MOSI_PIN    0 
#define SPI_MISO_PIN    1 
#define SPI_CLK_PIN     2 
#define SPI_CS_PIN      3
#endif

#ifdef EFM32TG840F32
#define SPI_CS_PORT     gpioPortD
#define SPI_MOSI_PORT   gpioPortD 
#define SPI_MISO_PORT   gpioPortD 
#define SPI_CLK_PORT    gpioPortD 

#define SPI_MOSI_PIN    0 
#define SPI_MISO_PIN    1 
#define SPI_CLK_PIN     2 
#define SPI_CS_PIN      3
#endif

/*------------------------------------------------------------------------*/
/*                           Function prototypes                          */
/*------------------------------------------------------------------------*/
void vSpiInitialize(void);
uint8_t bSpiReadWrite(uint8_t biDataIn);
void vSpiWriteDataBurst(uint8_t biDataInLength, uint8_t *pabiDataIn);
void vSpiReadDataBurst(uint8_t biDataOutLength, uint8_t *paboDataOut);
void vSpi_ClearNsel(eSpi_Nsel qiSelect);
void vSpi_SetNsel(eSpi_Nsel qiSelect);

















#endif
