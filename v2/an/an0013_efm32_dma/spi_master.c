/*****************************************************************************
 * @file spi_master.c
 * @brief DMA SPI master transmit/receive example
 * @author Silicon Labs
 * @version 2.06
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

#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "dmactrl.h"

#define DMA_CHANNEL_TX   0
#define DMA_CHANNEL_RX   1
#define DMA_CHANNELS     2

/* DMA Callback structure */
DMA_CB_TypeDef spiCallback;

/* Transfer Flags */
volatile bool rxActive;
volatile bool txActive;

/* SPI Data Buffers */
const char spiTxData[] = "Hello World! This is Gecko!";
#define SPI_TRANSFER_SIZE (sizeof(spiTxData)/sizeof(char))
volatile char spiRxData1[SPI_TRANSFER_SIZE];
volatile char spiRxData2[SPI_TRANSFER_SIZE];


/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) primary;
  (void) user;
  
  /* Clear flag to indicate complete transfer */
  if (channel == DMA_CHANNEL_TX)
  {
    txActive = false;  
  }
  else if (channel == DMA_CHANNEL_RX)
  {
    rxActive = false;
  }
}



/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);  
  CMU_ClockEnable(cmuClock_USART1, true);  
}



/**************************************************************************//**
 * @brief  Setup SPI as Master
 *****************************************************************************/
void setupSpi(void)
{
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;  
  
  /* Initialize SPI */
  usartInit.databits = usartDatabits8;
  usartInit.baudrate = 1000000;
  USART_InitSync(USART1, &usartInit);
  
  /* Turn on automatic Chip Select control */
  USART1->CTRL |= USART_CTRL_AUTOCS;
  
  /* Enable SPI transmit and receive */
  USART_Enable(USART1, usartEnable);
  
  /* Configure GPIO pins for SPI */
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0); /* MOSI */
  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput,    0); /* MISO */
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0); /* CLK */	
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1); /* CS */	
 
  /* Enable routing for SPI pins from USART to location 1 */
  USART1->ROUTE = USART_ROUTE_TXPEN | 
                  USART_ROUTE_RXPEN | 
                  USART_ROUTE_CSPEN | 
                  USART_ROUTE_CLKPEN | 
                  USART_ROUTE_LOCATION_LOC1;
}



/**************************************************************************//**
 * @brief Configure DMA in basic mode for both TX and RX to/from USART
 *****************************************************************************/
void setupDma(void)
{
  /* Initialization structs */
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  rxChnlCfg;
  DMA_CfgDescr_TypeDef    rxDescrCfg;
  DMA_CfgChannel_TypeDef  txChnlCfg;
  DMA_CfgDescr_TypeDef    txDescrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  spiCallback.cbFunc  = transferComplete;
  spiCallback.userPtr = NULL;
  
  /*** Setting up RX DMA ***/

  /* Setting up channel */
  rxChnlCfg.highPri   = false;
  rxChnlCfg.enableInt = true;
  rxChnlCfg.select    = DMAREQ_USART1_RXDATAV;
  rxChnlCfg.cb        = &spiCallback;
  DMA_CfgChannel(DMA_CHANNEL_RX, &rxChnlCfg);

  /* Setting up channel descriptor */
  rxDescrCfg.dstInc  = dmaDataInc1;
  rxDescrCfg.srcInc  = dmaDataIncNone;
  rxDescrCfg.size    = dmaDataSize1;
  rxDescrCfg.arbRate = dmaArbitrate1;
  rxDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_RX, true, &rxDescrCfg);
  
  /*** Setting up TX DMA ***/

  /* Setting up channel */
  txChnlCfg.highPri   = false;
  txChnlCfg.enableInt = true;
  txChnlCfg.select    = DMAREQ_USART1_TXBL;
  txChnlCfg.cb        = &spiCallback;
  DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

  /* Setting up channel descriptor */
  txDescrCfg.dstInc  = dmaDataIncNone;
  txDescrCfg.srcInc  = dmaDataInc1;
  txDescrCfg.size    = dmaDataSize1;
  txDescrCfg.arbRate = dmaArbitrate1;
  txDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);
}



/**************************************************************************//**
 * @brief  SPI DMA Transfer
 * NULL can be input as txBuffer if tx data to transmit dummy data
 * If only sending data, set rxBuffer as NULL to skip DMA activation on RX
 *****************************************************************************/
void spiDmaTransfer(uint8_t *txBuffer, uint8_t *rxBuffer,  int bytes)
{ 
  /* Only activate RX DMA if a receive buffer is specified */  
  if (rxBuffer != NULL)
  {
    /* Setting flag to indicate that RX is in progress
     * will be cleared by call-back function */
    rxActive = true;
    
    /* Clear RX regsiters */
    USART1->CMD = USART_CMD_CLEARRX;
    
    /* Activate RX channel */
    DMA_ActivateBasic(DMA_CHANNEL_RX,
                      true,
                      false,
                      rxBuffer,
                      (void *)&(USART1->RXDATA),
                      bytes - 1); 
  }
  /* Setting flag to indicate that TX is in progress
   * will be cleared by call-back function */
  txActive = true;
  
  /* Clear TX regsiters */
  USART1->CMD = USART_CMD_CLEARTX;
  
  /* Activate TX channel */
  DMA_ActivateBasic(DMA_CHANNEL_TX,
                    true,
                    false,
                    (void *)&(USART1->TXDATA),
                    txBuffer,
                    bytes - 1); 
}



/**************************************************************************//**
 * @brief  Returns if an SPI transfer is active
 *****************************************************************************/
bool spiDmaIsActive(void)
{
  bool temp;
  temp = rxActive;
  temp = temp | txActive;
  return temp;
}



/**************************************************************************//**
 * @brief  Sleep in EM1 until DMA transfer is done
 *****************************************************************************/
void sleepUntilDmaDone(void)
{
  /* Enter EM1 while DMA transfer is active to save power. Note that
   * interrupts are disabled to prevent the ISR from being triggered
   * after checking the transferActive flag, but before entering
   * sleep. If this were to happen, there would be no interrupt to wake
   * the core again and the MCU would be stuck in EM1. While the 
   * core is in sleep, pending interrupts will still wake up the 
   * core and the ISR will be triggered after interrupts are enabled
   * again. 
   */ 
  bool isActive = false;
  
  while(1)
  {
    INT_Disable();
    isActive = spiDmaIsActive();
    if ( isActive )
    {
      EMU_EnterEM1(); 
    }
    INT_Enable();
    
    /* Exit the loop if transfer has completed */
    if ( !isActive )
    {
      break;
    }
  }  
}



/**************************************************************************//**
 * @brief  Main function
 * This example sets up the DMA to transfer outbound and incoming data from the
 * SPI (USART1) to/from the source/destination buffers. Three tests are done:
 * 1) Transmit data (string) without reading received data
 * 2) Transmit data (string) and transfer received data to RAM buffer
 * 3) Transmit dummy data and transfer received data to RAM buffer
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configura USART for SPI */
  setupSpi();
    
  /* Configure DMA transfer from RAM to SPI using ping-pong */      
  setupDma();
  
  /* Send data to slave, no data reception */
  spiDmaTransfer((uint8_t*) spiTxData, NULL, SPI_TRANSFER_SIZE);
  
  /* Sleep until DMA is done */
  sleepUntilDmaDone();
  
  /* Send data to slave and save received data in buffer */
  spiDmaTransfer((uint8_t*) spiTxData, (uint8_t*) spiRxData1, SPI_TRANSFER_SIZE);
  
  /* Sleep until DMA is done */
  sleepUntilDmaDone();
  
  /* Send dummy data to slave and save received data in buffer */
  spiDmaTransfer(NULL, (uint8_t*) spiRxData2, SPI_TRANSFER_SIZE);

  /* Sleep until DMA is done */
  sleepUntilDmaDone();
 
  /* Cleaning up after DMA transfers */
  DMA_Reset();

  /* Done */
  while (1);
}
