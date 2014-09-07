/*****************************************************************************
 * @file spi_tx_ping_pong.c
 * @brief DMA Ping-Pong SPI transfer example
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

#define DMA_CHANNEL_SPI       0

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Transfer Flag */
volatile bool transferActive;

/* SPI Transmit Data */
const char spiData[] = "Hello World! This is Gecko!";
#define SPI_PINGPONG_TRANSFERS  3
#define SPI_TRANSFER_SIZE       (sizeof(spiData)/sizeof(char))


/**************************************************************************//**
 * @brief  USART1_TX Interrupt handler
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  USART_IntClear(USART1, USART_IFC_TXC);
  
  if (transferActive)
   while(1); /* ERROR Underflow in USART. DMA not able to keep up */ 
}

/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) user;
 
  static int transfernumber = 0;
  
  /* Keeping track of the number of transfers */
  transfernumber++;
  
  /* Let the transfer be repeated a few times to illustrate re-activation
  * Stop re-activating when 1 transfer remains as one transfer will be ongoing 
  * while this callback runs */
  if (transfernumber < SPI_PINGPONG_TRANSFERS-1) 
  {
    /* Re-activate the DMA */
    DMA_RefreshPingPong(channel,
                        primary,
                        false,
                        NULL,
                        NULL,
                        SPI_TRANSFER_SIZE/2 - 1,
                        false);
  }
  
  /* Set active flag low when all transfers are done */
  if (transfernumber == SPI_PINGPONG_TRANSFERS)
    /* Clearing Flag */
    transferActive = false;  
}


/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{
  /* Set HFRCO frequency to 28 MHz */
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);  
  CMU_ClockEnable(cmuClock_USART1, true);  
}


/**************************************************************************//**
 * @brief Configure DMA for Ping-pong RAM to SPI TX
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_USART1_TXBL;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_SPI, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataIncNone;
  descrCfg.srcInc  = dmaDataInc2;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_SPI, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_SPI, false, &descrCfg);
  
  /* Setting flag to indicate that transfer is in progress
    will be cleared by call-back function */
  transferActive = true;

  /* Enabling PingPong Transfer*/  
  DMA_ActivatePingPong(DMA_CHANNEL_SPI,
                       false,
                       (void *)&(USART1->TXDOUBLE),
                       (void *)&spiData,
                       SPI_TRANSFER_SIZE/2 - 1,
                       (void *)&(USART1->TXDOUBLE),
                       (void *)&spiData,
                        SPI_TRANSFER_SIZE/2 - 1);
}


/**************************************************************************//**
 * @brief  Setup SPI
 *****************************************************************************/
void setupSpi(void)
{
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;  
  
  /* Initialize SPI */
  usartInit.databits = usartDatabits16;
  usartInit.baudrate = 14000000;
  USART_InitSync(USART1, &usartInit);
  
  /* Turn on automatic Chip Select control */
  USART1->CTRL |= USART_CTRL_AUTOCS;
  
  /* Enable SPI transmit */
  USART_Enable(USART1, usartEnableTx);
  
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
  
  /* Enable TX Complete interrupt to indicate when USART has sent all its data */
  USART_IntEnable(USART1, USART_IEN_TXC);
  NVIC_EnableIRQ(USART1_TX_IRQn);
}


/**************************************************************************//**
 * @brief  Main function
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

  /* Enter EM1 while DMA transfer is active to save power. Note that
   * interrupts are disabled to prevent the ISR from being triggered
   * after checking the transferActive flag, but before entering
   * sleep. If this were to happen, there would be no interrupt to wake
   * the core again and the MCU would be stuck in EM1. While the 
   * core is in sleep, pending interrupts will still wake up the 
   * core and the ISR will be triggered after interrupts are enabled
   * again. 
   */
  while(1)
  {
    INT_Disable();
    if ( transferActive )
    {
      EMU_EnterEM1(); 
    }
    INT_Enable();
    
    /* Exit the loop if transfer has completed */
    if ( !transferActive )
    {
      break;
    }
  }
 
  /* Cleaning up after DMA transfers */
  DMA_Reset();

  /* Done */
  while (1);
}
