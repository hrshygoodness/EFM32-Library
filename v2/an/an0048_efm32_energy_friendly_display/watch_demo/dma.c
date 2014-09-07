/**************************************************************************//**
 * @file dma.c
 * @brief Copy pixels to Memory LCD with DMA
 * @author Silicon Labs
 * @version 1.05
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
#include "em_dma.h"
#include "em_emu.h"
#include "em_ebi.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "dmactrl.h"
#include "memlcd.h"

/* The DMA channel to use */
#define DMA_CHANNEL 0

/* Max number of elements the DMA can transfer in one cycle */
#define DMA_MAX_TRANSFER_SIZE 1024

/* Frame buffer width, including control signals. 
 * W = 128 px + 8 addr bits + 8 dummy bits */
#define FRAME_BUFFER_WIDTH 144

/* Total width of each line in frame buffer, in bits. 
 * Includes 128 pixels + 8 address bits + 8 dummy bits for SPI
 * + 16 bits of padding required because of a bug in emWin (v 5.16)
 * that requires the display width to be divisble by 32 */
#define FRAME_BUFFER_STRIDE 160

/* Frame buffer control prototypes */
uint16_t *FB_getActiveBuffer(void);

/* Main prototypes */
void invalidateDisplay(void);
void setLowFrequency(void);
void setHighFrequency(void);


/* DMA structures */
DMA_Init_TypeDef dmaInit;
DMA_CB_TypeDef dmaCallback;

/* Flags to check if DMA is currently active */
volatile bool spiTransferActive = false;

/* Configuration structure */
MEMLCD_Config *pConf;



/** Called by DMA when transfer is complete */
static void transferComplete(unsigned int channel, bool primary, void *user) 
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Wait for USART to finish */
  while (!(pConf->usart->STATUS & USART_STATUS_TXC)) ;
  
  /* De-assert SCS */
  GPIO_PinOutClear( pConf->scs.port, pConf->scs.pin );
  
    /* Reset flag to indicate that transfer is done */
  spiTransferActive = false;
}


/**  
  * Configure DMA to send part of, or the entire frame buffer over SPI
  * to the memory LCD. Rectangle copy is used because it can transfer
  * the entire frame in one DMA cycle. 
  * (In addition, the extra padding bits needed because emWin v5.16 does 
  * not handle a 144 px width correctly are skipped by rectangle copy
  * by setting the width to 144 px and stride to 160 px.) 
  */
void configRectangleCopy(USART_TypeDef *usart)
{
  DMA_CfgChannel_TypeDef   channelConfig;
  DMA_CfgDescr_TypeDef     descriptorConfig;

  /* Setting callback function */  
  dmaCallback.cbFunc = transferComplete;
  dmaCallback.userPtr = NULL;

  /* Setting up channel */
  channelConfig.highPri   = false;                /* No high priority */
  channelConfig.enableInt = true;                 /* Enable interrupt */
  
  /* Select USARTx peripheral */
  if ( usart == USART0 ) {
    channelConfig.select = DMAREQ_USART0_TXBL;   
  } else if ( usart == USART1 ) {
    channelConfig.select = DMAREQ_USART1_TXBL;   
  } else if ( usart == USART2 ) {
    channelConfig.select = DMAREQ_USART2_TXBL; 
  }
  
  channelConfig.cb        = &dmaCallback;         /* Callback routine */
  DMA_CfgChannel(DMA_CHANNEL, &channelConfig);

  /* Configure descriptor */
  descriptorConfig.dstInc   = dmaDataIncNone;     /* Do not increase destination */
  descriptorConfig.srcInc   = dmaDataInc2;        /* Increase source by 2 bytes */
  descriptorConfig.size     = dmaDataSize2;       /* Element size is 2 bytes */
  descriptorConfig.arbRate  = dmaArbitrate1;      /* Arbiratrate after each transfer */
  descriptorConfig.hprot    = 0;                  /* Non-privileged access */
  
  /* Configure the LOOP0 register for 2D copy */
  DMA_CfgLoop_TypeDef loopConfig;
  loopConfig.enable = false;
  loopConfig.nMinus1 = FRAME_BUFFER_WIDTH/16-1;  /* Number of elements (-1) to transfer */
  DMA_CfgLoop(DMA_CHANNEL, &loopConfig);
  
  /* Configure the RECT0 register for 2D copy */
  DMA_CfgRect_TypeDef rectConfig;
  rectConfig.dstStride = 0;
  rectConfig.srcStride = FRAME_BUFFER_STRIDE / 8; /* Width of the total frame buffer, in bytes */
  rectConfig.height = 128;
  DMA_CfgRect(DMA_CHANNEL, &rectConfig);
  
  /* Create the descriptor */
  DMA_CfgDescr(DMA_CHANNEL, true, &descriptorConfig); 
}

/** Initialize DMA operation. 
  * Enables the clock and sets up descriptors and 
  * registers for dma frame buffer transfer */
void initDMA(MEMLCD_Config *config) 
{
  pConf = config;
  
  CMU_ClockEnable(cmuClock_DMA, true);
  
  dmaInit.hprot = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Configure registers and descriptors for frame buffer transfer */
  configRectangleCopy(pConf->usart);
}


void dmaStartFrameTransfer(int firstLine, int lastLine)
{
  /* Get address of first line */
  uint16_t *startAddr = FB_getActiveBuffer();
  startAddr += firstLine * 10;
  
  /* Create update command and address of first line */
  uint16_t cmd = MEMLCD_CMD_UPDATE | ((firstLine+1) << 8); 
  
  /* Enable chip select */
  GPIO_PinOutSet( pConf->scs.port, pConf->scs.pin );
  
  /* Set number of lines to copy */
  DMA->RECT0 = (DMA->RECT0 & ~_DMA_RECT0_HEIGHT_MASK) | (lastLine - firstLine);
  
  /* Indicate to the rest of the program that SPI transfer is in progress */
  spiTransferActive = true;
  
  /* Send the update command */
  USART_TxDouble(pConf->usart, cmd);
    
  /* Start the transfer */
  DMA_ActivateBasic(DMA_CHANNEL,
                    true,                               /* Use primary channel */
                    false,                              /* No burst */
                    (void *)&(pConf->usart->TXDOUBLE),  /* Write to USART */
                    startAddr,                          /* Start address */
                    FRAME_BUFFER_WIDTH/16-1);           /* Width -1 */  
  
}



