/**************************************************************************//**
 * @file dma_frame_copy.c
 * @brief Copy frame from PSRAM to display controller
 * @author Silicon Labs
 * @version 1.06
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
#include "dmactrl.h"
#include "dma_frame_copy.h"

/* Application specific defines */
#include "display_conf.h"

/* Use channel 0 */
#define DMA_CHANNEL 0

/* Max number of elements the DMA can transfer in one cycle */
#define DMA_MAX_TRANSFER_SIZE 1024

/* 320 * 240 pixels, 1024 pixels per dma transfer = 75 transfers */
#define NUM_DMA_TRANSFERS 75

/* Address to write pixel data to. 
 * This value depends on the EBI bank and 
 * Which address line is connected to D/C 
 */
#define LCD_PIXEL_DATA_ADDRESS 0x84000002

/* DMA init structure */
DMA_Init_TypeDef dmaInit;

/* DMA callback structure */
DMA_CB_TypeDef dmaCallback;

/* Flag to check if DMA is currently active */
volatile bool dmaTransferActive = false;

/* Array containing descriptors which will be sequentially loaded
 * to move the whole frame buffer in one DMA operation */
DMA_DESCRIPTOR_TypeDef dmaScatterGatherDescriptors[NUM_DMA_TRANSFERS];


/* Enable clocks and initialize DMA registers */
void initDMA(void) 
{
  CMU_ClockEnable(cmuClock_DMA, true);
  
  dmaInit.hprot = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
}


/* Called by DMA when transfer is complete */
void transferComplete(unsigned int channel, bool primary, void *user) 
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Reset flag to indicate that transfer is done */
  dmaTransferActive = false;
}


/* Start a DMA transfer and return */
void startDmaTransfer(uint16_t *sourceAddress, int numElements) 
{
  int i;
  int elementsLeft = numElements;
  DMA_CfgChannel_TypeDef channelConfig;
  DMA_CfgDescrSGAlt_TypeDef    descriptorConfig;

  /* Setting call-back function */  
  dmaCallback.cbFunc = transferComplete;
  dmaCallback.userPtr = NULL;

  /* Setting up channel */
  channelConfig.highPri = false;              /* No high priority */
  channelConfig.enableInt = true;             /* Enable interrupt */
  channelConfig.select = 0;                   /* Memory to memory transfer */
  channelConfig.cb = &dmaCallback;            /* Callback routine */
  DMA_CfgChannel(DMA_CHANNEL, &channelConfig);

  /* Common values for all the descriptors */
  descriptorConfig.dstInc = dmaDataIncNone;   /* Do not increase destination */
  descriptorConfig.srcInc = dmaDataInc2;      /* Increase source by 2 bytes */
  descriptorConfig.size = dmaDataSize2;       /* Element size is 2 bytes */
  descriptorConfig.arbRate = dmaArbitrate1;    
  descriptorConfig.hprot = 0;
  descriptorConfig.nMinus1 = DMA_MAX_TRANSFER_SIZE-1;
  descriptorConfig.peripheral = false; 
  descriptorConfig.dst = (void *)LCD_PIXEL_DATA_ADDRESS;
  
  for ( i=0; i<NUM_DMA_TRANSFERS; i++ ) {
    
    /* Configure number of elements */
    if ( elementsLeft == 0 ) {
      break;
    } else if ( elementsLeft < DMA_MAX_TRANSFER_SIZE ) {
      descriptorConfig.nMinus1 = elementsLeft-1;
      elementsLeft = 0;
    } else {
      elementsLeft -= DMA_MAX_TRANSFER_SIZE;
    }
    
    /* Set up source for all descriptors */
    descriptorConfig.src = (void *) (sourceAddress + i*DMA_MAX_TRANSFER_SIZE);

    /* Create the descriptor */
    DMA_CfgDescrScatterGather(dmaScatterGatherDescriptors, i, &descriptorConfig); 
  }
  
  /* Set transfer active flag */
  dmaTransferActive = true;
  
  /* Start the transfer */
  DMA_ActivateScatterGather(DMA_CHANNEL,
                            false,
                            dmaScatterGatherDescriptors,
                            NUM_DMA_TRANSFERS); 
  
}



/* Copy the entire frame buffer from PSRAM to LCD controller, using DMA */
void copyFrameToLcd(uint16_t *frameBufferStart, int halfwords) 
{
  /* Start DMA transfer */
  startDmaTransfer(frameBufferStart, halfwords);
  
  /* Sleep until transfer is finished. 
   * This is done to prevent software from overwriting the frame buffer
   * before it is copied to the display (would cause flickering) */
  while(dmaTransferActive) {
    EMU_EnterEM1();
  }
}


