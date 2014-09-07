/**************************************************************************//**
 * @file
 * @brief DMA Auto Example
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dma.h"

#include "em_lcd.h"
#include "segmentlcd.h"

/* DMA control block, must be aligned to 256. */
#if defined (__ICCARM__)
#pragma data_alignment=256
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
#error Undefined toolkit, need to define alignment
#endif

/* DMA channel to use */
#define DMA_CHANNEL_FLASH    0

/* Status of the transfer */
bool flashTransferActive;

/* The const array will be stored in flash */
const char flashData[8] = "TESTING";

/* Size of the transfer */
#define FLASHDATA_SIZE    (sizeof(flashData))

/* Buffer to transfer to - initialize with zeros */
char ramBuffer[9] = { 0 };

/**************************************************************************//**
 * @brief DMA callback function
 * This function is called when the DMA transfer is complete
 *****************************************************************************/
void flashTransferComplete(unsigned int channel, bool primary, void *user)
{
  /* Clearing flag to indicate that transfer is complete */
  flashTransferActive = false;

  /* Indicate that the transfer is complete turning on the antenna symbol
   * and write the content of the ramBuffer to the LCD screen */
  SegmentLCD_Symbol(LCD_SYMBOL_ANT, true);
  SegmentLCD_Write(ramBuffer);
}

/**************************************************************************//**
 * @brief Flash transfer function
 * This function sets up the DMA transfer
 *****************************************************************************/
void performFlashTransfer(void)
{
  /* Setting call-back function */
  DMA_CB_TypeDef cb[DMA_CHAN_COUNT];
  cb[DMA_CHANNEL_FLASH].cbFunc = flashTransferComplete;

  /* usrPtr can be used to send data to the callback function,
   * but this is not used here, which is indicated by the NULL pointer */
  cb[DMA_CHANNEL_FLASH].userPtr = NULL;

  /* Setting up channel */
  DMA_CfgChannel_TypeDef chnlCfg;
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = 0;
  chnlCfg.cb        = &(cb[DMA_CHANNEL_FLASH]);
  DMA_CfgChannel(DMA_CHANNEL_FLASH, &chnlCfg);

  /* Setting up transfer descriptor */
  DMA_CfgDescr_TypeDef descrCfg;
  descrCfg.dstInc  = dmaDataInc1;
  descrCfg.srcInc  = dmaDataInc1;
  descrCfg.size    = dmaDataSize1;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_FLASH, true, &descrCfg);

  /* Setting flag to indicate that transfer is in progress
   * will be cleared by call-back function */
  flashTransferActive = true;

  /* Starting the transfer. Using Auto (all data is transfered at once) */
  DMA_ActivateAuto(DMA_CHANNEL_FLASH,
                   true,
                   (void *) &ramBuffer,
                   (void *) &flashData,
                   FLASHDATA_SIZE - 1);

  /* Entering EM1 to wait for completion (the DMA requires EM1) */
  while (flashTransferActive)
  {
    EMU_EnterEM1();
  }
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Initialize LCD */
  SegmentLCD_Init(true);

  /* Enable the DMA clock */
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Initialize DMA */
  DMA_Init_TypeDef dmaInit;
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure the DMA and perform the transfer */
  performFlashTransfer();

  while (1)
  {
    /* The transfer has finished. We wish to display the result on the LCD
     * but use as little power as possible; go to EM2 (the LCD requires EM2). */
    EMU_EnterEM2(true);
  }
}
