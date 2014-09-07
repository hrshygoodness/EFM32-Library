/*****************************************************************************
 * @file flash_to_ram.c
 * @brief DMA Flash to RAM transfer example
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
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "dmactrl.h"

#define DMA_CHANNEL_FLASHTORAM       0

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Transfer Flag */
volatile bool transferActive;

/* Flash Transfer Data */
const uint32_t flashData[] = {0,1,2,3,4,5,6,7,8,9};
#define FLASHDATA_SIZE (sizeof(flashData)/sizeof(uint32_t))
volatile uint32_t ramBufferFlashData[FLASHDATA_SIZE];



/**************************************************************************//**
* @brief  Call-back called when flash transfer is complete
*****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Clearing flag to indicate that transfer is complete */
  transferActive = false;  
}



/**************************************************************************//**
* @brief  Enable clocks
*****************************************************************************/
void setupCmu(void)
{
  /* Enable clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
}



/**************************************************************************//**
* @brief Configure DMA for Flash to RAM transfer
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
  
  /* Setting call-back function */  
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;
  
  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = 0;
  chnlCfg.cb        = &(cb);
  DMA_CfgChannel(DMA_CHANNEL_FLASHTORAM, &chnlCfg);
  
  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc4;
  descrCfg.srcInc  = dmaDataInc4;
  descrCfg.size    = dmaDataSize4;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_FLASHTORAM, true, &descrCfg);
}



/**************************************************************************//**
* @brief  Main function
* This example shows how to copy a block of data from the flash to RAM using
* a DMA auto transfer.
*****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure DMA transfer from Flash to RAM */      
  setupDma();
  
  /* Setting flag to indicate that transfer is in progress
  *  will be cleared by call-back function */
  transferActive = true;
  
  /* Starting the transfer. Using Auto (all data is transfered at once) */
  DMA_ActivateAuto(DMA_CHANNEL_FLASHTORAM,
                   true,
                   (void *)ramBufferFlashData,
                   (void *)flashData,
                   FLASHDATA_SIZE - 1);
  
  
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




