/*****************************************************************************
 * @file memory_scatter_gather.c
 * @brief DMA memory Scatter-Gather Example
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

#define DMA_CHANNEL_SCATTERGATHER       0

/* DMA init structure */
DMA_Init_TypeDef dmaInit;
/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Transfer Flag */
volatile bool transferActive;

/* Scatter Gather Specific */
#define SCATTER_GATHER_TRANSFERS            3
const char scatterGatherSourceA[]       = "First (A) task text";
const char scatterGatherSourceB[]       = "Second (B) task text";
const char scatterGatherSourceC[]       = "Third (C) task text";
#define SOURCE_A_LENGTH       (sizeof(scatterGatherSourceA)/sizeof(char))
#define SOURCE_B_LENGTH       (sizeof(scatterGatherSourceB)/sizeof(char))
#define SOURCE_C_LENGTH       (sizeof(scatterGatherSourceC)/sizeof(char))
volatile char scatterGatherDestA[SOURCE_A_LENGTH];
volatile char scatterGatherDestB[SOURCE_B_LENGTH];
volatile char scatterGatherDestC[SOURCE_C_LENGTH];
DMA_DESCRIPTOR_TypeDef dmaScatterCfgBlock[SCATTER_GATHER_TRANSFERS];


/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{      
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Clearing Flag */
  transferActive = false;    
}


/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
}


/**************************************************************************//**
 * @brief Configure DMA for Scatter Gather transfers
 *****************************************************************************/
void configureDmaTransfer(void)
{
  DMA_CfgChannel_TypeDef    chnlCfg;
  DMA_CfgDescrSGAlt_TypeDef cfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
/* Perform DMA transfer from Flash to Ram */  
  cb.cbFunc  = transferComplete; 
  cb.userPtr = NULL;


  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = 0;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_SCATTERGATHER, &chnlCfg);
  
  /* Setting up the transfers in the scatter-gather sequence */
  cfg.src        = (void *) &scatterGatherSourceA;       
  cfg.dst        = (void *) &scatterGatherDestA;       
  cfg.dstInc     = dmaDataInc1;
  cfg.srcInc     = dmaDataInc1;     
  cfg.size       = dmaDataSize1;       
  cfg.arbRate    = dmaArbitrate1;    
  cfg.nMinus1    = SOURCE_A_LENGTH-1;
  cfg.hprot      = 0;      
  cfg.peripheral = false; 
  DMA_CfgDescrScatterGather(&dmaScatterCfgBlock[0], 0, &cfg);

  /* Changing the source and destination addresses, length and configuring */
  cfg.src = (void *) &scatterGatherSourceB;  
  cfg.dst = (void *) &scatterGatherDestB;
  cfg.nMinus1 = SOURCE_B_LENGTH-1;
  DMA_CfgDescrScatterGather(&dmaScatterCfgBlock[0], 1, &cfg);
  
  /* Changing the source and destination addresses, length and configuring */
  cfg.src = (void *) &scatterGatherSourceC;  
  cfg.dst = (void *) &scatterGatherDestC;
  cfg.nMinus1 = SOURCE_C_LENGTH-1;
  DMA_CfgDescrScatterGather(&dmaScatterCfgBlock[0], 2, &cfg);

  /* Setting flag to indicate that transfer is in progress
  *  will be cleared by call-back function */
  transferActive = true; 
}


/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure DMA scatter-gather transfer */      
  configureDmaTransfer();
  
  /* Starting the scatter-gather DMA operation */
  DMA_ActivateScatterGather(DMA_CHANNEL_SCATTERGATHER,
                            false,
                            &dmaScatterCfgBlock[0],
                            SCATTER_GATHER_TRANSFERS);
  
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
