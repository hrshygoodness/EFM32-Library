/*****************************************************************************
 * @file adc_basic.c
 * @brief DMA Basic ADC transfer example
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
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_int.h"
#include "dmactrl.h"

#define DMA_CHANNEL_ADC       0

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Transfer Flag */
volatile bool transferActive;

/* ADC Transfer Data */
#define ADCSAMPLES                        20
volatile uint16_t ramBufferAdcData[ADCSAMPLES];
#define ADCSAMPLESPERSEC              100000



/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Stopping ADC by stopping TIMER0 */
   TIMER_Enable(TIMER0, false);
  
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
  CMU_ClockEnable(cmuClock_ADC0, true);  
  CMU_ClockEnable(cmuClock_TIMER0, true); 
  CMU_ClockEnable(cmuClock_PRS, true); 
}



/**************************************************************************//**
 * @brief Configure DMA for ADC RAM Transfer
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
    
  /* Setting up call-back function */  
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);
    
  /* Setting flag to indicate that transfer is in progress
   * will be cleared by call-back function. */
  transferActive = true;
  
  /* Starting transfer. Using Basic since every transfer must be initiated
   * by the ADC. */
  DMA_ActivateBasic(DMA_CHANNEL_ADC,
                    true,
                    false,
                    (void *)ramBufferAdcData,
                    (void *)&(ADC0->SINGLEDATA),
                    ADCSAMPLES - 1);
}



/**************************************************************************//**
 * @brief Configure TIMER to trigger ADC through PRS at a set sample rate
 *****************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;
  
  /* Configure ADC single mode to sample Ref/2 */
  adcInit.prescale = ADC_PrescaleCalc(7000000, 0); /* Set highest allowed prescaler */
  ADC_Init(ADC0, &adcInit);
  
  adcInitSingle.input     =  adcSingleInpVrefDiv2;  /* Reference */
  adcInitSingle.prsEnable = true;                  
  adcInitSingle.prsSel    = adcPRSSELCh0;           /* Triggered by PRS CH0 */
  ADC_InitSingle(ADC0, &adcInitSingle);
  
  /* Connect PRS channel 0 to TIMER overflow */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);
  
  /* Configure TIMER to trigger 100 kHz sampling rate */
  TIMER_TopSet(TIMER0,  CMU_ClockFreqGet(cmuClock_TIMER0)/ADCSAMPLESPERSEC);
  TIMER_Enable(TIMER0, true);
}



/**************************************************************************//**
 * @brief  Main function
 * This exmaple sets up the TIMER to trigger the ADC through PRS at a set
 * interval. The ADC then sets a DMA request and the DMA fetches each sample
 * until the a set number of samples have been received. 
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure DMA transfer from ADC to RAM */      
  setupDma();
  
  /* Configure ADC Sampling and TIMER trigger through PRS. Start TIMER as well */
  setupAdc();
  
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
