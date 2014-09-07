/*****************************************************************************
 * @file adc_ping_pong.c
 * @brief DMA Ping-Pong ADC transfer example
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
#include "em_int.h"
#include "dmactrl.h"

#define DMA_CHANNEL_ADC       0

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* Transfer Flag */
volatile bool transferActive;

/* ADC Transfer Data */
#define ADC_PINGPONG_TRANSFERS            10
#define ADCSAMPLES                        20
volatile uint16_t ramBufferAdcData1[ADCSAMPLES];
volatile uint16_t ramBufferAdcData2[ADCSAMPLES];



/**************************************************************************//**
 * @brief  ADC Interrupt handler
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Clear interrupt flag */
  ADC_IntClear(ADC0, ADC_IFC_SINGLEOF);
  
  while(1){
    /* ERROR: ADC Result overflow has occured
     * This indicates that the DMA is not able to keep up with the ADC sample
     * rate and that a samples has been written to the ADC result registers
     * before the DMA was able to fetch the previous result */ 
  }
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
  
  /* Let the transfer be repeated a few times to illustrate re-activation */
  if (transfernumber < (ADC_PINGPONG_TRANSFERS)) 
  {
    /* Re-activate the DMA */
    DMA_RefreshPingPong(channel,
                        primary,
                        false,
                        NULL,
                        NULL,
                        ADCSAMPLES - 1,
                        false);
  }
  
  else
  {
    /* Stopping ADC */
    ADC_Reset(ADC0);
       
    /* Clearing Flag */
    transferActive = false;  
  } 
}



/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{
  /* Set HFRCO frequency to 21 MHz */
  CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
  
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA,  true);  
  CMU_ClockEnable(cmuClock_ADC0, true);  
}



/**************************************************************************//**
 * @brief Configure DMA for Ping-pong ADC to RAM Transfer
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
  DMA_CfgDescr(DMA_CHANNEL_ADC, false, &descrCfg);
  
  /* Setting flag to indicate that transfer is in progress
    will be cleared by call-back function */
  transferActive = true;

  /* Enabling PingPong Transfer*/  
  DMA_ActivatePingPong(DMA_CHANNEL_ADC,
                          false,
                          (void *)ramBufferAdcData1,
                          (void *)&(ADC0->SINGLEDATA),
                          ADCSAMPLES - 1,
                          (void *)ramBufferAdcData2,
                          (void *)&(ADC0->SINGLEDATA),
                          ADCSAMPLES - 1);
}


/**************************************************************************//**
 * @brief Configure ADC to sample Vref/2 repeatedly at 10.5/13 Msamples/s
 *****************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;
  
  /* Configure ADC single mode to sample Vref/2 at 10.5 MHz */
  /* With a 21 MHz clock, this gives the DMA 26 cycles to fetch each result */
  adcInit.prescale = ADC_PrescaleCalc(10500000, 0); /* Prescale to 10.5 MHz */
  ADC_Init(ADC0, &adcInit);
  
  adcInitSingle.input =  adcSingleInpVrefDiv2; /* Reference */
  adcInitSingle.rep   = true;                  
  ADC_InitSingle(ADC0, &adcInitSingle);
  
  /* Enable ADC single overflow interrupt to indicate lost samples */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLEOF);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Start repetitive ADC single conversions */
  ADC_Start(ADC0, adcStartSingle);
}

/**************************************************************************//**
 * @brief  Main function
 * The ADC is set up to sample at full speed with a clock speed of half
 * the HFCORECLK used by the DMA. The DMA transfers the data to a RAM buffer
 * using ping-pong transfer.
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure DMA transfer from ADC to RAM using ping-pong */      
  setupDma();
  
  /* Configura ADC Sampling */
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
