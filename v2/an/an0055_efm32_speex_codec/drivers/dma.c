/**************************************************************************//**
 * @file dma.c
 * @brief Setup DMA for ADC, DAC and FLASH write
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_prs.h"
#include "config.h"
#include "dma.h"
#include "timer.h"

/** DMA control block, requires proper alignment. */
#if defined (__ICCARM__)
#pragma location = PRI_ADDRESS
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHANNEL_NUMBER];
#pragma location = ALT_ADDRESS
DMA_DESCRIPTOR_TypeDef dmaControlBlockAlt[DMA_ALT_CHANNEL_NUMBER];
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHANNEL_NUMBER] __attribute__((at(PRI_ADDRESS)));
DMA_DESCRIPTOR_TypeDef dmaControlBlockAlt[DMA_ALT_CHANNEL_NUMBER] __attribute__((at(ALT_ADDRESS)));
#else
#error Undefined toolkit, need to define alignment 
#endif

volatile uint32_t dacPpMode;
/* Variable to decide which descriptor to configure in PP mode DMA */
volatile uint32_t frameCount;

extern volatile int16_t sampleBuffer[][SAMPLE_SIZE];
#if SPEEX_BAND == NB8KCODEC
extern volatile uint8_t encodeFrameBuffer[];
#endif

/**************************************************************************//**
 * @brief DMA Interrupt handler
 *****************************************************************************/
void DMA_IRQHandler(void)
{
  if (DMA->IF & DMA_CHANNEL_DACMASK)
  { 
    DMA->IFC = DMA_CHANNEL_DACMASK;
    if (!(frameCount & 0x01))
    {
      dmaControlBlock[DMA_CHANNEL_DAC].CTRL = dacPpMode;
    }
    else
    {
      dmaControlBlockAlt[DMA_CHANNEL_DAC].CTRL = dacPpMode;
    }
    frameCount++;
  }
#if SPEEX_BAND == NB8KCODEC
  else if (DMA->IF & DMA_CHANNEL_ADCMASK)
  {
    DMA->IFC = DMA_CHANNEL_ADCMASK;
    if (!(frameCount & 0x01))
    {
      dmaControlBlock[DMA_CHANNEL_ADC].CTRL = ADC_PP_START;
    }
    else
    {
      dmaControlBlockAlt[DMA_CHANNEL_ADC].CTRL = ADC_PP_START;
    }
    frameCount++;
  }    
#endif
  else
  { 
    /* Bus error, loop here to enable the debugger to see what has happened */
    while(1)
      ;
  }  
}

/**************************************************************************//**
 * @brief  Initialize DMA
 *****************************************************************************/
void initDma(void)
{ 
  /* Setting description pointer */
  DMA->CTRLBASE = (uint32_t)dmaControlBlock;
  /* Enabling channel */
  DMA->CONFIG = DMA_CONFIG_EN;
  /* Clear/Enabling DMA interrupts */
  NVIC_ClearPendingIRQ(DMA_IRQn);
  NVIC_EnableIRQ(DMA_IRQn);
  /* Enable bus error interrupt */
  DMA->IEN |= DMA_IEN_ERR;
}

#if SPEEX_BAND == NB8KCODEC
/**************************************************************************//**
 * @brief Intializes ADC DMA
 *****************************************************************************/
void initAdcDma(void)
{
  /* Setting up primary DMA from ADC */
  dmaControlBlock[DMA_CHANNEL_ADC].SRCEND = (uint32_t *)&ADC0->SINGLEDATA;
  dmaControlBlock[DMA_CHANNEL_ADC].DSTEND = (uint16_t *)(sampleBuffer[0]) + (SAMPLE_SIZE - 1);
  
  /* Setting up alternate DMA from ADC */
  dmaControlBlockAlt[DMA_CHANNEL_ADC].SRCEND = (uint32_t *)&ADC0->SINGLEDATA;
  dmaControlBlockAlt[DMA_CHANNEL_ADC].DSTEND = (uint16_t *)(sampleBuffer[1]) + (SAMPLE_SIZE - 1);
  
  /* Set ADC request source */
  DMA->CH[DMA_CHANNEL_ADC].CTRL = DMAREQ_ADC0_SINGLE;

  /* Enabling interrupt from DMA */
  DMA->IEN |= DMA_CHANNEL_ADCMASK;  
}

/**************************************************************************//**
 * @brief Start ADC DMA 
 *****************************************************************************/
void startAdcDma(void)
{
  /* Reset parameters */
  frameCount = 0;

  /* Setup (transfer size - 1) and pingpong mode DMA for primary */
  dmaControlBlock[DMA_CHANNEL_ADC].CTRL = ADC_PP_START;

  /* Setup (transfer size - 1) and pingpong mode DMA for alternate */
  dmaControlBlockAlt[DMA_CHANNEL_ADC].CTRL = ADC_PP_START;

  /* Start with primary descriptor */
  DMA->CHALTC = DMA_CHANNEL_DACMASK;
  /* Enable DMA */
  DMA->CHENS = DMA_CHANNEL_ADCMASK;

  /* Select TIMER as source and Timer overflow as signal */
  PRS_SourceSignalSet(ADC_PRS_CH, TIMER_PRS, TIMER_SRC, prsEdgeOff);
  /* Enable PRS trigger */
  ADC0->SINGLECTRL |= ADC_SINGLECTRL_PRSEN;
  
  /* Start timer */
  TIMER_USED->CMD = TIMER_CMD_START;
}

/**************************************************************************//**
 * @brief Stop ADC DMA 
 *****************************************************************************/
void stopAdcDma(void)
{
  /* Stop timer */
  TIMER_USED->CMD = TIMER_CMD_STOP;
  /* Disable DMA */
  DMA->CHENC = DMA_CHANNEL_ADCMASK;
  /* Disable PRS trigger */
  ADC0->SINGLECTRL &= ~ADC_SINGLECTRL_PRSEN;
}

/**************************************************************************//**
 * @brief Intializes Flash DMA
 *****************************************************************************/
void initFlashDma(void)
{
  /* Setting up primary DMA for Flash */
  dmaControlBlock[DMA_CHANNEL_FLASH].SRCEND = (uint32_t *)(encodeFrameBuffer) + (FLASH_WORD_SIZE - 1);
  dmaControlBlock[DMA_CHANNEL_FLASH].DSTEND = (uint32_t *)&MSC->WDATA;

  /* Set Flash request source */
  DMA->CH[DMA_CHANNEL_FLASH].CTRL = DMAREQ_MSC_WDATA;
}

/**************************************************************************//**
 * @brief Start Flash DMA 
 *****************************************************************************/
void startFlashDma(void)
{
  /* Setup transfer size and basic mode DMA */
  dmaControlBlock[DMA_CHANNEL_FLASH].CTRL = FLASH_WR_START;
  /* Load first word into the DMA */
  MSC->WDATA = *((uint32_t *)(encodeFrameBuffer));
  /* Enable DMA */
  DMA->CHENS = DMA_CHANNEL_FLHMASK;
  /* Trigger the transfer */
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;
}
#endif

/**************************************************************************//**
 * @brief Intializes DAC DMA
 *****************************************************************************/
void initDacDma(void)
{
  /* Setting up primary DMA for DAC */
  dmaControlBlock[DMA_CHANNEL_DAC].DSTEND = (uint32_t *)&DAC0->DAC_CHDATA;
  
  /* Setting up alternate DMA for DAC */
  dmaControlBlockAlt[DMA_CHANNEL_DAC].DSTEND = (uint32_t *)&DAC0->DAC_CHDATA;
  
  /* Set DAC request source */
  DMA->CH[DMA_CHANNEL_DAC].CTRL = DAC_CHDMA;

  /* Enabling interrupt from DMA */
  DMA->IEN |= DMA_CHANNEL_DACMASK;  
}

/**************************************************************************//**
 * @brief Start DAC DMA 
 * @param bufferSize - Buffer size of DAC DMA buffer
 *****************************************************************************/
void startDacDma(uint16_t bufferSize)
{
  /* Reset parameters */
  frameCount = 0;
  dacPpMode = SRC_INC16_DST_FIX + ((bufferSize - 1) << _DMA_CTRL_N_MINUS_1_SHIFT) + DMA_CTRL_CYCLE_CTRL_PINGPONG;

  /* Setup (transfer size - 1) and pingpong mode DMA for primary */
  dmaControlBlock[DMA_CHANNEL_DAC].CTRL = dacPpMode;
  dmaControlBlock[DMA_CHANNEL_DAC].SRCEND = (uint16_t *)(sampleBuffer[0]) + (bufferSize - 1);
  
  /* Setup (transfer size - 1) and pingpong mode DMA for alternate */
  dmaControlBlockAlt[DMA_CHANNEL_DAC].CTRL = dacPpMode;
  dmaControlBlockAlt[DMA_CHANNEL_DAC].SRCEND = (uint16_t *)(sampleBuffer[1]) + (bufferSize - 1);

  /* Start with primary descriptor */
  DMA->CHALTC = DMA_CHANNEL_DACMASK;
  /* Enable DMA */
  DMA->CHENS = DMA_CHANNEL_DACMASK;
  
  /* Enable DAC channel with PRS */
  DAC0->DAC_CHCTRL |= (DAC_CHCTRLEN + DAC_CHPRSEN);

  /* Setup TIMER TOP value for DAC */
  setTopValue(bufferSize);
}

/**************************************************************************//**
 * @brief Stop DAC DMA 
 *****************************************************************************/
void stopDacDma(void)
{
  /* Disable DAC channel and stop timer */
  TIMER_USED->CMD = TIMER_CMD_STOP;
  DAC0->DAC_CHCTRL &= ~(DAC_CHCTRLEN + DAC_CHPRSEN);
  /* Disable DMA */
  DMA->CHENC = DMA_CHANNEL_DACMASK;
}
