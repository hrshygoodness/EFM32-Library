/**************************************************************************//**
 * @file flash.c
 * @brief Functions for writing and clearing flash
 * @author Silicon Labs
 * @version 1.03
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
#include "em_msc.h"
#include "em_dma.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "flash.h"
#include "bootloader_config.h"

/* Align DMA descriptor properly */
#if ( ( DMA_CHAN_COUNT > 4 ) && ( DMA_CHAN_COUNT <= 8 ) )
#define DMACTRL_ALIGNMENT   256
#elif ( ( DMA_CHAN_COUNT > 8 ) && ( DMA_CHAN_COUNT <= 16 ) )
#define DMACTRL_ALIGNMENT   512
#else
#error "Unsupported DMA channel count"
#endif

#if defined (__ICCARM__)
#pragma data_alignment=DMACTRL_ALIGNMENT
DMA_DESCRIPTOR_TypeDef descr;
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef descr __attribute__ ((aligned(DMACTRL_ALIGNMENT)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef descr __attribute__ ((aligned(DMACTRL_ALIGNMENT)));
#else
#error Undefined toolkit, need to define alignment
#endif


/* These macros are defined here instead of using the NVIC functions 
 * in order to execute them from RAM. */
#define ENABLE_IRQ(IRQn)          NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F))
#define DISABLE_IRQ(IRQn)         NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F))
#define CLEARPENDING_IRQ(IRQn)    NVIC->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F))


/* Flag indicating whether an erase operation is currently active */
static volatile bool eraseActive = false;

/* Buffer to hold one flash page of data. Used when copying
 * data from one page to another with DMA */
static uint32_t pageBuffer[WORDS_PER_PAGE];


/******************************************************************************
 * MSC interrupt handler.
 *****************************************************************************/
RAMFUNC void MSC_IRQHandler(void)
{
  uint32_t flags = MSC->IF; 
  
  /* Clear flags */
  MSC->IFC = flags;
  
  if ( flags & MSC_IF_ERASE ) {
    eraseActive = false;
  } 
}

/******************************************************************************
 * DMA IRQ Handler. Only used to wake up the MCU when DMA transfer is complete. 
 *****************************************************************************/
RAMFUNC void DMA_IRQHandler(void)
{
  /* Clear interrupt flags */
  DMA->IFC = DMA->IF;
}


/******************************************************************************
 * Initialize the MSC controller and DMA. This is needed before we can write
 * anything to flash. 
 *****************************************************************************/
void FLASH_init(void)
{
  /* Enable clock to DMA */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_DMA;
  
  /* Write MSC unlock code to enable interface */
  MSC->LOCK = MSC_UNLOCK_CODE;
  
  /* Enable memory controller */
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
  
  /* Enable DMA */
  DMA->CONFIG = DMA_CONFIG_EN;
  
  /* Setup the DMA control block. */
  DMA->CTRLBASE = (uint32_t) &descr;
}

/******************************************************************************
 * Writes a number of words to flash. This function sets up and starts
 * a DMA transfer and then returns immediately. DMA will continue in 
 * background until all the words have been written. If a DMA transfer
 * is already active when this function is called, it will stall 
 * until the transfer is complete. 
 * 
 * @param addr
 *    The address to start writing to. 
 * 
 * @param src
 *    Source address for the words to be written
 * 
 * @param numWords
 *    The number of words to write
 *****************************************************************************/
RAMFUNC void FLASH_write(uint32_t *addr, uint32_t *src, uint32_t numWords)
{
  /* Wait until previous transfer is done */
  while (DMA->CHENS & DMA_CHENS_CH0ENS);
  
  MSC->ADDRB = (uint32_t)addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  
  /* Set the MSC as the destination. */
  descr.DSTEND = (void *)(&(MSC->WDATA));

  /* Set up the end pointer to copy from the buffer. */
  descr.SRCEND = (void *)(src + numWords - 1);

  descr.CTRL = DMA_CTRL_DST_INC_NONE       /* Do not increment destination */
               | DMA_CTRL_DST_SIZE_WORD    /* Transfer whole words at a time */
               | DMA_CTRL_SRC_INC_WORD     /* Write one word at the time */
               | DMA_CTRL_SRC_SIZE_WORD    /* Transfer whole words at a time */
               | DMA_CTRL_R_POWER_1
               | DMA_CTRL_CYCLE_CTRL_BASIC /* Basic transfer */
                                           /* Number of transfers minus two. */
                                           /* This field contains the number of transfers minus 1. */
                                           /* Because one word is transerred using WRITETRIG we need to */
                                           /* Substract one more. */
               | ((numWords - 2) << _DMA_CTRL_N_MINUS_1_SHIFT);

  /* Set channel to trigger on MSC ready for data */
  DMA->CH[0].CTRL = DMA_CH_CTRL_SOURCESEL_MSC
                    | DMA_CH_CTRL_SIGSEL_MSCWDATA;
 

  /* Load first word into the DMA */
  MSC->WDATA = *src;

  /* Activate channel 0 */
  DMA->CHENS = DMA_CHENS_CH0ENS;
  
  /* Start the transfer */
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;
}



/******************************************************************************
 * Erases a page of flash
 * 
 * @param pageStart
 *    The start address of the page
 *****************************************************************************/
RAMFUNC void FLASH_erasePage(uint32_t pageStart)
{
  uint32_t acc = 0xFFFFFFFF;
  uint32_t *ptr;
  
  /* Optimization - check if block is allready erased.
   * This will typically happen when the chip is new. */
  for (ptr = (uint32_t *)pageStart; ptr < (uint32_t *)(pageStart + FLASH_PAGE_SIZE); ptr++) {
    acc &= *ptr;
  }

  /* If the accumulator is unchanged, there is no need to do an erase. */
  if (acc == 0xFFFFFFFF)
    return;

  /* Load address */
  MSC->ADDRB    = pageStart;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  
  /* Enable interrupt when erase is complete */
  MSC->IEN = MSC_IEN_ERASE;
  CLEARPENDING_IRQ(MSC_IRQn);
  ENABLE_IRQ(MSC_IRQn);
  
  /* Set flag to indicate that erase is ongoing */
  eraseActive = true;

  /* Send Erase Page command */
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;
  
  /* Waiting for erase to complete. Interrupt will wake
   * up CPU when erase is complete. */
  while (eraseActive) {
     
    /* Enter EM1 */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
  }
  
  /* Disable interrupts again */
  MSC->IEN = 0;
  DISABLE_IRQ(MSC_IRQn);
}



/******************************************************************************
 * Writes one word to flash. Does not return before the write is complete.
 * 
 * @param addr
 *      Address to write to
 * 
 * @param data
 *      Word to write
 *****************************************************************************/
RAMFUNC void FLASH_writeWord(uint32_t *addr, uint32_t data)
{
  /* Check for an active transfer. If a write is in progress,
   * we have to delay. */  
  while (MSC->STATUS & MSC_STATUS_BUSY);

  /* Load address */
  MSC->ADDRB    = (uint32_t)addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Load data */
  MSC->WDATA = data;

  /* Trigger write once */
  MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

  /* Waiting for the write to complete */
  while (MSC->STATUS & MSC_STATUS_BUSY) ;
}


/******************************************************************************
 * Copies a firmware image from the temporary storage to the boot region.
 *****************************************************************************/
RAMFUNC void copyFirmwareFromTempStorage(void)
{
  uint32_t i;
  
  uint32_t *source = (uint32_t *)TEMP_START_ADDRESS;
  uint32_t *dest = (uint32_t *)FIRMWARE_START_ADDRESS;
  FirmwareHeader *tempHeader = (FirmwareHeader *)TEMP_START_ADDRESS;
  
  uint32_t pageCounter = 0; 
  
  /* Get number of words to write. fwHeader->size is in bytes */
  uint32_t wordsLeft = FIRMWARE_HEADER_SIZE + tempHeader->size / 4;
  
  uint32_t wordsToWrite;
  
  /* Enable interrupt when write is complete */
  DMA->IEN |= DMA_IEN_CH0DONE;
  ENABLE_IRQ(DMA_IRQn);
  
  /* Copy entire image to boot location. One page at a time. */
  while ( wordsLeft ) 
  {
    /* Calculate the number of words to write to this page */
    wordsToWrite = wordsLeft > WORDS_PER_PAGE ? WORDS_PER_PAGE : wordsLeft;
    
    /* Copy page to RAM so DMA can read from it while
     * writing to flash */
    for ( i=0; i<wordsToWrite; i++ ) {
      pageBuffer[i] = source[i];
    }
    
    /* Handle special case when writing the header. The 
     * verified field must be initially set to FW_NOT_VERIFIED */
    if ( pageCounter == 0 ) {
      FirmwareHeader *h = (FirmwareHeader *)pageBuffer;
      h->verified = FW_NOT_VERIFIED;
    }
    
    /* Erase the page */
    FLASH_erasePage((uint32_t)dest);
    
    /* Write up to one page of data */
    FLASH_write(dest, pageBuffer, wordsToWrite);
    
    /* Wait until DMA transfer is done */
    while (DMA->CHENS & DMA_CHENS_CH0ENS) {
      
      /* Enter EM1. Interrupt will wake up when write is complete. */
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
      __WFI();
    }
    
    source += wordsToWrite;
    dest += wordsToWrite;
    wordsLeft -= wordsToWrite;
    pageCounter++;
  }
  
  /* Disable MSC interrupts */
  DMA->IEN &= ~DMA_IEN_CH0DONE;
  DISABLE_IRQ(DMA_IRQn);
  
}
