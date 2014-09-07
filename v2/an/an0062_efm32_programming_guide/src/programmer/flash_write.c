/*******************************************************************************
 * @file flash_write.c
 * @brief Functions to write directly to flash over SWD
 * @author Silicon Labs
 * @version 1.03
 *******************************************************************************
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
#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "dap.h"
#include "errors.h"
#include "flash_write.h"
#include "utils.h"
#include "delay.h"


/**********************************************************
 * Erases enough pages of flash to fit a image of the 
 * given size. The size of flash pages is retrieved
 * from the target. The pages are erased by 
 * writing directly to the MSC registers 
 * 
 * @param size
 *    Size in bytes of firmware
 **********************************************************/
void erasePages(uint32_t size)
{
  uint32_t addr = 0;
  uint32_t mscStatus;
  uint32_t timeOut;
  
  /* Enable write in MSC */
  writeMem((uint32_t)&(MSC->WRITECTRL), MSC_WRITECTRL_WREN);
  
  /* Get the size of one flash page */
  uint32_t pageSize = getPageSize();
    
  /* Loop until enough pages have been erased */
  while ( addr < size ) 
  {
    /* Enter the start address of the page */
    writeMem((uint32_t)&(MSC->ADDRB), addr);
    
    /* Load address into internal register */
    writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_LADDRIM);
    
    /* Start page erase operation */
    writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_ERASEPAGE);
    
    /* Wait until erase is complete */
    timeOut = MSC_TIMEOUT;
    do {
      delayMs(1);
      mscStatus = readMem((uint32_t)&(MSC->STATUS));
      timeOut--;
    } while ( (mscStatus & MSC_STATUS_BUSY) && timeOut > 0 );
    
    if ( mscStatus & MSC_STATUS_BUSY ) {
      printf("Timed out while waiting for erase to complete\n");
      RAISE(SWD_ERROR_FLASH_WRITE_FAILED);
    }
    
    /* Move to next page */
    addr += pageSize;
  }
}


/**********************************************************
 * Writes a firmware image to internal flash of the
 * target using direct writes to the MSC registers.
 * Writing starts at address 0x00 at the target
 * and data is copied from the supplied buffer
 * 
 * @param fwImage
 *    Pointer to firmware image
 *    
 * @param size
 *    Size in bytes of firmware image
 **********************************************************/
static void uploadImage(uint32_t *fwImage, uint32_t size)
{
  int w;
  uint32_t timeOut;
  uint32_t mscStatus;
  
  /* Calculate the number of words to write */
  uint32_t numWords = size / 4;
  if ( numWords * 4 < size ) numWords++;
  
  printf("Uploading firmware\n");
    
  /* Start writing all words from the given buffer. 
   * Writing starts at address 0x00 on the target */
  for ( w=0; w<numWords; w++ ) 
  {
    /* Enter address */
    writeMem((uint32_t)&(MSC->ADDRB), w * 4);
    
    /* Load address into internal register */
    writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_LADDRIM);
    
    /* Write value to WDATA */
    writeMem((uint32_t)&(MSC->WDATA), fwImage[w]);
    
    /* Start flash write */
    writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_WRITEONCE);
    
    /* Wait for write to complete */
    timeOut = MSC_TIMEOUT;
    do {
      mscStatus = readMem((uint32_t)&(MSC->STATUS));
      timeOut--;
    } while ( (mscStatus & MSC_STATUS_BUSY) && timeOut > 0 );
    
    if ( mscStatus & MSC_STATUS_BUSY ) {
      RAISE(SWD_ERROR_FLASH_WRITE_FAILED);
    }
  }
}


/**********************************************************
 * Writes one word to flash 
 * 
 * @param addr The address to write to
 * 
 * @param data The value to write
 **********************************************************/
void writeWordToFlash(uint32_t addr, uint32_t data)
{
  uint32_t timeOut = MSC_TIMEOUT;
  uint32_t mscStatus;
  
  /* Enable write in MSC */
  writeMem((uint32_t)&(MSC->WRITECTRL), MSC_WRITECTRL_WREN);
  
  /* Load address */
  writeMem((uint32_t)&(MSC->ADDRB), addr);
  
  /* Load address into internal register */
  writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_LADDRIM);
  
  /* Write word to WDATA */
  writeMem((uint32_t)&(MSC->WDATA), data);
  
  /* Start flash write */
  writeMem((uint32_t)&(MSC->WRITECMD), MSC_WRITECMD_WRITEONCE);
  
  /* Wait for write to complete */
  do {
    mscStatus = readMem((uint32_t)&(MSC->STATUS));
    timeOut--;
  } while ( (mscStatus & MSC_STATUS_BUSY) && timeOut > 0 );
  
  if ( mscStatus & MSC_STATUS_BUSY ) {
    RAISE(SWD_ERROR_FLASH_WRITE_FAILED);
  }
}


/**********************************************************
 * Uses direct writes to MSC registers in order to 
 * program the target.
 **********************************************************/
void flashWithMsc(void)
{  
  uint32_t startTime = DWT->CYCCNT;
  uint32_t time;
  
#pragma section="fw_section"  
  int fwSize = __section_size("fw_section");
  
  printf("Preparing to upload firmware (%d bytes)\n", fwSize);
    
  haltTarget();
  
  printf("Target halted\n");
  
  /* Erase all pages that we are going to write to */
  printf("Erasing pages\n");
  erasePages(fwSize);
  
  /* Write firmware image to flash */
  uploadImage(__section_begin("fw_section"), fwSize);
    
  /* Verify application */
  if ( !verifyFirmware(__section_begin("fw_section"), fwSize) ) {
    return;
  }
  
  /* Calculate the total time spent */
  time = DWT->CYCCNT - startTime;
  printf("Total time: %f s (%f kB/s)\n", (time/48e6), ((float)fwSize/1024) / (time/48e6));
  
  printf("Starting application\n");
  
  /* Reset target */  
  resetTarget();
}
