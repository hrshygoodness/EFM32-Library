/**************************************************************************//**
 * @file use_flashloader.c
 * @brief Handles programming with help of a flashloader
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
#include <stdbool.h>
#include <stdio.h>
#include "em_device.h"
#include "dap.h"
#include "flashloader.h"
#include "utils.h"
#include "use_flashloader.h"
#include "errors.h"
#include "delay.h"


#define RAM_START 0x20000000

/* Initializes a global flashLoaderState object 
 * which is placed at the same location in memory 
 * as in the flashloader itself. Using this
 * we can easily get the address of each of the fields */
flashLoaderState_TypeDef *flState = (flashLoaderState_TypeDef *)STATE_LOCATION;



/**********************************************************
 * Uploads and runs the flashloader on the target
 * The flashloader is written directly to the start
 * of RAM. Then the PC and SP are loaded from the
 * flashloader image. 
 **********************************************************/
void uploadFlashloader(uint32_t *flImage, uint32_t size)
{
  int w;
  uint32_t addr;
  uint32_t tarWrap;
  
  uint32_t numWords = size / 4;
  if ( numWords * 4 < size ) numWords++;
   
  resetAndHaltTarget();
  
  /* Get the TAR wrap-around period */
  tarWrap = getTarWrap();
  
  printf("Uploading flashloader\n");
  
  /* Enable autoincrement on TAR */
  writeAP(AP_CSW, AP_CSW_DEFAULT | AP_CSW_AUTO_INCREMENT);
  
  for ( w=0; w<numWords; w++ ) 
  {
    /* Get address of current word */
    addr = RAM_START + w * 4;
    
    /* At the TAR wrap boundary we need to reinitialize TAR
     * since the autoincrement wraps at these */
    if ( (addr & tarWrap) == 0 )
    {
      writeAP(AP_TAR, addr);
    }
    
    writeAP(AP_DRW, flImage[w]);
  }
  
  writeAP(AP_CSW, AP_CSW_DEFAULT);
  
  printf("Booting flashloader\n");
  
  /* Load SP (Reg 13) from flashloader image */
  writeCpuReg(13, flImage[0]);
  
  /* Load PC (Reg 15) from flashloader image */
  writeCpuReg(15, flImage[1]);
    
  runTarget();
}


/**********************************************************
 * Verifies that the flashloader is ready. Will throw an
 * exception if the flashloader is not ready within a
 * timeout. 
 **********************************************************/
void verifyFlashloaderReady(void)
{
  uint32_t status;
  
  uint32_t timeout = FLASHLOADER_RETRY_COUNT;
  
  do {
    status = readMem( (uint32_t)&(flState->flashLoaderStatus) );
    timeout--;
  } while ( status == FLASHLOADER_STATUS_NOT_READY  && timeout > 0 );
      
  if ( status == FLASHLOADER_STATUS_READY ) {
    printf("Flashloader ready\n");
  } else {
    printf("Flashloader not ready. Status: 0x%.8x\n", status);
    RAISE(SWD_ERROR_FLASHLOADER_ERROR);
  }
}

/**********************************************************
 * Waits until the flashloader reports that it is ready
 **********************************************************/
void waitForFlashloader(void)
{
  uint32_t status;
  int retry = FLASHLOADER_RETRY_COUNT;
  
  /* Wait until flashloader has acknowledged the command */
  do {
    status = readMem((uint32_t)&(flState->debuggerStatus));
    retry--;
  } while ( status != DEBUGGERCMD_NONE && retry > 0 );
  
  /* Wait until command has completed */
  retry = FLASHLOADER_RETRY_COUNT;
  do {
    delayUs(500);
    status = readMem((uint32_t)&(flState->flashLoaderStatus));
    retry--;
  } while ( status == FLASHLOADER_STATUS_NOT_READY && retry > 0 );
  
  /* Raise an error if we timed out or flashloader has reported an error */
  if ( status == FLASHLOADER_STATUS_NOT_READY ) 
  {
    printf("Timed out while waiting for flashloader\n");
    RAISE(SWD_ERROR_FLASHLOADER_ERROR);
  } 
  else if ( status != FLASHLOADER_STATUS_READY ) 
  {
    printf("Flashloader returned error code %d\n", status);
    RAISE(SWD_ERROR_FLASHLOADER_ERROR);
  }
}

/**********************************************************
 * Tells the flashloader to erase one page at the 
 * given address.
 **********************************************************/
void sendErasePageCmd(uint32_t addr)
{
  writeMem((uint32_t)&(flState->writeAddress1), addr);
  writeMem((uint32_t)&(flState->numBytes1), 1);
  writeMem((uint32_t)&(flState->debuggerStatus), DEBUGGERCMD_ERASE_PAGE);
  
  waitForFlashloader();
}

/**********************************************************
 * Tells the flashloader to erase the entire flash
 **********************************************************/
void sendMassEraseCmd(void)
{
  writeMem((uint32_t)&(flState->debuggerStatus), DEBUGGERCMD_MASS_ERASE);
  
  waitForFlashloader();
}


/**********************************************************
 * Writes a chunk of data to a buffer in the flashloader.
 * This function does not make any checks or assumptions.
 * It simply copies a number of words from the 
 * local to the remote buffer.
 * 
 * @param remoteAddr
 *    Address of the flashloader buffer at the target
 * 
 * @param localBuffer
 *    The local buffer to write from
 * 
 * @param numWords
 *    Number of words to write to buffer
 **********************************************************/
void writeToFlashloaderBuffer(uint32_t remoteAddr, uint32_t *localBuffer, int numWords)
{
    uint32_t bufferPointer = (uint32_t)remoteAddr;
    int curWord = 0;
    uint32_t tarWrap;

    /* Get the TAR wrap-around period */
    tarWrap = getTarWrap();
    
    /* Set auto increment on TAR to allow faster writes */
    writeAP(AP_CSW, AP_CSW_DEFAULT | AP_CSW_AUTO_INCREMENT);
    
    /* Initialize TAR with the start of buffer */
    writeAP(AP_TAR, bufferPointer);
    
    /* Send up to one full buffer of data */    
    while ( curWord < numWords )
    {
      /* At TAR wrap boundary we need to reinitialize TAR
       * since the autoincrement wraps at these */
      if ( (bufferPointer & tarWrap) == 0 )
      {
        writeAP(AP_TAR, bufferPointer);
      }
      
      /* Write one word */
      writeAP(AP_DRW, localBuffer[curWord]);
      
      /* Increment local and remote pointers */
      bufferPointer += 4;
      curWord += 1;
    }
            
    /* Disable auto increment on TAR */
    writeAP(AP_CSW, AP_CSW_DEFAULT);
}



/**********************************************************
 * Uploads a binary image (the firmware) to the flashloader.
 **********************************************************/
void uploadImageToFlashloader(uint32_t *fwImage, uint32_t size)
{   
  uint32_t numWords = size / 4;
  uint32_t curWord = 0;
  uint32_t writeAddress = 0;
  bool useBuffer1 = true;
  
  /* Get the buffer location (where to temporary store data 
   * in target SRAM) from flashloader */
  uint32_t bufferLocation1 = readMem( (uint32_t)&(flState->bufferAddress1) );
  uint32_t bufferLocation2 = readMem( (uint32_t)&(flState->bufferAddress2) );
  
  /* Get size of target buffer */
  uint32_t bufferSize = readMem( (uint32_t)&(flState->bufferSize) );
  
  /* Round up to nearest word */
  if ( numWords * 4 < size ) numWords++;
    
  
  /* Fill the buffer in RAM and tell the flashloader to write
   * this buffer to Flash. Since we are using two buffers
   * we can fill one buffer while the flashloader is using
   * the other. 
   */  
  while ( curWord < numWords ) 
  {
    /* Calculate the number of words to write */
    int wordsToWrite = numWords - curWord < bufferSize / 4 ? numWords - curWord : bufferSize / 4;
        
    /* Write one chunk to the currently active buffer */
    writeToFlashloaderBuffer(
           useBuffer1 ? bufferLocation1 : bufferLocation2,
           &fwImage[curWord],
           wordsToWrite
    );
    
    /* Wait until flashloader is done writing to flash */
    waitForFlashloader();
   
    
    /* Tell the flashloader to write the data to flash */
    if ( useBuffer1 ) 
    {      
      writeMem( (uint32_t)&(flState->numBytes1), wordsToWrite * 4);
      writeMem( (uint32_t)&(flState->writeAddress1), writeAddress);
      writeMem( (uint32_t)&(flState->debuggerStatus), DEBUGGERCMD_WRITE_DATA1 );
    } 
    else
    {
      writeMem( (uint32_t)&(flState->numBytes2),  wordsToWrite * 4);
      writeMem( (uint32_t)&(flState->writeAddress2), writeAddress);
      writeMem( (uint32_t)&(flState->debuggerStatus), DEBUGGERCMD_WRITE_DATA2 );      
    }
      
    /* Increase address */
    curWord += wordsToWrite;
    writeAddress += wordsToWrite * 4;
    
    /* Flip buffers */
    useBuffer1 = !useBuffer1;
  }
  
  /* Wait until the last flash write operation has completed */
  waitForFlashloader();
  
}

void erasePagesWithFlashloader(int size)
{
  /* Get page size from flashloader */
  uint32_t pageSize = readMem( (uint32_t)&(flState->pageSize) );
  
  /* Calculate number of pages needed to store image */
  uint32_t usedPages = size / pageSize;
  if ( usedPages * pageSize < size ) usedPages++;  
  
  printf("Erasing %d page(s)\n", usedPages);
  
  /* Tell flashloader to erase all pages we are going to write to */
  int addr = 0;
  while ( addr < size ) 
  {
    sendErasePageCmd(addr);
    addr += pageSize;
  }
}


/**********************************************************
 * Uses the flashloader to upload an application to 
 * the target. This function will first upload the 
 * flashloader, then communicate with the flashloader
 * to send the program. If successful the application
 * will be started when this function returns. 
 * 
 * @param verify
 *    If true, verify the application after it has been 
 *    written to flash
 **********************************************************/
void flashWithFlashloader(bool verify)
{
  uint32_t startTime = DWT->CYCCNT;
  uint32_t time;
  
#pragma section="fw_section"  
  int fwSize = __section_size("fw_section");
  
  printf("Preparing to upload firmware (%d bytes)\n", fwSize);
 
  /* Upload flashloader */
#pragma section="flashloader_section"  
  uploadFlashloader((uint32_t *)__section_begin("flashloader_section"), __section_size("flashloader_section")); 
  
  /* Check that flashloader is ready */
  verifyFlashloaderReady();
 


#if defined(SWD_FAST)
  
  /* Doing one Mass Erase is faster than erasing individual pages,
   * but it will case wear on all pages.
   * Only available on GG, LG, WG and ZG devices. */
  
  uint32_t part = readMem((uint32_t)&(DEVINFO->PART));
  uint32_t family = (part &  _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;
  
  if ( family == _DEVINFO_PART_DEVICE_FAMILY_GG ||
       family == _DEVINFO_PART_DEVICE_FAMILY_LG ||
       family == _DEVINFO_PART_DEVICE_FAMILY_WG ||
       family == _DEVINFO_PART_DEVICE_FAMILY_ZG )
  {
    printf("Erasing entire flash\n");
    sendMassEraseCmd();      
  }
  else
  {
    erasePagesWithFlashloader(fwSize);
  }
#else 
  erasePagesWithFlashloader(fwSize);
#endif
      
  printf("Uploading firmware image\n");

  uploadImageToFlashloader(__section_begin("fw_section"), fwSize);
    
  /* Verify that the uploaded image is equal to the original */
  if ( verify ) 
  {
    if ( verifyFirmware((uint32_t *)__section_begin("fw_section"), fwSize) ) {
      printf("Application verified\n");
    } else {
      RAISE(SWD_ERROR_VERIFY_FW_FAILED);
    }
  }
  
  /* Calculate the total time spent flashing */
  time = DWT->CYCCNT - startTime;
  printf("Total time: %f s (%f kB/s)\n", (time/48e6), ((float)fwSize/1024) / (time/48e6));
  
  printf("Starting application\n");
  
  /* Reset target to start application */
  resetTarget();
}
  
  