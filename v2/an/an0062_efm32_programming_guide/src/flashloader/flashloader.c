/**************************************************************************//**
 * @file flashloader.c
 * @brief Universal flashloader for EFM32 devices
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
#include <stdint.h>
#include "em_device.h"

#include "em_msc.h"
#include "flashloader.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Place the state struct in at a defined address. 
 * This struct is used to communicate with the 
 * programmer.
 */
#pragma location=STATE_LOCATION
__no_init volatile flashLoaderState_TypeDef state;


/* This is the buffer to use for writing data to/from.
   It is dynamically resized based on available RAM.
   Therefore it must be placed last by the linker. */
#pragma location="buffer"
#pragma data_alignment=4
__no_init uint8_t flashBuffer[1];

/* On devices that supports double word writes this flag
 * will be set to true.  */
bool writeDouble = false;




/**********************************************************
 * HardFault handler. Sets the status field to tell
 * programmer that we have encountered a HardFault and
 * enter an endless loop 
 **********************************************************/
static void HardFaultHandler(void)
{
  state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_HARDFAULT;
  while(1);
}

/**********************************************************
 * NMI handler. Sets the status field to tell
 * programmer that we have encountered a NMI and
 * enter an endless loop 
 **********************************************************/
static void NmiHandler(void)
{
  state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_NMI;
  while(1);
}

/**********************************************************
 * Handles errors from MSC. Sets the status field to tell
 * programmer which error occured and then
 * enter an endless loop 
 * 
 * @param ret
 *    MSC error code
 **********************************************************/
static void handleMscError(msc_Return_TypeDef ret)
{
  if (ret != mscReturnOk)
  {
    /* Generate error code. */
    switch(ret)
    {
    case (mscReturnTimeOut):      
      state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_TIMEOUT;
      break;
    case (mscReturnLocked):
      state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_LOCKED;
      break;
    case (mscReturnInvalidAddr):
      state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_INVALIDADDR;
      break;
    default:
      state.flashLoaderStatus = FLASHLOADER_STATUS_ERROR_UNKNOWN;
      break;
    }
    while(1) ;
  }
}


/**********************************************************
 * Initalizes the state struct with values from the DI
 * page on current device. This function initializes
 *   - SRAM size
 *   - Flash Size
 *   - Production Revision
 *   - Device Family
 *   - Page Size
 * 
 **********************************************************/
static void setupEFM32(void)
{
  writeDouble = false;

  /* Automatically detect device characteristics */
  state.sramSize = (DEVINFO->MSIZE & _DEVINFO_MSIZE_SRAM_MASK) >> _DEVINFO_MSIZE_SRAM_SHIFT;
  state.flashSize = (DEVINFO->MSIZE & _DEVINFO_MSIZE_FLASH_MASK) >> _DEVINFO_MSIZE_FLASH_SHIFT;
  state.prodRev = (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;
  state.partFamily = (DEVINFO->PART & _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;

  /* Early samples had sram and flash size swapped */
  uint32_t flashSize = state.flashSize;
  if (state.sramSize > flashSize)
  {
    state.flashSize = state.sramSize;
    state.sramSize = flashSize;
  }

  /* Check for unprogrammed flash. */
  if (state.sramSize == 0xFFFFFFFF)
  {
    state.sramSize = 1;
  }

  /* Calculate it in bytes */
  state.sramSize *= 1024;

  /* Figure out the size of the flash pages. */
  switch(state.partFamily)
  {
  case _DEVINFO_PART_DEVICE_FAMILY_GG:          /* Giant Gecko   */
    if (state.prodRev < 13)
    {
      /* All Giant Gecko rev B, with prod rev. < 13 use 2048 as page size, not 4096 */
      state.pageSize = 2048;
    }
    else 
    {
      state.pageSize = 4096;
      writeDouble     = true;
    }
    break;
    
  case _DEVINFO_PART_DEVICE_FAMILY_LG:          /* Leopard Gecko */
  case _DEVINFO_PART_DEVICE_FAMILY_WG:          /* Wonder Gecko  */
    state.pageSize = 2048;
    break;

  case _DEVINFO_PART_DEVICE_FAMILY_G:
    /* Gecko parts */
    /* Early samples have invalid SRAM size in device information page, default to 2K) */
    if (state.prodRev < 4)
    {
      state.sramSize = 2048;
    }
    /* Same page size as tiny */
  case _DEVINFO_PART_DEVICE_FAMILY_TG:
    state.pageSize = 512;
    break;
    
  case _DEVINFO_PART_DEVICE_FAMILY_ZG:
    state.pageSize = 1024;
    break;

  default:
    /* Uncalibrated and/or untested */
    state.sramSize = 2048;
    state.flashSize = 1025;
    state.pageSize = 512;
    break;
  }
}


/**********************************************************
 * Waits on the MSC_STATUS register until the selected
 * bits are set or cleared. This function will busy wait
 * until (MSC_STATUS & mask) == value. 
 * Errors are also handled by this function. Errors
 * will cause the flashloader to set the status
 * flag and stop execution by entering a busy loop. 
 * 
 * @param mask
 *    The mask to apply to MSC_STATUS
 * 
 * @param value
 *    The value to compare against, after applying the mask
 * 
 **********************************************************/
static void mscStatusWait( uint32_t mask, uint32_t value )
{
  uint32_t status;
  int timeOut = MSC_PROGRAM_TIMEOUT;

  while (1)
  {
    status = MSC->STATUS;

    /* Check for errors */
    if ( status & ( MSC_STATUS_LOCKED | MSC_STATUS_INVADDR ) )
    {
      /* Disable write access */
      MSC->WRITECTRL &= ~(MSC_WRITECTRL_WREN | MSC_WRITECTRL_WDOUBLE);
      MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

      /* Set error flag and enter busy loop */
      if ( status & MSC_STATUS_LOCKED )
      {
        handleMscError( mscReturnLocked );
      }

      /* Set error flag and enter busy loop */
      if ( status & MSC_STATUS_INVADDR ) 
      {
        handleMscError( mscReturnInvalidAddr );
      }
    }
    
    /* We only care about WORDTIMEOUT when waiting for WDATAREADY */
    if ( (status & MSC_STATUS_WORDTIMEOUT) && (mask == MSC_STATUS_WDATAREADY) )
    {
      handleMscError( mscReturnTimeOut );
    }
    
    /* Check end condition */
    if ( ( status & mask ) == value )
    {
      /* We are done waiting */
      break;
    }

    timeOut--;
    if ( timeOut == 0 )
    {
      /* Timeout occured. Set flag and enter busy loop */
      handleMscError( mscReturnTimeOut );
    }
  }
}

/**********************************************************
 * Execute a flash command (erase/write etc). Will busy 
 * wait until command is complete. 
 *
 * @param cmd
 *    Flash command (value to write to MSC_CMD)
 **********************************************************/
static void doFlashCmd( uint32_t cmd )
{
  MSC->WRITECMD = cmd;
  mscStatusWait( MSC_STATUS_BUSY, 0 );
}

/**********************************************************
 * Writes one word to flash.
 *
 * @param addr
 *    Address to write to. Must be a valid flash address.
 * 
 * @param data
 *    Word to write
 **********************************************************/
static void pgmWord( uint32_t addr, uint32_t data )
{
  MSC->ADDRB    = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  MSC->WDATA    = data;
  doFlashCmd( MSC_WRITECMD_WRITEONCE );
}


/**********************************************************
 * Erases on page of flash. 
 *
 * @param addr
 *    Address of page. Must be a valid flash address.
 * 
 * @param pagesize
 *    Size of one page in bytes
 **********************************************************/
static void eraseSector( uint32_t addr, uint32_t pagesize )
{
  uint32_t *p = (uint32_t*)addr, result;

  /* Check if page already is erased. If so we can
   * simply return. */
  do
  {
    result    = *p++;
    pagesize -= 4;
  } while ( pagesize && ( result == 0xFFFFFFFF ) );

  if ( result != 0xFFFFFFFF )
  {
    /* Erase the page */
    MSC->ADDRB    = addr;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
    doFlashCmd( MSC_WRITECMD_ERASEPAGE );
  }
}

/**********************************************************
 * Writes multiple words to flash
 *
 * @param addr
 *    Where to start writing. Must be a valid flash address.
 * 
 * @param p
 *    Pointer to data
 * 
 * @param cnt
 *    Number of bytes to write. Must be a multiple
 *    of four.
 **********************************************************/
static void pgmBurst( uint32_t addr, uint32_t *p, uint32_t cnt )
{
  /* Wait until MSC is ready */
  mscStatusWait( MSC_STATUS_BUSY, 0 );
  
  /* Enter start address */
  MSC->ADDRB    = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  
  /* Write first word. Address will be automatically incremented. */
  MSC->WDATA    = *p++;
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;
  cnt          -= 4;

  /* Loop until all words have been written */
  while ( cnt )
  {
    mscStatusWait( MSC_STATUS_WDATAREADY, MSC_STATUS_WDATAREADY );
    MSC->WDATA = *p++;
    cnt       -= 4;
  }
  
  /* End writing */
  MSC->WRITECMD = MSC_WRITECMD_WRITEEND;
}

/**********************************************************
 * Writes multiple words to flash, using double word
 * writes.
 *
 * @param addr
 *    Where to start writing. Must be a valid flash address.
 * 
 * @param p
 *    Pointer to data
 * 
 * @param cnt
 *    Number of bytes to write. 
 * 
 * @returns 
 *    Number of bytes left that was NOT written.
 **********************************************************/
static uint32_t pgmBurstDouble( uint32_t addr, uint32_t *p, uint32_t cnt )
{
  /* Wait until MSC is ready */
  mscStatusWait( MSC_STATUS_BUSY, 0 );
  
  /* Enter start address */
  MSC->ADDRB    = addr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
  
  /* Write first words. Address will be automatically incremented. */
  MSC->WDATA    = *p++;
  MSC->WDATA    = *p++;
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;
  cnt          -= 8;

  /* Loop until all words have been written */
  while ( cnt > 7 )
  {
    mscStatusWait( MSC_STATUS_WDATAREADY, MSC_STATUS_WDATAREADY );
    MSC->WDATA = *p++;
    MSC->WDATA = *p++;
    cnt       -= 8;
  }
  
   /* End writing */
  MSC->WRITECMD = MSC_WRITECMD_WRITEEND;
  
  /* Return number of bytes not written by this function */
  return cnt;
}

void main(void)
{
  uint8_t  *pBuff;
  uint32_t addr, burst, pageMask, byteCount;

  /* Relocate vector table */
  SCB->VTOR = 0x20000000;
    
  /* Disable interrupts */
  __disable_irq();
  
  /* Signal setup */
  state.flashLoaderStatus = FLASHLOADER_STATUS_NOT_READY;
  state.debuggerStatus = DEBUGGERCMD_NOT_CONNECTED;

  /* Get device info including memory size */
  setupEFM32();
  
  /* Calculate size of available buffers. Two buffers are
   * used. Each buffer will  fill up half of the remaining RAM. 
   * Round down to nearest word boundry */
  state.bufferSize = (state.sramSize - ((uint32_t) &flashBuffer - 0x20000000)) / 2;
  
  /* Only use full 4 bytes (1 word) */
  state.bufferSize = state.bufferSize & 0xFFFFFFFC;
   
  /* Set the address of both buffers  */
  state.bufferAddress1 = (uint32_t) &flashBuffer;
  state.bufferAddress2 = ((uint32_t) &flashBuffer) + state.bufferSize;

  /* Signal setup complete. Ready to accept commands from programmer. */
  state.flashLoaderStatus = FLASHLOADER_STATUS_READY;

  /* Poll debuggerStatus field to listen for commands
   * from programmer */
  while(1)
  {
    
    /* Erase page(s) command */
    if (state.debuggerStatus == DEBUGGERCMD_ERASE_PAGE)
    {
      /* Clear the flag to indicate that we are busy */
      state.flashLoaderStatus = FLASHLOADER_STATUS_NOT_READY;
      state.debuggerStatus = DEBUGGERCMD_NONE;

      /* Enable flash writes */
      MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
      
      /* Get address of first page to erase */
      uint32_t writeAddress = state.writeAddress1;
      
      /* Erase all pages in the given range */
      for (addr = writeAddress; addr < writeAddress + state.numBytes1; addr += state.pageSize)
      {
        eraseSector( addr, state.pageSize );
      }
      
      /* Disable flash writes */
      MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

      /* Operation complete. Set flag to ready again. */
      state.flashLoaderStatus = FLASHLOADER_STATUS_READY;
    }
    
    /* Mass erase command */
    if (state.debuggerStatus == DEBUGGERCMD_MASS_ERASE)
    {
      /* Clear the flag to indicate that we are busy */
      state.flashLoaderStatus = FLASHLOADER_STATUS_NOT_READY;
      state.debuggerStatus = DEBUGGERCMD_NONE;

      /* Enable flash writes */
      MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
      
      /* Unlock Mass Erase */
      MSC->MASSLOCK = 0x631A;
      
      /* Erase entire flash */
      doFlashCmd(MSC_WRITECMD_ERASEMAIN0 | MSC_WRITECMD_ERASEMAIN1);
      
      /* Reset lock */
      MSC->MASSLOCK = 0;
      
      /* Disable flash writes again */
      MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

      /* Operation complete. Set flag to ready again. */
      state.flashLoaderStatus = FLASHLOADER_STATUS_READY;
    }
    

    /* Write command */
    if (state.debuggerStatus == DEBUGGERCMD_WRITE_DATA1 || state.debuggerStatus == DEBUGGERCMD_WRITE_DATA2 )
    {
      /* Select buffer based on write command */
      bool useBuffer1 = state.debuggerStatus == DEBUGGERCMD_WRITE_DATA1 ? true : false;
      
      /* Clear the flag to indicate that we are busy */
      state.flashLoaderStatus = FLASHLOADER_STATUS_NOT_READY;
      state.debuggerStatus = DEBUGGERCMD_NONE;

      pageMask  = ~(state.pageSize - 1);
      
      /* Set up buffer, size and destination */
      pBuff     = useBuffer1 ? (uint8_t *)state.bufferAddress1 : (uint8_t *)state.bufferAddress2;
      byteCount = useBuffer1 ? state.numBytes1                 : state.numBytes2;
      addr      = useBuffer1 ? state.writeAddress1             : state.writeAddress2;

      /* Enable flash writes */
      MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

      /* Use double word writes if available */
      if ( ( byteCount > 7 ) && writeDouble )
      {
        if ( addr & 7 )   /* Start address not on 8 byte boundary ? */
        {
          pgmWord( addr, *(uint32_t*)pBuff );
          pBuff     += 4;
          addr      += 4;
          byteCount -= 4;
        }

        /* Enable double word writes */
        MSC->WRITECTRL |= MSC_WRITECTRL_WDOUBLE;
        
        /* Writes as many words as possible using double word writes */
        while ( byteCount > 7 )
        {         
          /* Max burst len is up to next flash page boundary. */
          burst = MIN( byteCount, ( ( addr + state.pageSize ) & pageMask ) - addr );
 
          /* Write data to flash */
          burst -= pgmBurstDouble( addr, (uint32_t*)pBuff, burst );

          pBuff     += burst;
          addr      += burst;
          byteCount -= burst;
        }
        
        /* Wait until operations are complete */
        mscStatusWait( MSC_STATUS_BUSY, 0 );
        
        /* Disable double word writes */
        MSC->WRITECTRL &= ~MSC_WRITECTRL_WDOUBLE;
      }

      /* Writes all remaining bytes */
      while ( byteCount )
      {
        /* Max burst len is up to next flash page boundary. */
        burst = MIN( byteCount, ( ( addr + state.pageSize ) & pageMask ) - addr );

        /* Write data to flash */
        pgmBurst( addr, (uint32_t*)pBuff, burst );

        pBuff     += burst;
        addr      += burst;
        byteCount -= burst;
      }
      
      /* Wait until operations are complete */
      mscStatusWait( MSC_STATUS_BUSY, 0 );

      /* Disable flash writes */
      MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

      /* Operation complete. Set flag to ready again. */
      state.flashLoaderStatus = FLASHLOADER_STATUS_READY;
    }
  }
}

/* Configure stack size and fault handlers */
#pragma language=extended
#pragma segment="CSTACK"

/* Auto defined by linker */
extern unsigned char CSTACK$$Limit;

/* With IAR, the CSTACK is defined via project options settings */
#pragma data_alignment=256
#pragma location = ".intvec"
const void * const __vector_table[]=
{
    &CSTACK$$Limit,               /*  0 - Initial stack pointer */
    (void *) main,                /*  1 - Reset (start instruction) */
    (void *) NmiHandler,          /*  2 - NMI */
    (void *) HardFaultHandler     /*  3 - HardFault */
};
