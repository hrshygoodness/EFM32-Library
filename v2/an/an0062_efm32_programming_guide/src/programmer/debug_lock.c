/*******************************************************************************
 * @file debug_lock.c
 * @brief Performs Debug Lock and Unlock
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "em_device.h"
#include "dap.h"
#include "utils.h"
#include "debug_lock.h"
#include "errors.h"
#include "em_gpio.h"
#include "em_ebi.h"
#include "em_cmu.h"
#include "flash_write.h"
#include "delay.h"


/* The address of the Debug Lock Word. This word must be 
 * cleared in order to lock debug access */
#define DEBUG_LOCK_WORD  (0x0FE04000 + 127 * 4)



extern bool targetLocked;


/**********************************************************
 * Verify that the flash is cleared. To save time
 * we only check the first word on every page. 
 * this could be modified to check every word.
 **********************************************************/
void verifyFlashIsErased(void)
{
  printf("Verifying that flash is erased\n");

  uint32_t addr = 0;
  uint32_t value;
  
  /* Get flash page size and total size from the target DI page */
  int flashSize = getFlashSize();
  int pageSize = getPageSize();
  
  while ( addr < flashSize ) 
  {
    value = readMem(addr);
    if ( value != 0xFFFFFFFF ) 
    {
      printf("Error at address 0x%.8x\n"
             "Value should be 0xFFFFFFFF, is 0x%.8x\n", addr, value);
     
      RAISE(SWD_ERROR_DEVICE_ERASE_FAILED);
    }
    addr += pageSize;
  }
}


/**********************************************************
 * This function will verify that the target is unlocked
 * by trying to access the AHB-AP. It will also check that
 * the flash is erased because if we tried to unlock a 
 * already unlocked chip (e.g we lost debug access because 
 * the debug pins are disabled or we enter EM4 too fast) 
 * the AHB-AP would already be available. 
 **********************************************************/
void verifyUnlockedStatus(void)
{
  int retry = CONNECT_RETRY_COUNT;
  bool success = false;
    
  uint32_t apId;
  
  while ( retry > 0 )
  {
    TRY 
      delayUs(200);
      
      initDp();        
      apId = readApId();
      
      /* Verify that target is unlocked by reading the AHB-AP ID */
      if ( verifyAhbApId(apId) )
      {
        initAhbAp();
   
        /* Verify that flash is erased */
        verifyFlashIsErased();

        /* If we get here, the target is unlocked */
        success = true;
      }
     CATCH
       /* Do nothing on error. We retry */
     ENDTRY
     
     /* If unlocked return from here since we cannot
      * return from within a TRY block */
     if ( success )
     {
       return;
     }
     
     retry--;
  }
  
  /* Raise an exception if device is NOT unlocked */
  RAISE(SWD_ERROR_UNLOCK_FAILED);
}


/**********************************************************
 * Locks the target by clearing the Debug Lock Word and 
 * then performing a pin reset. 
 **********************************************************/
void lockTarget(void)
{
  uint32_t apId;
  
  printf("Locking target\n");
  
  /* Clear Debug Lock Word */
  writeWordToFlash(DEBUG_LOCK_WORD, 0);
  
  printf("Verifying that Debug Lock Word is cleared\n");
  
  /* Verify that DLW is cleared */
  uint32_t dlw = readMem(DEBUG_LOCK_WORD);
  if ( dlw != 0 ) {
    RAISE(SWD_ERROR_CLR_DLW_FAILED);
  }
  
  /* Perform a pin reset. After this the target should be locked. */
  hardResetTarget();
  delayMs(100);

  /* Verify that target is locked by reading the AAP ID */
  initDp();        
  apId = readApId();
  
  if ( apId == EFM32_AAP_ID )
  {
    printf("Target successfully locked\n");
    return;
  }
  else
  {
    RAISE(SWD_ERROR_LOCK_FAILED);
  }
    
}


/**********************************************************
 * Performs the Debug Unlock sequence to erase all
 * device contents and obtain debug access. 
 * In case the device is not locked, it will initiate
 * the AAP window extension sequence in order to 
 * access the AAP before trying to unlock. 
 **********************************************************/
void unlockTarget(void)
{  
  /* If target is unlocked we reset target and send the
   * AAP Window Expansion Sequence to have longer time
   * to access the AAP registers */
  if ( !targetLocked ) {
    printf("Target is not locked.\n" 
           "Performing AAP Window Expansion Sequence.\n");
    performAapExpansion();
  } 
  
  printf("Sending Unlock Sequence\n");
    
  /* Start a Device Erase (Debug Unlock) operation */
  writeAP(AAP_CMDKEY, AAP_UNLOCK_KEY);
  writeAP(AAP_CMD, AAP_CMD_DEVICEERASE);
  writeAP(AAP_CMDKEY, 0);
    
  /* Wait long enough for Device Erase to complete. 
   * This is typically around 50ms. We wait much longer
   * just to be sure. */
  delayMs(1000);
  
  /* Reset and verify that target is unlocked */
  hardResetTarget();
  delayMs(100);  

  /* Verify that target is unlocked. This function will raise
   * an exception if target is not unlocked. */
  verifyUnlockedStatus();
  
  /* If we get here, the target is unlocked */
  printf("Target successfully unlocked\n");
}

/**********************************************************
 * Performs AAP Window Expansion. This function will reset
 * target, send the AAP expansion sequence, initialize the
 * debug interface and verify that AAP can be accessed. 
 * It will try several delays between reset is released
 * and reading the AAP in case the reset line has a slow
 * ramp-up. 
 * 
 * After this function completes the AAP registers are
 * available. If it fails to access the AAP, it will 
 * throw an exception. 
 * 
 **********************************************************/
void performAapExpansion(void)
{  
  uint32_t dpId, apId;
  int i,j;
  bool success = false;
  
  for ( i=0; i<AAP_EXPANSION_RETRY_COUNT; i++ ) {
    
    /* Pull reset pin low */
    GPIO_PinOutClear((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
    
    SWCLK_CLR();
    
    delayMs(50);
    
    /* Send the AAP Window expansion sequence */
    aapExtensionSequence();
    
    /* Release reset */
    delayUs(10);
    GPIO_PinOutSet((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
    
    /* Try different delays in case of slow reset ramp */
    for ( j=0; j<i; j++ ) {
      delayUs(10);
    }
    
    /* Connect to SW-DP */
    TRY
      dpId = initDp();        
      apId = readApId();
      
      if ( verifyDpId(dpId) && apId == EFM32_AAP_ID ) 
      {
        
        /* Success! AAP registers can now be accessed.
         * We cannot return in the middle of a TRY block.
         * Set flag here and return after ENDTRY */
        success = true;
      }
    CATCH
      /* Do nothing in case of error. We retry. */
    ENDTRY
      
    /* Return if we found the AAP registers*/
    if ( success )
    {
      printf("AAP registers found\n");
      return;
    }
  }
  
  /* Failed to get access to AAP registers. Raise an error. */
  RAISE(SWD_ERROR_AAP_EXTENSION_FAILED);
}

  
  
  
    
    
      
    
