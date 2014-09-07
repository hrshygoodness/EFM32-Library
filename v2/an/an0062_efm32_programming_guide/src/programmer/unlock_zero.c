/*******************************************************************************
 * @file unlock_zero.c
 * @brief Unlock Zero Gecko devices
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
#include "em_device.h"
#include "em_ebi.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "errors.h"
#include "dap.h"
#include "utils.h"
#include "delay.h"
#include "debug_lock.h"
#include "unlock_zero.h"


extern bool targetLocked;


/**********************************************************
 * Performs AAP Window Expansion. This function will reset
 * target, send the AAP expansion sequence, initialize the
 * debug interface and verify that AAP can be accessed. 
 * It will try several delays between reset is released
 * and reading the AAP in case the reset line has a slow
 * rampup. 
 * 
 * After this function completes the AAP registers are
 * available. If it fails to access the AAP, it will 
 * throw an exception. 
 * 
 **********************************************************/
void performAapExpansionZero(void)
{  
  uint32_t dpId, apId;
  int i;
  
  bool success = false;
  
  for ( i=0; i<AAP_EXPANSION_RETRY_COUNT; i++ ) 
  {      
    
    printf("Trying %d\n", i);
    /* Pull reset pin low */
    GPIO_PinOutClear((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
    
    SWCLK_CLR();
    
    delayMs(50);
    
    /* Send the AAP Window expansion sequence */
    aapExtensionSequence();
    
    delayUs(10);
   
    /* Pull reset pin high again */
    GPIO_PinOutSet((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
        
    /* Try different delays in case of slow reset ramp */
    delayUs(10 * i);
    
    TRY
      /* Connect to SW-DP */
      dpId = initDp();       
      
      initAhbAp();
      
      apId = readMem(AAP_IDR_ZERO);
      
      if ( verifyDpId(dpId) && apId == EFM32_AAP_ID ) 
      {
        /* Success! AAP registers can now be accessed.
         * We cannot return in the middle of a TRY block.
         * Set flag here and return after ENDTRY */
        success = true;
      } 
    CATCH
      /* Do nothing on error. Try again. */
    ENDTRY
      
    /* Return here if we found the AAP registers*/
    if ( success )
    {
      return;
    }
  }
  
  /* Failed to get access to AAP registers. Raise an error. */
  RAISE(SWD_ERROR_AAP_EXTENSION_FAILED);
} 

/**********************************************************
 * This function will verify that the target is unlocked
 * by trying to access the AAP. It will also check that
 * the flash is erased.
 **********************************************************/
void verifyUnlockedStatusZero(void)
{
  int retry = CONNECT_RETRY_COUNT;
  bool success = false;  
  
  while ( retry > 0 )
  {
    TRY 
      delayUs(200);
      
      /* Initalize debug interface */
      initDp();        
      readApId();
      initAhbAp();
      
      /* Verify that Zero Gecko is unlocked by 
       * trying to read the AAP registers */
      checkIfZeroGeckoIsLocked();
      
      haltTarget();
      
      printf("Verifying that flash is erased\n");
      
      verifyFlashIsErased();
      
      /* If we get here, the target is unlocked */
      success = true;
    CATCH
      /* Do nothing on error. We retry */
    ENDTRY
      
    /* Simply return if target is unlocked */
    if ( success )
    {
      return;
    }
      
    retry--;
  } 
  
  /* Raise error if target is NOT unlocked */
  RAISE(SWD_ERROR_UNLOCK_FAILED);
}

  
/**********************************************************
 * Performs the Debug Unlock sequence to erase all
 * device contents and obtain debug access. 
 * In case the device is not locked, it will initiate
 * the AAP window extension sequence in order to 
 * access the AAP before trying to unlock. 
 * 
 * This function is a special case for the Zero Gecko
 * (with M0+ core), since the AAP is accessed with a 
 * different method than on devices with M3/M4 core. 
 **********************************************************/
void unlockTargetZero(void)
{  
  /* If target is unlocked we reset target and send the
   * AAP Window Expansion Sequence to have longer time
   * to access the AAP registers */
  if ( !targetLocked ) {
    printf("Target is not locked.\n" 
           "Performing AAP Window Expansion Sequence.\n");
    performAapExpansionZero();
  } 
  
  initAhbAp();
  
  printf("Sending Unlock Sequence\n");

  /* Start a Device Erase (Debug Unlock) operation */
  writeMem(AAP_CMDKEY_ZERO, AAP_UNLOCK_KEY);
  writeMem(AAP_CMD_ZERO, AAP_CMD_DEVICEERASE);
  writeMem(AAP_CMDKEY_ZERO, 0);
  
  /* Wait long enough for Device Erase to complete */
  delayMs(1000);
  
  /* Reset and verify that target is unlocked */
  hardResetTarget();
  delayMs(500);
  
  printf("Verifying that target is unlocked\n");
  
  verifyUnlockedStatusZero();
  
  /* If we get here, the target is unlocked */
  printf("Target successfully unlocked\n");
}





  