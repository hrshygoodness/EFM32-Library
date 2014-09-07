/*******************************************************************************
 * @file unlock_gecko.c
 * @brief Unlock Gecko (EFM32G) devices
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
#include "unlock_gecko.h"


/* This is the unlock sequence, transmitted LSB first. 
 * For most devices we have enough time to transmit these
 * commands using the normal functions declared in dap.c.
 * But, for Gecko (EFM32G) devices that we have lost debug
 * access to by some OTHER method than clearing the DLW
 * (e.g. disabling SWD pins, going to EM4 in the beginning
 * of main() etc.), this sequence must be performed fast
 * enough to fit within the AAP Window after reset. 
 * In this case the unlock sequence is bitbanged
 * by using EBI, since this is faster than what is 
 * possible by using GPIO. This array is used to 
 * generate the EBI bitbang sequence 
 */
uint8_t unlockSequence[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* Line reset */
  0x9E, 0xE7,                                /* JTAG-to-SWD */
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  /* Line reset */
  0x00, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00,  /* Read IDCODE */
  0xA9, 0x00, 0x00, 0x00, 0x00, 0x22,        /* Write PWRUPREQ to CTRL */ 
  0xB1, 0x00, 0x00, 0x00, 0x00, 0x00,        /* Write 0 to SELECT */ 
  0x8B, 0x00, 0x23, 0x98, 0xf5, 0x39,        /* Write unlock key to AAP_CMDKEY */
  0xA3, 0x20, 0x00, 0x00, 0x00, 0x20,        /* Write DEVICEERASE bit to AAP_CMD */
  0x8B, 0x00, 0x00, 0x00, 0x00, 0x00,        /* Write 0 to AAP_CMDKEY */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Idle cycles */
 
 
/* This array will hold the unlock sequence, coded in
 * a way that will output the correct waveform on the EBI
 * pins. Each bit requires two bytes of data to be fed
 * to EBI and we store everything in words so we can 
 * write as fast as possible to EBI. The size is
 * therefore
 *
 * N = sizeof(unlockSequence) * 8 * 2 / 4
 *   = sizeof(unlockSequence) * 4
 */
static uint32_t ebiUnlockSequence[sizeof(unlockSequence)*4];

/* The address to write to when using EBI bit-bang. 
 * This corresponds to the first word of EBI bank 2 */
static volatile uint32_t *ebiUnlockAddr = (uint32_t *)0x88000000;



/**********************************************************
 * Generates the values we feed to EBI to output the 
 * unlock sequence. Input is the bit pattern in 
 * unlockSequence. Output is the bytes we write to
 * ebiUnlockSequence. 
 **********************************************************/
void generateEbiValues(void)
{
  int i,j,bit;
  
  int N = sizeof(unlockSequence);
  
  uint8_t *p = (uint8_t *)ebiUnlockSequence; 
  
  for ( i=0; i<N; i++ ) {
    for ( j=0; j<8; j++ ) {
      bit = (unlockSequence[i] >> j) & 0x1;
      *p++ = bit << 1 | 0x0;                 /* Low clock period */
      *p++ = bit << 1 | 0x1;                 /* High clock period */
    }
  }
}

/**********************************************************
 * Initializes the EBI for SWD bit-banging
 **********************************************************/
void initEbi(void)
{
  CMU_ClockEnable(cmuClock_EBI, true);
  
  EBI_Init_TypeDef ebiConfig = EBI_INIT_DEFAULT;
  
  ebiConfig.mode = ebiModeD8A8;
    
  ebiConfig.writeHoldCycles = 0;
  ebiConfig.writeStrobeCycles = 1;
  ebiConfig.writeSetupCycles = 0;
  
  ebiConfig.readSetupCycles = 0;
  ebiConfig.readStrobeCycles = 1;
  ebiConfig.readHoldCycles = 0;
  
  
  ebiConfig.banks = EBI_BANK2;
  ebiConfig.wePolarity = ebiActiveHigh;
  
  EBI_Init(&ebiConfig);
  
  EBI->CTRL |= EBI_CTRL_NOIDLE2 | EBI_CTRL_ITS;
  
  EBI->ADDRTIMING2 = 0;
  
  EBI->ROUTE |= EBI_ROUTE_LOCATION_LOC1;
}

/**********************************************************
 * Deinits the EBI so the pins can be used by GPIO
 **********************************************************/
void deinitEbi(void)
{
  EBI_Disable();
  CMU_ClockEnable(cmuClock_EBI, false);
}


/**********************************************************
 * Uses EBI to perform the unlock sequence very fast. 
 * This function will reset the target (with pin reset)
 * several times and try different delays before 
 * starting the unlock sequence until it is successful. 
 * 
 * NOTE: in order for this to be successful the 
 * SWCLK,SWDIO pins must be mapped to PE8,PE9 since
 * the bit-bang sequence uses the two lower bits of 
 * the EBI bus. 
 **********************************************************/
void unlockTargetGecko(void)
{
  int i,m;
  bool success = false;
  
  generateEbiValues();
  
  int N = sizeof(ebiUnlockSequence);
  
  printf("Unlocking with EBI bit-bang\n");
  
  for ( m=0; m<20; m++ ) 
  {
    TRY    
      /* Configure EBI to control the pins */
      initEbi();
    
      hardResetTarget();
      
      /* Try different delays */
      delayUs(m * 10);
      
      /* Send the unlock sequence */
      for ( i=0; i<N; i++ ) {
        *ebiUnlockAddr = ebiUnlockSequence[i];
      }
      
      /* Wait until unlock operation is complete */
      delayMs(100);
      
      /* Configure GPIO to control the pins */
      deinitEbi();
        
      /* Reset target and check if it is unlocked */
      hardResetTarget();
      delayMs(50);
      
      /* Check if target is unlocked. This function will
       * simply return if we are successful and throw
       * an exception if we are not */
      verifyUnlockedStatus();
      
      /* If we get here, the target is unlocked. 
       * We cannot return in the middle of a TRY block. 
       * Set flag here and return after ENDTRY. */
      success = true;
    CATCH
      /* Do nothing, retry */
    ENDTRY
      
    if ( success )
    {
      return;
    }
  }
  
  /* If all the retries failed, stop and print an error message */
  RAISE(SWD_ERROR_UNLOCK_FAILED);
}