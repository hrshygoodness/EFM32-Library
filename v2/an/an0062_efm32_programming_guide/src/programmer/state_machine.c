/**************************************************************************//**
 * @file state_machine.c
 * @brief Handles the state machine where user can select and execute an action
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
#include <stdbool.h>
#include <setjmp.h>
#include <stdio.h>
#include "em_device.h"
#include "dap.h"
#include "utils.h"
#include "bsp.h"
#include "state_machine.h"
#include "debug_operations.h"
#include "errors.h"
#include "debug_lock.h"
#include "flash_write.h"
#include "use_flashloader.h"
#include "unlock_gecko.h"
#include "unlock_zero.h"
#include "kits.h"
#include "delay.h"

#ifdef STK
#include "segmentlcd.h"
#endif


/* This flag is set by the ISR for the push button 
 * PB1 to indicate that the currently selected 
 * task should be executed. Each task will clear 
 * the flag when executing */
volatile bool execute = false;

/* Holds the currently selected state/task. The state
 * can be selected by using push button PB0 */
int state = STATE_IDLE;

static int lastState = STATE_IDLE;

/* Flag indicating if the target is connected or not */
static bool connected = false;

/* Flag indicating if the target MCU is locked */
bool targetLocked = false;


/**********************************************************
 * Handles the Flash with MSC state.
 **********************************************************/
void stateFlashMsc(void)
{
#ifdef STK
  SegmentLCD_Write("MSC");
#endif
  
  if ( lastState != STATE_FLASH_MSC ) 
  {
    printf("SELECT: Upload with MSC\n");
  }
  
  if ( execute ) {
    
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      flashWithMsc();
      state = STATE_IDLE;
    CATCH
      /* Print error message */
      printf("SWD error: %s\n", getErrorString(errorCode));
    
      /* Schedule new connection */
      state = STATE_CONNECT;
      execute = true;
    ENDTRY
      
      

  }
}


/**********************************************************
 * Handles the Flash with Flashloader state
 **********************************************************/
void stateFlashWithFlashloader(void)
{
#ifdef STK
  SegmentLCD_Write("FLOAD");
#endif 
  
  if ( lastState != STATE_FLASH_FL ) 
  {
    printf("SELECT: Upload with flashloader\n");
  }

  if ( execute ) 
  {
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      flashWithFlashloader(true);
      state = STATE_IDLE;
    CATCH
      /* Print error message */
      printf("SWD error: %s\n", getErrorString(errorCode));
      state = STATE_CONNECT;
    ENDTRY
  }
}
 
/**********************************************************
 * Handles the Connect state
 **********************************************************/  
void stateConnect(void)
{
  uint32_t retry = CONNECT_RETRY_COUNT;
  
  bool abort = false;
 
#ifdef STK
  SegmentLCD_Write("Connect");
#endif
  
  if ( lastState != STATE_CONNECT ) 
  {
    printf("SELECT: Connect\n");
  }

  
  if ( execute )
  {
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    /* Reset target */
    hardResetTarget();
    
    delayMs(50);
    
    /* Try connecting several times */
    while ( retry-- > 0 ) 
    {
      TRY
        /* Try to connect */
        connectToTarget();
        
        /* If we get here the connection was successful */
        connected = true;
        targetLocked = false;
        state = STATE_IDLE;
        
        /* Stop retrying */
        abort = true;
      CATCH
        
        if ( errorCode == SWD_ERROR_MCU_LOCKED ) 
        {
          /* Abort if the target is locked */
          targetLocked = true;
          connected = false;
          state = STATE_IDLE;
          
          /* Stop retrying */
          abort = true;
        } 
        else 
        {
          /* Print error message if connection failed */
          printf("SWD Error: %s\n", getErrorString(errorCode));
          printf("Failed to connect. Retrying...\n");
          delayUs(200);
        }
      ENDTRY
        
      /* We cannot return in the middle of a TRY block. 
       * Check flag and return from this point */
      if ( abort )
      {
        return;
      }
    }
    
    /* Connection failed. */
    connected = false;
    printf("Failed to connect after %d retries\n", CONNECT_RETRY_COUNT);
    state = STATE_IDLE;
  }
}


/**********************************************************
 * Handles the Idle state. This state does nothing. 
 * The state is entered when another state has finished
 * executing. 
 **********************************************************/  
void stateIdle(void)
{
#ifdef STK
  if ( connected ) {
    SegmentLCD_Write("Connctd");
  } else if ( targetLocked ) {
    SegmentLCD_Write("Locked");
  } else {
    SegmentLCD_Write("No conn");
  }
#endif
 
  
  /* Print a message when we enter this state */
  if ( lastState != STATE_IDLE ) 
  {
    if ( connected ) {
      printf("Connected\n");
    } else if ( targetLocked ) {
      printf("Target is locked\n");
    } else {
      printf("No connection\n");
    }  
  }
   
  if ( execute ) 
  {
    /* The idle state does nothing */
    execute = false;
  }
}

/**********************************************************
 * Handles the Lock state. This state enables Debug Lock.
 **********************************************************/  
void stateLock(void)
{
#ifdef STK
  SegmentLCD_Write("Lock");
#endif
  
  if ( lastState != STATE_LOCK ) 
  {
    printf("SELECT: Lock target\n");
  }

  if ( execute ) 
  {
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      lockTarget();
    
      /* Schedule new connection */
      state = STATE_CONNECT;
      execute = true;
    CATCH
      /* Print error message */
      printf("Locking target failed: %s\n", getErrorString(errorCode));
    
      /* Schedule new connection */
      state = STATE_CONNECT;
      execute = true;
    ENDTRY;
  }
}


/**********************************************************
 * Handles the Unlock state. This state unlocks a device.
 **********************************************************/ 
void stateUnlock(void)
{
#ifdef STK
  SegmentLCD_Write("Unlock");
#endif 
  
  if ( lastState != STATE_UNLOCK ) 
  {
    printf("SELECT: Unlock target (Normal)\n");
  }
    
  if ( execute ) 
  {
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      unlockTarget();
    
      /* If we get here the target is unlocked */
      targetLocked = false;
    
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    CATCH
      /* Print error message */
      printf("Unlock target failed: %s\n", getErrorString(errorCode));
      
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    ENDTRY
  }       
}

/**********************************************************
 * Handles the Unlock (Gecko) state. This state
 * can unlock a Gecko (EFM32G) device during the AAP
 * window by performing the unlock sequence as fast 
 * as possible by bit-banging with EBI. This requires
 * that EBI_AD pins are used for SWDIO/SWCLK. 
 **********************************************************/ 
void stateUnlockGecko(void)
{
#ifdef STK
  SegmentLCD_Write("Unlck G");
#endif
  
  if ( lastState != STATE_UNLOCK_GECKO ) 
  {
    printf("SELECT: Unlock target (Gecko)\n");
  }

  
  if ( execute ) 
  {
    /* Acknowledge execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      unlockTargetGecko();
    
      /* If we get here the target is unlocked */
      targetLocked = false;
    
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    CATCH
      /* Print error message */
      printf("Unlock failed: %s\n", getErrorString(errorCode));
    
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    ENDTRY
    
  }
}

void stateUnlockZero(void)
{
#ifdef STK
  SegmentLCD_Write("Unlck Z");
#endif
  
  if ( lastState != STATE_UNLOCK_ZERO ) 
  {
    printf("SELECT: Unlock target (Zero)\n");
  }

  if ( execute ) 
  {
    /* Acknowlege execute */
    execute = false;
    state = STATE_BUSY;
    
    TRY
      /* Execute command */
      unlockTargetZero();
    
      /* If we get here the target is unlocked */
      targetLocked = false;
    
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    CATCH
      /* Print error message */
      printf("Unlock target failed: %s\n", getErrorString(errorCode));
      
      /* Schedule a new connection */
      state = STATE_CONNECT;
      execute = true;
    ENDTRY
  }       
}


/**********************************************************
 * Execute different functions depending on what the 
 * user selects with push buttons. 
 * PB0 selects a state. PB1 executes the current state.
 **********************************************************/
void stateLoop(void)
{  
  /* Try to connect at startup */
  lastState = STATE_CONNECT;
  state = STATE_CONNECT;
  execute = true;
  
  while(1) 
  {
    
    __disable_irq();
    int curState = state;
    __enable_irq();
    
    /* Set status leds */
    setConnectedLed(connected);
    setBusyLed(curState != STATE_IDLE && execute);
    
    switch (curState) {
    case STATE_IDLE:
      stateIdle();
      break;
    case STATE_FLASH_MSC:
      stateFlashMsc();
      break;
    case STATE_FLASH_FL:
      stateFlashWithFlashloader();
      break;
    case STATE_CONNECT:
      stateConnect();
      break;
    case STATE_LOCK:
      stateLock();
      break;
    case STATE_UNLOCK:
      stateUnlock();
      break;
    case STATE_UNLOCK_GECKO:
      stateUnlockGecko();
      break;
    case STATE_UNLOCK_ZERO:
      stateUnlockZero();
      break;
    }
    
    lastState = curState;
    
#if !defined(STK)
    checkButtons();
#endif
  }  
}

/**********************************************************
 * Retrieves the next state. The user can switch between
 * different states with PB0. 
 **********************************************************/
uint32_t getNextState(uint32_t curState)
{
  switch(curState)
  {
  case STATE_CONNECT:
    return STATE_FLASH_MSC;
  case STATE_FLASH_MSC:
    return STATE_FLASH_FL;
  case STATE_FLASH_FL:
    return STATE_LOCK;
  case STATE_LOCK:
    return STATE_UNLOCK;
  case STATE_UNLOCK:
    return STATE_UNLOCK_GECKO;
  case STATE_UNLOCK_GECKO:
    return STATE_UNLOCK_ZERO;
  case STATE_UNLOCK_ZERO:
    return STATE_CONNECT;
  default:
    return STATE_CONNECT;
  }
}    
     
