/**************************************************************************//**
 * @file bootloader.c
 * @brief Main file for the AES Bootloader
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
#include "em_usart.h"
#include "em_cmu.h"
#include "em_msc.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_chip.h"
#include "boot.h"
#include "uart.h"
#include "xmodem.h"
#include "aes.h"
#include "flash.h"
#include "verify.h"
#include "debuglock.h"
#include "bootloader_config.h"




/******************************************************************************
 * Sends a message over UART to let the remote end know that we
 * are in bootloader mode and ready to accept commands. 
 *****************************************************************************/
void sendWelcomeMessage(void)
{
  BLUART_sendString("EFM32 AES Bootloader\r\n");
  
  BLUART_sendString("Current firmware is...");
  
  if ( isFirmwareValid() ) 
  { 
    BLUART_sendString("OK\r\n");
  } 
  else 
  {
    BLUART_sendString("INVALID!\r\n");
  }
  
  BLUART_sendString("Debug Lock is...");
  if ( *((uint32_t *)DEBUG_LOCK_WORD) == 0 ) 
  {
    BLUART_sendString("ENABLED\r\n");
  }
  else
  {
    BLUART_sendString("DISABLED\r\n");
  }
    
  
  BLUART_sendString("Please enter a command: \r\n"
                    " h - display this message\r\n"
                    " u - upload encryped firmware\r\n"
                    " v - verify current firmware\r\n"
                    " l - lock debug access\r\n"
                    " r - reset\r\n");
}


/******************************************************************************
 * Enters 'download mode' where the MCU accepts an encrypted firmware
 * over XMODEM-CRC. 
 *****************************************************************************/
void enterDownloadMode(void)
{  
  /* Starts an XMODEM-CRC download */
  if ( XMODEM_download() ) {
    
    BLUART_sendString("Firmware downloaded\r\n");
    
    /* If temporaray storage is used: first verify the application, 
     * then start copying it to the boot area. */
    if ( USE_TEMP_STORAGE ) 
    {
      BLUART_sendString("Verifying firmware...");
      
      if ( verifyTempStorage() ) 
      {
        /* Mark the temporary storage as verified. This allows
         * the bootloader to know it is safe to restart copying
         * the firmware, should the following operations fail */
        markTempAsVerified();
        BLUART_sendString("OK\r\n");
        
        /* Start copying from temp storage to boot region */
        BLUART_sendString("Activating firmware...");
        markFirmwareAsDeleted();
        copyFirmwareFromTempStorage();
        
        /* Only mark the boot region as verified after everything is copied. */
        markFirmwareAsVerified();
        BLUART_sendString("DONE\r\n");
      } 
      else  
      {
        /* Verification of temporary storage failed */
        BLUART_sendString("INVALID\r\n");
        return;
      }
      
    } 
    else /* Temporary storage is not used */
    {
      /* Start verifying the firmware */
      BLUART_sendString("Verifying firmware...");
      
      if ( verifyActiveFirmware() ) 
      {
        /* Mark the firmware as verified so the bootloader 
         * only has to check this field next time */
        markFirmwareAsVerified();
        BLUART_sendString("OK\r\n");
      } 
      else 
      {
        /* Verification of firmware failed */
        BLUART_sendString("INVALID\r\n");
      }
    }
  } 
  else 
  {
    /* Something went wrong during the download. Abort. */
    BLUART_sendString("Firmware download failed!\r\n");
  }
}

/******************************************************************************
 * Enables Debug Lock by clearing the Debug Lock Word and reset the MCU.
 *****************************************************************************/
void enableDebugLock(void)
{
  if ( DEBUGLOCK_lock() ) {
    BLUART_sendString("Debug Lock Word cleared successfully!\r\n"
                      "Please reset MCU to lock debug access\r\n");
  } else {
    BLUART_sendString("Failed to enable Debug Lock\r\n");
  }
}

  

/******************************************************************************
 * This function waits for commands over UART
 *****************************************************************************/
void commandLoop(void)
{
  uint8_t command = 0;
  
  sendWelcomeMessage();
  
  while(1) {
    
    /* Wait for command */
    command = BLUART_receive();
    
    switch(command)
    {
    case 'h':
      sendWelcomeMessage();
      break;
    case 'u':
      enterDownloadMode();
      break;
    case 'v':
      BLUART_sendString("Verifying firmware...");
      if ( verifyActiveFirmware() ) {
        BLUART_sendString("OK\r\n");
      } else {
        BLUART_sendString("INVALID\r\n");
      }
      break;
    case 'l':
      BLUART_sendString("Locking debug access...\r\n");
      enableDebugLock();
      break;
    case 'r':
      BLUART_sendString("Reset\r\n");
      NVIC_SystemReset();
      break;
    }
  }
}

/* Interrupt handler only used to wake up the MCU when 
 * waiting for the bootloader pin to be pulled low */
void GPIO_EVEN_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;
}

/*****************************************************************************
 * Waits in EM3 until the bootloader pin is pulled low. This saves power
 * while waiting to install firmware. 
 *****************************************************************************/
void enterLowPowerWait(void)
{
  /* Enable interrupt on GPIO pin. Note that 
   * if the pin is changed to an odd pin number
   * the interrupt handler must also be changed */
  GPIO_IntConfig(BOOTLOADER_PIN, true, false, true);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  /* Wait in EM3 until the pin is pulled high */
  while ( !GPIO_PinInGet(BOOTLOADER_PIN) ) {
    EMU_EnterEM3(false);
  }
  
  /* Disable interrupts again */
  GPIO_IntConfig(BOOTLOADER_PIN, false, false, false);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
}


/*****************************************************************************
 * Enters 'bootloader mode' where a new encrypted firmware upgrade can be 
 * sent over UART.
 *****************************************************************************/
void enterBootloaderMode(void)
{
  FLASH_init();

  /* Enable UART */
  BLUART_init();
  
  /* If the current firmware is invalid, but temp storage is
   * enabled and contains a valid firmware, copy this firmware
   * to boot region immediately.  */
  if ( USE_TEMP_STORAGE && !isFirmwareValid() && isTempStorageValid() )
  {
    copyFirmwareFromTempStorage();
    markFirmwareAsVerified();
  }
  
  /* Check if bootloader pin is pulled high. If it is left low
   * enter EM2 to save power. The MCU will wake up when pin is 
   * pulled high. */
  if ( !GPIO_PinInGet(BOOTLOADER_PIN) ) {
    enterLowPowerWait();
  }

  /* Wait for commands over UART */
  commandLoop();
}

   

/*****************************************************************************
 * Bootloader entry point. 
 *****************************************************************************/
int main(void)
{
  CHIP_Init();
  
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(BOOTLOADER_PIN, gpioModeInputPull, 0);
  
  /* The bootloader pin is left low, boot normally */
  if ( !GPIO_PinInGet(BOOTLOADER_PIN) ) 
  {
    
    /* Verify firmware before booting */
    if ( isFirmwareValid() ) 
    {
      BOOT_boot();
    } 
    else 
    {
      
      /* Firmware is invalid. Check temp storage */
      if ( USE_TEMP_STORAGE && isTempStorageValid() )
      {
        FLASH_init();
        copyFirmwareFromTempStorage();
        markFirmwareAsVerified();
        
        /* Application will boot after reset */
        NVIC_SystemReset();
      }
      
      /* No valid application is present. Enter bootloader mode. */
      else 
      {
        enterBootloaderMode();
      }
    }
  } 
  /* The bootloader pin is pulled high, enter bootloader */
  else 
  {
    enterBootloaderMode();
  }
  
  /* Never reached */
  return 0;
}
