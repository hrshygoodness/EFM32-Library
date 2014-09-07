/**************************************************************************//**
 * @file verify.c
 * @brief Verification of firmware validity
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

#include <string.h>
#include "em_cmu.h"
#include "aes.h"
#include "bootloader_config.h"
#include "verify.h"
#include "flash.h"


extern uint8_t hashKey[];
extern uint8_t hashInitVector[];


/******************************************************************************
 * Calculates the hash of the firmware. The hash is the last 128-bit cipher
 * text block in a CBC mode AES encryption. 
 * 
 * @param firmwareAddress 
 *    Address of the firmware location
 * 
 * @param hashValue
 *    Buffer to store the hash value. Must be at least 16 bytes wide.
 *****************************************************************************/
static bool calculateHash(uint32_t firmwareAddress, uint8_t *hashValue)
{
  int i;
  
  FirmwareHeader *fwHeader = (FirmwareHeader *)firmwareAddress;
  
  /* Check that size is within limits */
  if ( fwHeader->size > FIRMWARE_END_ADDRESS - (FIRMWARE_START_ADDRESS + FIRMWARE_HEADER_SIZE) + 1 ) {
    return false;
  }
  
  /* Get fixed boot address for firmware */
  uint8_t *addr = (uint8_t *)(firmwareAddress + FIRMWARE_HEADER_SIZE);
  
  /* Calculate end address of firmware */
  uint8_t *endAddr = (uint8_t *)(firmwareAddress + FIRMWARE_HEADER_SIZE + fwHeader->size);
  
  /* Buffer holding the last computed cipher text block */
  uint8_t prevCipher[AES_BLOCKSIZE];
  
  /* Initialize prevCipher with the init vector */
  for ( i=0; i<AES_BLOCKSIZE; i++ ) {
    prevCipher[i] = hashInitVector[i];
  }
  
  /* Enable the AES clock. This is needed before accessing the AES registers */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_AES;
  
  /* Loop over the entire application */
  while ( addr < endAddr ) {
     
    /* Encrypt a block in CBC mode. Store output in hashValue */
    encryptCBC128(hashKey, addr, prevCipher, hashValue);
    
    /* Store the cipher text for next iteration */
    for ( i=0; i<AES_BLOCKSIZE; i++ ) {
      prevCipher[i] = hashValue[i];
    }
    
    /* Increase the source pointer by one AES block (4 words) */
    addr += AES_BLOCKSIZE;
  }
  
  /* Disable the AES clock again to save power */
  CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_AES;
  
  return true;
}


/******************************************************************************
 * Verifies if a firmware image matches the hash value in the header. 
 * If the calculated hash matches the value found in the header
 * this function returns true. Returns false otherwise.
 *****************************************************************************/
bool verifyFirmware(uint32_t firmwareAddress)
{
  int i;
  uint8_t calculatedHash[AES_BLOCKSIZE];
  FirmwareHeader *fwHeader = (FirmwareHeader *)firmwareAddress;
  
  /* Calulate hash of the entire application in flash */
  if ( !calculateHash(firmwareAddress, calculatedHash) ) {
    return false;
  }
  
  /* Compare with the expected value in header */
  for ( i=0; i<AES_BLOCKSIZE; i++ ) {
    if ( calculatedHash[i] != fwHeader->hash[i] ) {
      return false;
    }
  }
  
  return true;
}

bool verifyActiveFirmware(void)
{
  return verifyFirmware(FIRMWARE_START_ADDRESS);
}

bool verifyTempStorage(void)
{
  return verifyFirmware(TEMP_START_ADDRESS);
}

bool isFirmwareValid(void)
{
  FirmwareHeader *fwHeader = (FirmwareHeader *)FIRMWARE_START_ADDRESS;
  return fwHeader->verified == FW_VERIFIED;
}  

bool isTempStorageValid(void)
{
  FirmwareHeader *fwHeader = (FirmwareHeader *)TEMP_START_ADDRESS;
  return fwHeader->verified == FW_VERIFIED; 
}

void markFirmwareAsVerified(void)
{
  FirmwareHeader *fwHeader = (FirmwareHeader *)FIRMWARE_START_ADDRESS;
  FLASH_writeWord(&fwHeader->verified, FW_VERIFIED);
}

void markTempAsVerified(void)
{
  FirmwareHeader *fwHeader = (FirmwareHeader *)TEMP_START_ADDRESS;
  FLASH_writeWord(&fwHeader->verified, FW_VERIFIED);
}

void markFirmwareAsDeleted(void)
{
  FirmwareHeader *fwHeader = (FirmwareHeader *)FIRMWARE_START_ADDRESS;
  FLASH_writeWord(&fwHeader->verified, FW_DELETED);
}


