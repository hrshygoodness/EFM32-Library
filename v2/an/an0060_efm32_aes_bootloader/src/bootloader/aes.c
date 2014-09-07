/**************************************************************************//**
 * @file aes.c
 * @brief Performs encryption operations using the EFM32 AES module
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
#include <string.h>
#include "em_device.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "flash.h"
#include "bootloader_config.h"


extern uint8_t encryptionKey[];
extern uint8_t initVector[];

/* Storage for 256 bit decryption key */
static uint8_t decryptionKey[AES_KEYSIZE];

/* Temporary storage for the previous cipher when doing CBC decryption */
static uint8_t prevCipher[AES_BLOCKSIZE];

/******************************************************************************
 * AES interrupt handler. Performs no operation. The interrupts
 * are only used for waking up from EM1 once the encryption/decryption
 * is done. 
 *****************************************************************************/
void AES_IRQHandler(void)
{
  /* Clear the interrupt flag */
  AES->IFC = AES_IFC_DONE;
}


/******************************************************************************
 * @brief Calculate 128 bits decryption key from encryption key
 *
 * @param key
 *   The encryption key
 *
 * @param decryptionKey
 *   Buffer to store the decryption key 
 *****************************************************************************/
RAMFUNC static void calcDecryptionKey256(const uint8_t *key, uint8_t *decryptionKey)
{
  int i;
  
  uint32_t *_key = (uint32_t *)key;
  uint32_t *_decryptionKey = (uint32_t *)decryptionKey;
    
  AES->CTRL = AES_CTRL_AES256;

  for (i=3; i>=0; i--)
  {
    AES->KEYHA = __REV(_key[i]);
    AES->KEYLA = __REV(_key[i+4]);
  }
  
  /* Start encryption */
  AES->CMD  = AES_CMD_START;

  /* Wait for AES to finish */
  while (AES->STATUS & AES_STATUS_RUNNING);

  /* Fetch finished low key */
  for (i=3; i>=0; i--)
  {
    _decryptionKey[i] = __REV(AES->KEYHA);
    _decryptionKey[i+4] = __REV(AES->KEYLA);
  }
}

/**************************************************************************//**
 * @brief Decrypt one AES block using the CBC mode. 
 *
 * @param key
 *   The decryption key
 *
 * @param inputData
 *   The cipher text block to decrypt
 *
 * @param prevCipher
 *   The cipher text of the previous block
 * 
 * @param outputData
 *   Buffer to store the decrypted block
 *****************************************************************************/
RAMFUNC void decryptCBC256(const uint8_t *key, const uint8_t* inputData, const uint8_t *prevCipher, uint8_t *outputData)
{
  int i;
  
  uint32_t *_key = (uint32_t *)key;
  uint32_t *_inputData = (uint32_t *)inputData;
  uint32_t *_prevCipher = (uint32_t *)prevCipher;
  uint32_t *_outputData = (uint32_t *)outputData;
    
  AES->CTRL = AES_CTRL_DECRYPT | AES_CTRL_AES256;
   
  /* Load key and input block */
  for (i=3; i>=0; i--)
  {
    AES->KEYHA = __REV(_key[i]);
    AES->KEYLA = __REV(_key[i+4]);
    AES->DATA  = __REV(_inputData[i]);
  }
  
  /* Start and wait until decryption is complete */
  AES->CMD = AES_CMD_START;
  while ( AES->STATUS & AES_STATUS_RUNNING );
  
  /* Retrieve decrypted block */
  for (i=3; i>=0; i--)
  {
    _outputData[i] = __REV(AES->DATA) ^ _prevCipher[i];
  }
  
}


/******************************************************************************
 * @brief Encrypt one AES block using the CBC mode
 *
 * @param key
 *   The encryption key
 *
 * @param inputData
 *   The plain text block to encrypt
 *
 * @param prevCipher
 *   The cipher text of the previous block
 * 
 * @param outputData
 *   Buffer to store the encrypted block
 *****************************************************************************/
RAMFUNC void encryptCBC128(const uint8_t *key, uint8_t *inputData, const uint8_t *prevCipher, uint8_t *outputData)
{
  int i;
  
  uint32_t *_key = (uint32_t *)key;
  uint32_t *_inputData = (uint32_t *)inputData;
  uint32_t *_prevCipher = (uint32_t *)prevCipher;
  uint32_t *_outputData = (uint32_t *)outputData;  
 
  AES->CTRL = 0;
   
  /* Load key and input block */
  for (i=3; i>=0; i--)
  {
    AES->KEYLA = __REV(_key[i]);
    AES->DATA  = __REV(_inputData[i] ^ _prevCipher[i]);
  }
  
  /* Start and wait for encryption to finish */
  AES->CMD = AES_CMD_START;
  while ( AES->STATUS & AES_STATUS_RUNNING );
  
  /* Retrieve encrypted block */
  for (i=3; i>=0; i--)
  {
    _outputData[i] = __REV(AES->DATA);
  } 
  
}


/******************************************************************************
 * Initialize decryption with 256-bit AES CBC. This function will
 * enable the AES clock, calculate the decryption key and 
 * initialize the initial vector for use with CBC.
 * This function should be called before decrypting anything. 
 *****************************************************************************/
RAMFUNC void startDecryptCBC256(void)
{
  int i;
  
  /* Enable the clock to the AES peripheral. This is needed 
   * before any of the AES registers can be accessed */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_AES;
  
  /* Use the hardcoded encryption key to calcululate the 
   * key used for decrypting the firmware */
  calcDecryptionKey256(encryptionKey, decryptionKey);
  
  /* Initialize the previous cipher with the initilization vector */
  for ( i=0; i<AES_BLOCKSIZE; i++ ) {
    prevCipher[i] = initVector[i];
  }
}

/******************************************************************************
 * Decrypt one AES block in CBC-256. 
 *****************************************************************************/
RAMFUNC void decryptBlockCBC256(uint8_t *input, uint8_t *output)
{
  int i;
  
  decryptCBC256(decryptionKey, input, prevCipher, output);
  
  /* Copy output to prev cipher for next iteration */
  for ( i=0; i<AES_BLOCKSIZE; i++ ) {
    prevCipher[i] = input[i];
  }
}

/******************************************************************************
 * End encryption. This function should be called after decryption is complete
 * and the AES peripheral is no longer needed. 
 *****************************************************************************/
RAMFUNC void endDecryptCBC256(void)
{
  /* Turn off the clock to AES to save power */
  CMU->HFCORECLKEN0 &= ~CMU_HFCORECLKEN0_AES;
}



  
