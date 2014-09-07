/***************************************************************************//**
 * @file aes_cbc_128.c
 * @brief AES CBC 128-bit interrupt driven functions for EFM32
 * @author Silicon Labs
 * @version 1.12
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

#include <stdbool.h>
#include "em_device.h"
#include "aes_cbc_128.h"

static uint16_t          numberOfBlocks;
static uint16_t          blockIndex;
static uint32_t*         outputDataG;
static const uint32_t*   inputDataG;
static volatile uint32_t prev[4];
static volatile bool     aesFinished;
static volatile bool     decryptG;


/**************************************************************************//**
 * @brief  AES IRQ Handler
 *****************************************************************************/
void AES_IRQHandler(void)
{
  int i;
  
  /* Clear interrupt flag */
  AES->IFC = AES_IFC_DONE;

  /* Decryption */
  if (decryptG)
  {
    if (blockIndex < numberOfBlocks)
    {
      /* Writing to the XOR register here does NOT trigger decryption*/    
      for (i = 3; i >= 0; i--)
      {
        AES->XORDATA = __REV(prev[i]);
        prev[i]      = inputDataG[i];
      }
      inputDataG += 4;

      /* Store decrypted data */       
      for (i = 3; i >= 0; i--)
      {
        outputDataG[i] = __REV(AES->DATA);
      }
      outputDataG += 4;

      /* Load data and trigger a decryption */   
      for (i = 3; i >= 0; i--)
      {
        AES->DATA = __REV(inputDataG[i]);
      }
    }

    /* Indicate last block */
    else
    {
      aesFinished = true;
    }

    /* Increase block index */
    blockIndex++;
  }

  /* Encryption */
  else
  {
    /* Store encrypted data */  
    for (i = 3; i >= 0; i--)
    {
      outputDataG[i] = __REV(AES->DATA);
    }

    outputDataG += 4;

    if (blockIndex < numberOfBlocks)
    {
      /* If not last block, write new data to be decrypted. */
      /* AES is started automatically by the last write */
      inputDataG += 4;      
      for (i = 3; i >= 0; i--)
      {
        AES->XORDATA = __REV(inputDataG[i]);
      }   
    }

    /* Indicate last block */
    else
    {
      aesFinished = true;
    }

    /* Increase block index */
    blockIndex++;
  }
}

/**************************************************************************//**
 * @brief  Encrypt/decrypt with 128-bit key in CBC mode. Function returns after
 * initializing encryption/decryption. Use AesFinished() to check for 
 * completion. Output data only valid after completion. 
 *
 * @param[in] key
 *   When doing encryption, this is the 128-bit encryption key. When doing
 *   decryption, this is the 128-bit decryption key. The decryption key may
 *   be generated from the encryption key with AES_DecryptKey128().
 *
 * @param[in] inputData
 *   Buffer holding data to encrypt/decrypt.
 *
 * @param[out] outputData
 *   Buffer to put output data, must be of size blockNumber*16 bytes. This can
 *   be the same location as inputData.
 *
 * @param[in] decrypt
 *   True to decrypt, false to encrypt
 *
 * @param[in] blockNumber
 *   Number of 128-bit blocks to decrypt/encrypt.
 *
 * @param[in] iv
 *   128-bit initialization vector to use
 *****************************************************************************/
void AesCBC128(const uint8_t* key,
               const uint8_t* inputData,
               uint8_t*       outputData,
               bool           decrypt,
               const uint32_t blockNumber,
               const uint8_t* iv)
{
  int i;
  uint32_t* iv32  = (uint32_t*) iv;
  uint32_t* key32 = (uint32_t*) key;
  
  /* Copy to global variables */
  inputDataG     = (uint32_t*) inputData;
  outputDataG    = (uint32_t*) outputData;
  numberOfBlocks = blockNumber;

  /* Global flag for selection encryption or decryption */
  decryptG = decrypt;

  /* Reset block index */
  blockIndex = 0;

  /* Initialize finished flag */
  aesFinished = false;

  /* Clear and enable AES interrupt */
  AES->IFC = AES_IFC_DONE;
  AES->IEN = AES_IEN_DONE;
  NVIC_EnableIRQ(AES_IRQn);
  

  /* Write the KEY to AES module */
  /* Writing only the KEYHA 4 times will transfer the key to all 4
   * high key registers, used here, in 128 bit key mode, as buffers */   
  for (i = 3; i >= 0; i--)
  {
    AES->KEYHA = __REV(key32[i]);
  }

  /* Decryption */
  if (decryptG)
  {
    /* Configure AES module */
    AES->CTRL = AES_CTRL_DECRYPT |     /* Set decryption */
                AES_CTRL_KEYBUFEN |    /* Use key buffering */              
                AES_CTRL_DATASTART;    /* Start decryption on data write */

    /* Make a copy of the initial vector */
    for (i = 0; i < 4; i++)
    {
      prev[i] = iv32[i];
    }

    /* Load data and trigger a decryption */   
    for (i = 3; i >= 0; i--)
    {
      AES->DATA = __REV(inputDataG[i]);
    }  
  }

  /* Encryption */
  else
  {
    AES->CTRL = AES_CTRL_KEYBUFEN |     /* Use key buffering */
                AES_CTRL_XORSTART;      /* Start encryption on data write */

    /* Writing to the DATA register here does NOT trigger encryption */     
    for (i = 3; i >= 0; i--)
    {
      AES->DATA = __REV(iv32[i]);
    }   

    /* Load data and trigger encryption using XORDATA*/    
    for (i = 3; i >= 0; i--)
    {
      AES->XORDATA = __REV(inputDataG[i]);
    }   
  }
}

/**************************************************************************//**
 * @brief  Function to check if AES process has finished
 * @return true if AES operation has finished
 *****************************************************************************/
bool AesFinished(void)
{
  return aesFinished;
}



