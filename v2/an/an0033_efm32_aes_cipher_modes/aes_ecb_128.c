/***************************************************************************//**
 * @file aes_ecb_128.c
 * @brief AES ECB 128-bit interrupt driven functions for EFM32
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
#include "aes_ecb_128.h"

static uint16_t        numberOfBlocks;
static uint16_t        blockIndex;
static uint32_t*       outputDataG;
static const uint32_t* inputDataG;
static volatile bool   aesFinished;

/**************************************************************************//**
 * @brief  AES IRQ Handler
 *****************************************************************************/
void AES_IRQHandler(void)
{
  int i;
  
  /* Clear interrupt flag */
  AES->IFC = AES_IFC_DONE;

  /* Store en/decrypted data */
  for (i = 3; i >= 0; i--)
  {
    outputDataG[i] = __REV(AES->DATA);
  }
  
  outputDataG += 4;

  /* If not last block */
  if (blockIndex < numberOfBlocks - 1)
  {
    /* Load data and trigger encryption using DATA register, which is written 4 consecutive times */
    /* Writing the 128-bit block begins from the most significant 32-bit double word */
    inputDataG += 4;
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

/**************************************************************************//**
 * @brief  Encrypt/decrypt with 128-bit key in EBC mode. Function returns after
 * initializing encryption/decryption. Use AesFinished() to check for 
 * completion. Output data only valid after completion. 
 *
 * @param[in] key
 *   When doing encryption, this is the 128 bit encryption key. When doing
 *   decryption, this is the 128 bit decryption key. The decryption key may
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
 *****************************************************************************/
void AesEcb128(const uint8_t* key,
               const uint8_t* inputData,
               uint8_t*       outputData,
               bool           decrypt,
               const uint32_t blockNumber)
{
  int i;
  
  uint32_t* key32 = (uint32_t*) key;
  
  /* Copy to global 32-bit variable pointers */
  inputDataG     = (uint32_t*) inputData;
  outputDataG    = (uint32_t*) outputData;
  
  numberOfBlocks = blockNumber;

  /* Reset block index */
  blockIndex = 0;

  /* Initialize finished flag */
  aesFinished = false;

  /* Configure AES module */
  AES->CTRL = decrypt << _AES_CTRL_DECRYPT_SHIFT |  /* Set decryption */
              AES_CTRL_KEYBUFEN |                   /* Use key buffering */         
              AES_CTRL_DATASTART;                   /* Start encryption on data write */

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

  /* Load data and trigger encryption using DATA register, which is written 4 consecutive times */
  for (i = 3; i >= 0; i--)
  {
    AES->DATA = __REV(inputDataG[i]);
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
