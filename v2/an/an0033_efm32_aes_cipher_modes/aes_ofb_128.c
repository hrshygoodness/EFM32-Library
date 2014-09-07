/***************************************************************************//**
 * @file aes_ofb_128.c
 * @brief AES OFB 128-bit interrupt driven functions for EFM32
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
#include "aes_ofb_128.h"

static uint16_t         numberOfBlocks;
static uint16_t         blockIndex;
static uint32_t *       outputDataG;
static const uint32_t * inputDataG;
static volatile bool    aesFinished;

/**************************************************************************//**
 * @brief  AES IRQ Handler
 *****************************************************************************/
void AES_IRQHandler(void)
{
  int i;
  
  /* Clear interrupt flag */
  AES->IFC = AES_IFC_DONE;

  if (blockIndex < numberOfBlocks)
  {
    /* Store en/decrypted data and increment to the next start input vector */  
    for (i = 3; i >= 0; i--)
    {
      outputDataG[i] = __REV(AES->DATA) ^ inputDataG[i];
    }
   
    outputDataG += 4;
    inputDataG  += 4;

    /* Start en/decryption */
    AES->CMD = AES_CMD_START;
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
 * @brief  Encrypt/decrypt with 128-bit key in OFB mode. Function returns after
 * initializing encryption/decryption. Use AesFinished() to check for 
 * completion. Output data only valid after completion. 
 *
 * @param[in] key
 *   This is the 128-bit encryption key regardless of encryption or decryption 
 *
 * @param[in] inputData
 *   Buffer holding data to encrypt/decrypt.
 *
 * @param[out] outputData
 *   Buffer to put output data, must be of size blockNumber*16 bytes. This can
 *   be the same location as inputData.
 *
 * @param[in] blockNumber
 *   Number of 128-bit blocks to decrypt/encrypt.
 *
 * @param[in] iv
 *   128-bit initialization vector to use
 *****************************************************************************/
void AesOfb128(const uint8_t* key,
               const uint8_t* inputData,
               uint8_t*       outputData,
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

  /* Reset block index */
  blockIndex = 0;

  /* Initialize finished flag */
  aesFinished = false;

  /* Clear and enable AES interrupt */
  AES->IFC = AES_IFC_DONE;
  AES->IEN = AES_IEN_DONE;
  NVIC_EnableIRQ(AES_IRQn);
  
  /* Configure AES module */
  AES->CTRL = AES_CTRL_KEYBUFEN;  /* Set key buffer */

  /* Write KEY and DATA to AES module - but this will NOT start the en/decryption. */
  for (i = 3; i >= 0; i--)
  {
    /* Write the KEY to AES module beginning with the most significant 32-bit double word */
    /* Writing only the KEYHA 4 times will transfer the key to all 4
     * high key registers, used here, in 128 bit key mode, as buffers */
    AES->KEYHA = __REV(key32[i]);

    /* Load data using DATA register*/
    AES->DATA = __REV(iv32[i]);
  } 

  AES->CMD  = AES_CMD_START;     /* Only here start en/decryption */
}

/**************************************************************************//**
 * @brief  Function to check if AES process has finished
 * @return true if AES operation has finished
 *****************************************************************************/
bool AesFinished(void)
{
  return aesFinished;
}
