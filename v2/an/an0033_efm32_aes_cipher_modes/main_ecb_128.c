/***************************************************************************//**
 * @file main_ecb_128.c
 * @brief AES ECB 128-bit example for EFM32
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

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_aes.h"
#include "aes_ecb_128.h"

const uint8_t exampleData[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 
                                0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
                                0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C, 
                                0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
                                0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11, 
                                0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF,
                                0xF6, 0x9F, 0x24, 0x45, 0xDF, 0x4F, 0x9B, 0x17, 
                                0xAD, 0x2B, 0x41, 0x7B, 0xE6, 0x6C, 0x37, 0x10};

const uint8_t expectedEncryptedData[] = { 0x3A, 0xD7, 0x7B, 0xB4, 0x0D, 0x7A, 0x36, 0x60, 
                                          0xA8, 0x9E, 0xCA, 0xF3, 0x24, 0x66, 0xEF, 0x97,
                                          0xF5, 0xD3, 0xD5, 0x85, 0x03, 0xB9, 0x69, 0x9D, 
                                          0xE7, 0x85, 0x89, 0x5A, 0x96, 0xFD, 0xBA, 0xAF,
                                          0x43, 0xB1, 0xCD, 0x7F, 0x59, 0x8E, 0xCE, 0x23, 
                                          0x88, 0x1B, 0x00, 0xE3, 0xED, 0x03, 0x06, 0x88,
                                          0x7B, 0x0C, 0x78, 0x5E, 0x27, 0xE8, 0xAD, 0x3F, 
                                          0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5D, 0xD4};

const uint8_t exampleKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 
                               0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

uint8_t       dataBuffer[sizeof(exampleData) / sizeof(exampleData[0])];
uint8_t       decryptionKey[16];

/**************************************************************************//**
 * @brief  Main function
 *         The example data is first encrypted and encrypted data is checked.
 *         The encrypted data is then decrypted and checked against original data.
 *         Program ends at last while loop if all is OK.
 *****************************************************************************/
int  main(void)
{
  uint32_t i;
  
  /* Initialize error indicator */
  bool error = false;

  /* Chip errata */
  CHIP_Init();

  /* Enable AES clock */
  CMU_ClockEnable(cmuClock_AES, true);

  /* Calculate decryption key from original key. Only needs to be done once for each key */
  AES_DecryptKey128(decryptionKey, exampleKey);
  
  /* Copy plaintext to dataBuffer */
  for (i=0; i<(sizeof(exampleData) / sizeof(exampleData[0])); i++)
  {
    dataBuffer[i] = exampleData[i];
  }

  /* Encrypt data in AES-128 ECB */
  AesEcb128(exampleKey,
            dataBuffer,
            dataBuffer,
            false,
            sizeof(dataBuffer)/(sizeof(dataBuffer[0])*16));

  /* Wait for AES to finish */
  while(!AesFinished());

  /* Check whether encrypted results are correct */
  for (i = 0; i < (sizeof(dataBuffer) / sizeof(dataBuffer[0])); i++)
  {
    if (dataBuffer[i] != expectedEncryptedData[i])
    {
      error = true;
    }
  }

  /* Decrypt data with AES-128 ECB */
  AesEcb128(decryptionKey,
            dataBuffer,
            dataBuffer,
            true,
            sizeof(dataBuffer) / (sizeof(dataBuffer[0])*16));

  /* Wait for AES to finish */
  while(!AesFinished());

  /* Check whether decrypted result is identical to the plaintext */
  for (i = 0; i < (sizeof(dataBuffer) / (sizeof(dataBuffer[0])*16)); i++)
  {
    if (dataBuffer[i] != exampleData[i])
    {
      error = true;
    }
  }

  /* Check for success */
  if (error)
  {
    while (1) ; /* Ends here if there has been an error */
  }
  else
  {
    while (1) ; /* Ends here if all OK */
  }
}
