/**************************************************************************//**
 * @file aes.h
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
#ifndef _AES_H_
#define _AES_H_

#include "ramfunc.h"

bool decryptImage(void);

RAMFUNC void encryptCBC128(const uint8_t *key, uint8_t *inputData, const uint8_t *prevCipher, uint8_t *outputData);

void decryptCBC(const uint8_t *key, uint8_t* inputData, const uint8_t *prevCipher, uint8_t *outputData);

RAMFUNC void decryptBlockCBC256(uint8_t *input, uint8_t *output);
RAMFUNC void startDecryptCBC256(void);
RAMFUNC void endDecryptCBC256(void);

#endif
