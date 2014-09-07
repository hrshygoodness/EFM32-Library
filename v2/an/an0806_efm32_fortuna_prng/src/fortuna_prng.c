/**************************************************************************//**
 * @file fortuna_prng.c
 * @brief Core part of Fortuna PRNG.
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
#include <string.h>

/* AES definitions in EFM32 inclusion: */
#include "em_device.h"
#include "em_aes.h"
#include "em_emu.h"

/* Fortuna headers */
#include "fortuna.h"
#include "fortuna_prng.h"
#include "fortuna_entropy_accumulator.h"

/* Definitions */
#define AES_BLOCK_SIZE             (128)       /* AES block size in bits */
#define AES_BLOCK_SIZE_32BIT_WORDS (AES_BLOCK_SIZE / 32)
#define KEY_SIZE_32BIT_WORDS       (FORTUNA_KEY_SIZE / 32)


/* Static variables. */
/* The number of reseed that occur since the generator starts to produce
   an output.*/
static uint32_t  reseedCount = 0;
/* The secret random key used by the generator to produces PRN. */
static uint32_t  aesKey[KEY_SIZE_32BIT_WORDS];
/* Flag that indicates whether the AES key is initilized or not. */
static bool      aesKeyInitialized = false;
/* 128-bit input data to the generator operating AES in counter mode. */
static uint32_t  prngCounter[AES_BLOCK_SIZE_32BIT_WORDS] =
  { 0x00000000, 0x00000000, 0x00000000, 0x00000000 };
/* Flag that indicates whether AES has finished encryption operation. */
static volatile bool aesFinished;

/* Static functions. */
static void Aes256(uint32_t* key, uint32_t* inputData, uint32_t* outputData);
/* A Function to increment the value of the 128-bit counter. */
static void PrngCounterIncrement(void);
/* Generate a block of random data. */
static int  prnBlockGenerate(uint32_t* prnBlock);


/**************************************************************************//**
 * @brief
 *   Increment counter for next AES encryption in Counter mode.
 *
 * @details
 *   The 128 bit counter is array of 4 32-bit integers. so the first counter
 *   is at index 0 and the last counter is at index 3. if the first counter
 *   overflows then the next counter gets its turn and so on.
 *
 *****************************************************************************/
static void PrngCounterIncrement(void)
{
  int i;

  for (i=0; i<AES_BLOCK_SIZE_32BIT_WORDS; i++)
  {
    if (++prngCounter[i] != 0)
    {
      break;
    }
  }
}


/**************************************************************************//**
 * @brief
 *   Generate random key from pools.
 *
 * @return   0 on success.
 *           FORTUNA_NOT_ENOUGH_ENTROPY if there is not enough entropy.
 *****************************************************************************/
int PRNG_SeedGetFromPools(FORTUNA_Seed_t seed)
{
  int     i, j;

  if (!EA_PoolReady(0))
  {
    return FORTUNA_NOT_ENOUGH_ENTROPY;
  }

  /* Clear seed contents. */
  memset (seed, 0, sizeof(FORTUNA_Seed_t));

  /* Increment reseed counter. */
  reseedCount ++;

  /* Generate key from pools. */
  for (i = 0; i < NUM_POOLS; i++)
  {
    if ((0==i) || ((reseedCount % (1 << i))==0))
    {
      if (EA_PoolReady(i))
      {
        for (j = 0; j < FORTUNA_SEED_SIZE; j++)
        {
          seed[j] ^= EA_PoolDataWordGet(i, j); /* mix ready pools */
          
          /* Peform entropy diffusion by hashing the pool data.
             The Jenkins hash is selected here for efficiency.
             The user may select other hash algorithms if appropriate. */
          seed[j] = EA_JenkinsHash((uint8_t*)&seed[j], sizeof(uint32_t));
        }
      }
    }
    else
    {
      /* We can break out of the loop when we encounter the first pool that
         fails the 'reseedCount' selection. */
      break;
    }
  }
  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *   Seed the PRNG.
 *
 * @param    seed      Seed value to set in the PRNG.
 *
 * @return   N/A
 *****************************************************************************/
void PRNG_Reseed (FORTUNA_Seed_t seed)
{
  int j;

  /* Mix the seed with the existing key and hash the result.
     The Jenkins hash is selected here for efficiency.
     The user may select other hash algorithms if appropriate. */
  for (j = 0; j < FORTUNA_SEED_SIZE; j++)
  {
    aesKey[j] ^= seed[j];
    aesKey[j] = EA_JenkinsHash((uint8_t*)&aesKey[j], sizeof(uint32_t));
  }

  /* Mark that the AES key is initialized for the first time. */
  aesKeyInitialized = true;

  /* Advance the state of the counter by one. */
  PrngCounterIncrement();
}


/**************************************************************************//**
 * @brief
 *   Generate new PRNG AES key from AES output.
 *
 * @return   The new key generated from the generator output.
 *****************************************************************************/
int PRNG_KeySetFromOutput(void)
{
  int       retval;
  uint32_t  prnBlock[AES_BLOCK_SIZE_32BIT_WORDS];

  /* Generate a new block of random data for the lower 128 bits of the key. */
  if (FORTUNA_OK != (retval = prnBlockGenerate(prnBlock)))
  {
    return retval;
  }
  
  /* Copy the AES output into the lower 128 bits of the PRNG AES key. */
  memcpy(aesKey, prnBlock, sizeof(aesKey)/2);

  /* Generate a new block of random data for the upper 128 bits of the key. */
  if (FORTUNA_OK != (retval = prnBlockGenerate(prnBlock)))
  {
    return retval;
  }

  /* Copy the AES output into the upper 128 bits of the key. */
  memcpy((void*) ((uint32_t)aesKey+sizeof(aesKey)/2),
         prnBlock,
         sizeof(aesKey)/2);

  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief   AES interrupt Handler. Clears interrupt and then indicates data is
 *          ready by setting aesFinished to true.
 *****************************************************************************/
void AES_IRQHandler(void)
{
  /* Acknowledge interrupt. */
  AES->IFC = AES_IF_DONE;

  /* Indicate AES has finished. */
  aesFinished = true;
}


/**************************************************************************//**
 * @brief
 *   Encrypt data using AES-256
 *
 * @param   key         Pointer to key to use in the AES encryption
 * @param   inputData   Pointer to input data to encrypt
 * @param   outputData  Pointer to location where to store
 *                      the encrypted data (ciphertext)
 *****************************************************************************/
static void Aes256(uint32_t* key, uint32_t* inputData, uint32_t* outputData)
{
  int i;

  /* Configure AES module */
  AES->CTRL = AES_CTRL_AES256 |   /* 256-bit key mode*/
              AES_CTRL_DATASTART; /* Start encryption on data write */

  /* Clear flag which indicates whether the AES is finished. */
  aesFinished = false;

  /* Load key and input data. */
  for (i = 3; i >= 0; i--)
  {
    /* Load the 256-key into the low and high key registers */
    AES->KEYHA = __REV(key[i]);
    AES->KEYLA = __REV(key[i+4]);

    /* Load data and trigger encryption using DATA register which is written 4
       consecutive times. Writing the 128-bit block begins from the most
       significant 32-bit double word. */
    AES->DATA = __REV(*(inputData + i));
  }

  /* Wait in EM1 for the AES to finish. */
  while (false == aesFinished)
  {
    EMU_EnterEM1();
  }

  /* Store encrypted data */
  for (i = 3; i >= 0; i--)
  {
    *(outputData + i) = __REV(AES->DATA);
  }
}


/**************************************************************************//**
 * @brief
 *  Generate a block of psuedo random numbers.
 *
 * @details
 *  This function that produces a block of pseudo random numbers based on the
 *  global variables aesKey and prngCounter, and stores the output in the global
 *  variable called prnBlock.
 *
 * @return  0 on success.
 *          FORTUNA_NOT_ENOUGH_ENTROPY if AES key is not initialized.
 *****************************************************************************/
static int prnBlockGenerate(uint32_t*  prnBlock)
{
  if (!aesKeyInitialized)
    return FORTUNA_NOT_ENOUGH_ENTROPY;

  /* Encrypt counter with 256-bit key and generate random number. */
  Aes256(aesKey, prngCounter, prnBlock);

  /* Advance the state of the counter by one. */
  PrngCounterIncrement();

  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *  Generate the specified amount of pseudo random data.
 *
 *  @param   randomData   Pointer to location where to store the output of
 *                        random data.
 *  @param   numBytes     Number of bytes of random data to generate.
 *                        Maximum is FORTUNA_MAX_DATA_SIZE bytes.
 *
 *  @return   0 on success.
 *            FORTUNA_INVALID_DATA_SIZE if request size is higher than
 *                                      FORTUNA_MAX_DATA_SIZE.
 *            FORTUNA_NOT_ENOUGH_ENTROPY if there is not enough entropy.
 *****************************************************************************/
int PRNG_RandomDataGet (void* randomData, int numBytes)
{
  uint32_t  prnBlock[AES_BLOCK_SIZE_32BIT_WORDS];
  uint8_t*  pData = randomData;
  int       bytesToCopy;
  int       retval;

  if (numBytes > FORTUNA_MAX_DATA_SIZE)
  {
    return FORTUNA_INVALID_DATA_SIZE;
  }  
  if (!aesKeyInitialized)
  {
    return FORTUNA_NOT_ENOUGH_ENTROPY;
  }
  
  while (numBytes>0)
  {
    /* Generate a new block of pseudo random numbers. */
    if (FORTUNA_OK != (retval = prnBlockGenerate(prnBlock)))
      return retval;
    
    bytesToCopy = numBytes < AES_BLOCK_SIZE_32BIT_WORDS ?
      numBytes : AES_BLOCK_SIZE_32BIT_WORDS;

    /* Copy block into the buffer provided by the caller. */
    memcpy (pData, prnBlock, bytesToCopy);
    
    numBytes     -= bytesToCopy;
    pData        += bytesToCopy;
  }

  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *   A function that clears the contents of the key, output or counter.
 *
 * @param    generatorType   Generator type to clear.
 *****************************************************************************/
void PRNG_Clear(void)
{
  /* Clear the state data structures. */
  memset(prngCounter, 0, sizeof(prngCounter));
  memset(aesKey, 0, sizeof(aesKey));
}
