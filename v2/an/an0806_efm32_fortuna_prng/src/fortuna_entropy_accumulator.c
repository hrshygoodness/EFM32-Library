/**************************************************************************//**
 * @file fortuna_entropy_accumulator.c
 * @brief Implementation of the entropy accumulator of the Fortuna PRNG.
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

/* Fortuna headers */
#include "fortuna.h"
#include "fortuna_prng.h"
#include "fortuna_entropy_accumulator.h"

/* Structure that represents an entropy data pool. */
typedef struct
{
#define POOL_SIZE   (FORTUNA_KEY_SIZE/32)          /* Pool size 32 bit words */
  uint32_t data[POOL_SIZE]; /* 32xPOOL_SIZE bits of pool = key size */
  int      nextPoolWord[MAX_ENTROPY_SOURCES];
  bool     ready;           /* Indicates that the pool is updated with a new
                               entropy. */
} Pool_t;


/* Static variables */

/* The table of entopy pools used to harvest entropy from the sources. */
static Pool_t  entropyPoolTbl[NUM_POOLS];
static int     nextPool[MAX_ENTROPY_SOURCES];


/**************************************************************************//**
 * @brief
 *  Initialize the entropy accumulator resources (entropy pools).
 *****************************************************************************/
void EA_Init (void)
{
  int i, j;
  for (i=0; i<NUM_POOLS; i++)
  {
    for (j=0; j<POOL_SIZE; j++)
    {
      entropyPoolTbl[i].data[j] = 0;
    }
    for (j=0; j<MAX_ENTROPY_SOURCES; j++)
    {
      entropyPoolTbl[i].nextPoolWord[j] = 0;
    }
    entropyPoolTbl[i].ready     = 0;
  }

  for (i=0; i<MAX_ENTROPY_SOURCES; i++)
  {
    nextPool[i]     = 0;
  }
}


/**************************************************************************//**
 * @brief
 *  Add random data to the entropy pools.
 *
 * @details
 *  This function adds a 32 bit word to one of the entropy pools. The entropy
 *  pool which is updated is selected by the nextPool table element which
 *  corresponds to the specified <sourceId>. This is element is incremented in
 *  order to distribute the random data in a round robin fashion among the
 *  pools. The new data is mixed (XORed) with the existing entropy (i.e. the
 *  current pool word value), and the result is hashed back into the pool word.
 *
 * @param   sourceId      Entropy source identifier in the range
 *                        0..MAX_ENTROPY_SOURCES.
 * @param   eventData     32 bits of random data to add to the entropy pool.
 *
 * @return  FORTUNA_OK on success.
 *          FORTUNA_INVALID_SOURCE_ID if sourceId is invalid.
 *****************************************************************************/
int EA_AddRandomEvent(int       sourceId,
                      uint32_t  eventData)
{
  int       poolId        = nextPool[sourceId];
  Pool_t*   pPool         = &entropyPoolTbl[poolId];
  int       poolWordIdx   = pPool->nextPoolWord[sourceId];
  uint32_t  poolWordValue = pPool->data[poolWordIdx];

  if (sourceId >= MAX_ENTROPY_SOURCES)
    return FORTUNA_INVALID_SOURCE_ID;

  /* Mix (XOR) the event data with the existing entropy in the pools. */
  poolWordValue ^= eventData;

  /* Perform entropy diffusion (Jenkins Hash) */
  pPool->data[poolWordIdx] =
    EA_JenkinsHash((uint8_t*)&poolWordValue, sizeof(uint32_t));

  if (++(pPool->nextPoolWord[sourceId]) >= POOL_SIZE)
  {
    pPool->nextPoolWord[sourceId] = 0;
    /* Pool is full. Update pool status to 'ready'. */
    pPool->ready = true;
  }

  if (++nextPool[sourceId] >= NUM_POOLS)
  {
    nextPool[sourceId] = 0;
  }

  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *  Check if an entropy pools are ready with sufficient entropy data.
 *
 * @param   poolId    Identifier (0..NUM_POOLS) of the pool to check.
 *
 * @return  'true' if the entropy pool have enough entropy data.
 *          'false' if the entropy pool does not have enough entropy data.
 *****************************************************************************/
bool EA_PoolReady(int poolId)
{
  return entropyPoolTbl[poolId].ready;
}


/**************************************************************************//**
 * @brief
 *  Returns a 32 bit data word from an entropy pool.
 *
 * @param   poolId    Identifier (0..NUM_POOLS) of the pool to get data from.
 * @param   poolWord  Word number (0..POOL_SIZE) to return.
 *
 * @return  32 bit random data word
 *****************************************************************************/
uint32_t EA_PoolDataWordGet(int poolId,  int poolWord)
{
  return entropyPoolTbl[poolId].data[poolWord];
}


/**************************************************************************//**
 * @brief Jenkins hash function
 * Takes data buffer of any size and hashes it to a 32-bit data.
 *
 * @param       buffer    Pointer to data buffer to hash.
 * @param       nBytes    Size of data buffer in bytes.
 *
 * @return      The 32 bits hash value.
 *****************************************************************************/
uint32_t EA_JenkinsHash(uint8_t *buffer, uint32_t nBytes)
{
  uint32_t hash, i;

  /* Jenkins hash algorithm */
  for (hash = i = 0; i < nBytes; ++i)
  {
    hash += buffer[i];
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }
  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return hash;
}
