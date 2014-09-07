/**************************************************************************//**
 * @file fortuna.c
 * @brief Implementation of the EFM32 Fortuna interface.
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

/* CMSIS and EMLIB headers */
#include "em_device.h"
#include "em_cmu.h"

/* Fortuna headers */
#include "fortuna.h"
#include "fortuna_adc.h"
#include "fortuna_prng.h"
#include "fortuna_entropy_accumulator.h"
#include "fortuna_seed_manager.h"


/* The snapshot of the system tick at the time of the last reseed event.
   Used as paramater for decision in the next reseed event. */
static uint32_t lastReseedTicks = 0;
static uint32_t lastSeedFileUpdateTicks = 0;


/* Number of generated pseudo random number data bytes since last reseed event.
   Used as paramater for decision in the next reseed event. */
static uint32_t prngByteCount = 0;


/**************************************************************************//**
 * @brief
 *  Initialize the EFM32 Fortuna PRNG library.
 *
 * @details
 *  This function initializes the EFM32 Fortuna PRNG library by
 *  setting up all required devices and internal modules.
 *
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *****************************************************************************/
int FORTUNA_Init (void)
{
  FORTUNA_Seed_t  seed;
  int             retval;

  /* Initialize AES clock which will enable us to use the AES. */
  CMU_ClockEnable(cmuClock_AES, true);

  /* Setup AES to generate interrupt when encryption operations are done. */
  AES->IFC  = AES_IFC_DONE;
  AES->IEN |= AES_IFC_DONE;
  NVIC_EnableIRQ(AES_IRQn);

  /* Initialize entropy accumulator. */
  EA_Init();

  /* Read seed file in order to seed the PRNG initially. We will
     reseed the PRNG only if the the seed file is valid. If the seed
     file is invalid, we postpone the initial reseed until there is
     enough entropy in the entropy pools. FORTUNA_RandomDataGet will
     return error until there is enough entropy. The user should try
     to feed the PRNG with enough entropy. */
  if (FORTUNA_OK == SM_SeedFileRead(seed))
  {
    /* Feed the PRNG with the seed value. */
    PRNG_Reseed(seed);
    lastReseedTicks = SysTickGet();
    prngByteCount   = 0;

    /* Update the seed file with a new random seed from the PRNG. */
    if (FORTUNA_OK != (retval = SM_SeedFileUpdate()))
    {
      return retval;
    }
    else
    {
      lastSeedFileUpdateTicks = SysTickGet();
    }
  }
  
  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *  Add random data to the Fortuna entropy pools.
 *
 * @param   sourceId      Entropy source identifier in the range
 *                        0..MAX_ENTROPY_SOURCES.
 * @param   eventData     32 bits of random data to add to the entropy pool.
 * 
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *****************************************************************************/
int FORTUNA_AddRandomEvent (int  sourceId,  uint32_t  eventData)
                            
{
  return EA_AddRandomEvent (sourceId, eventData);
}


/**************************************************************************//**
 * @brief
 *  Checks whether the system tick counter has passed the specified period
 *  given by periodStart and periodTicks.
 *
 * @param    periodStart   The tick that marks the start of the period.
 * @param    periodTicks   The number of ticks that defines the period size.
 *
 * @return   'true' if system ticks has elapsed the period.
 *           'false' if not.
 *****************************************************************************/
static bool PeriodElapsed(uint32_t periodStart, uint32_t periodTicks)
{
  uint32_t currentTicks = SysTickGet();

  /* Check if the tick counter (32 bits) has wrapped around. */
  if (periodStart > currentTicks)
  {
    if ((currentTicks + (MAX_SYS_TICK_VAL-periodStart)) > periodTicks)
      return true;
  }
  else
  {
    if ((currentTicks - periodStart) > periodTicks)
      return true;
  }

  return false;
}


/**************************************************************************//**
 * @brief
 *  Checks and performs updates of the seed file or not.
 *
 * @details
 *  This function decides whether it is time for the periodic update of the
 *  seed file or not. The update period is specified by the
 *  'SEED_FILE_UPDATE_PERIOD_TICKS' definition.
 *
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *
 *****************************************************************************/
static int SeedFileUpdateCheck(void)
{
  int retval;

  if (PeriodElapsed (lastSeedFileUpdateTicks, SEED_FILE_UPDATE_PERIOD_TICKS))
  {
    /* Replace the old seed file with a fresh one. */
    if (FORTUNA_OK != (retval = SM_SeedFileUpdate()))
    {
      return retval;
    }
    else
    {
      lastSeedFileUpdateTicks = SysTickGet();
    }
  }
  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *  Decides whether to reseed the PRNG (from pools) or not.
 *
 * @details
 *  This function decides whether a reseed of the PRNG should be performed or
 *  not. Reseeds are performed when one of the following conditions are met:
 *  - the last reseed time is more than 'RESEED_INTERVAL_TICKS' ago.
 *  - the last reseed was performed more than 'RESEED_LIMIT' bytes of PRN data
 *    ago.
 *
 * @return      'true' if reseed conditions are met. 'false' if not.
 *****************************************************************************/
static bool ReseedNow(void)
{
  if (PeriodElapsed (lastReseedTicks, RESEED_PERIOD_TICKS))
  {
    return true;
  }

  /* Reseed periodically after the defined number of data generations. */
  if ((prngByteCount) >= RESEED_LIMIT)
  {
    return true;
  }

  return false;
}


/**************************************************************************//**
 * @brief
 *  Check and perform reseed of the PRNG.
 *
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *****************************************************************************/
static int ReseedCheck(void)
{
  FORTUNA_Seed_t seed;
  int retval;

  /* Reseed the generator either from pool? */
  if (ReseedNow())
  {
    /* Reseed key from the entropy pools. */
    if (FORTUNA_OK != (retval = PRNG_SeedGetFromPools(seed)))
      return retval;
    else
    {
      PRNG_Reseed(seed);
      lastReseedTicks = SysTickGet();
      prngByteCount   = 0;
    }
  }

  return FORTUNA_OK;
}


/**************************************************************************//**
 * @brief
 *  Generate a user specified amount of pseudo random data.
 *
 *  @param   randomData   Pointer to location where to store the output of
 *                        random data.
 *  @param   numBytes     Number of bytes of random data to generate.
 *                        Maximum is FORTUNA_MAX_DATA_SIZE bytes.
 *
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *****************************************************************************/
int FORTUNA_RandomDataGet (void* randomData, int numBytes)
{
  int retval;

  if (numBytes > FORTUNA_MAX_DATA_SIZE)
    return FORTUNA_NOT_ENOUGH_ENTROPY;

  /* Check and reseed if required. */
  if (FORTUNA_OK != (retval = ReseedCheck()))
    return retval;
  
  /* Retrieve the pseudo random data from the PRNG. */
  retval = PRNG_RandomDataGet(randomData, numBytes);

  if (FORTUNA_OK == retval)
  {
    /* Update the byte counter in order to avoid generating more than
       the limited amount of data. */
    prngByteCount += numBytes;

    /* Switch to a new PRNG key to avoid later compromises of this output. */
    retval = PRNG_KeySetFromOutput();

    if (FORTUNA_OK == retval)
    {
      /* Check and update seed file if required. */
      retval = SeedFileUpdateCheck();
    }
  }

  return retval;
}


/**************************************************************************//**
 * @brief
 *  Return a 32 bit pseudo random integer.
 *
 * @param   pseudoRandomNumber   Pointer to 32 bit location where the returned
 *                               pseudo random number should be stored.
 *
 * @return  FORTUNA_OK on success. Error code defined in fortuna.h if failure.
 *****************************************************************************/
int FORTUNA_Rand(uint32_t* pseudoRandomNumber)
{
  return FORTUNA_RandomDataGet(pseudoRandomNumber, sizeof(uint32_t));
}


/**************************************************************************//**
 * @brief
 *  Shutdown the Fortuna PRNG.
 *
 * @details
 *  This function shuts down the Fortuna PRNG by updating the seed file with
 *  the last cipher output from the AES, and clearing the PRNG internal state.
 *
 * @return        N/A
 *****************************************************************************/
void FORTUNA_Shutdown(void)
{
  /* Make sure that the seed file is up-to-date for next operation. */
  SM_SeedFileUpdate();

  /* Clear internal state variables of the PRNG. */
  PRNG_Clear();
}
