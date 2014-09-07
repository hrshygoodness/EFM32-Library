/**************************************************************************//**
 * @file fortuna_seed_manager.c
 * @brief Implementation of seed manager of Fortuna PRNG.
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
#include "em_msc.h"

/* Fortuna headers */
#include "fortuna.h"
#include "fortuna_prng.h"
#include "fortuna_seed_manager.h"
#include "fortuna_entropy_accumulator.h"


/**************************************************************************//**
 * @brief
 *  Read seed from the seed file in flash memory.
 *
 * @details
 *  This function reads a seed from the seed file in flash memory. The seed
 *  file is stored at the address given by SEED_FILE_FLASH_ADDRESS defined in
 *  fortuna.h.
 *
 * @param seed    Loacation where to return the seed which is read from file.
 *
 * @return        FORTUNA_OK on success.
 *                FORTUNA_INVALID_SEED_FILE if seed file is invalid.
 *****************************************************************************/
int SM_SeedFileRead(FORTUNA_Seed_t seed)
{
  /* Point to the location of the seed file, the seed file is saved
   * at address SEED_FILE_FLASH_ADDRESS. */
  uint32_t *address = (uint32_t *) SEED_FILE_FLASH_ADDRESS;
  int      i;

  for (i = 0; i < FORTUNA_SEED_SIZE; i++)
  {
    seed[i] = address[i];
  }

  /* Check whether the check sum (Jenkins hash) of the seed matches the
     stored value at location right after the seed in flash. */
  if (address[i] == EA_JenkinsHash((uint8_t*)seed, sizeof(FORTUNA_Seed_t)))
    return FORTUNA_OK;
  else
    return FORTUNA_INVALID_SEED_FILE;
}


/**************************************************************************//**
 * @brief
 *  Save seed file to flash memory.
 *
 * @details
 *  This function saves the seed file to flash memory for use of initialization
 *  at start up or restart after shutdown. The seed file is stored at the
 *  address given by SEED_FILE_FLASH_ADDRESS defined in fortuna.h.
 *
 * @param       seed   Seed to save to flash memory.
 *
 * @return      FORTUNA_OK on success. Error code if fail, see fortuna.h.
 *****************************************************************************/
int SM_SeedFileSave(FORTUNA_Seed_t seed)
{
  msc_Return_TypeDef status;
  uint32_t           checksum;

  /* Initialize the MSC for writing */
  MSC_Init();

  /* Erase the page */
  status = MSC_ErasePage((uint32_t *) SEED_FILE_FLASH_ADDRESS);

  /* If there is error when erasing, deactivate MSC*/
  if (status != mscReturnOk)
  {
    MSC_Deinit();
    return FORTUNA_FLASH_ERASE_ERROR; /* Failed to Erase */
  }

  /* Write seed to flash memory. */
  status =
    MSC_WriteWord((uint32_t *) SEED_FILE_FLASH_ADDRESS,
                  (void *) seed,
                  sizeof(FORTUNA_Seed_t));

  /* Check for errors during flash write. */
  if (status != mscReturnOk)
  {
    MSC_Deinit();
    return FORTUNA_FLASH_WRITE_ERROR;  /* Failed to Write */
  }

  checksum = EA_JenkinsHash((uint8_t*)seed, sizeof(FORTUNA_Seed_t));

  /* Write seed to flash memory. */
  status =
    MSC_WriteWord((uint32_t *) SEED_FILE_FLASH_ADDRESS + FORTUNA_SEED_SIZE,
                  (void *) &checksum,
                  sizeof(uint32_t));

  /* Check for errors during flash write. */
  if (status != mscReturnOk)
  {
    MSC_Deinit();
    return FORTUNA_FLASH_WRITE_ERROR;  /* Failed to Write */
  }

  /* Deactivate the MSC*/
  MSC_Deinit();

  return FORTUNA_OK; /* Success Writing */
}


/**************************************************************************//**
 * @brief
 *  Update seed file with new random seed.
 *
 * @details
 *  This function requests a new random seed from the PRNG and saves it to
 *  the seed file in flash memory.
 *
 * @return      0 on success. Error code if fail, see fortuna.h.
 *****************************************************************************/
int SM_SeedFileUpdate (void)
{
  FORTUNA_Seed_t seed;
  int            retval;

  /* Get a random seed from the PRNG. */
  retval = PRNG_RandomDataGet(seed, sizeof(FORTUNA_Seed_t));
  if (FORTUNA_OK != retval)
    return retval;
  
  return SM_SeedFileSave(seed);
}
