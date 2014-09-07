/**************************************************************************//**
 * @file fortuna.h
 * @brief Fortuna Pseudo Random Number Generator (PRNG) public API for EFM32.
 * @author Silicon Labs
 * @version 1.03
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __FORTUNA_H
#define __FORTUNA_H

/*
 *  Fortuna user configuration definitions:
 */

#define NUM_POOLS                         (32)  /* Number of pools used for
                                                   entropy acquisition.
                                                   Max is 32 pools. */
#define MAX_ENTROPY_SOURCES                (1)  /* Maximum number of entropy
                                                   sources allowed to feed
                                                   Fortuna with entropy via
                                                   the FORTUNA_AddRandomEvent
                                                   function. */
#define RESEED_PERIOD_TICKS              (100)  /* Minimum time between
                                                   reseed events in ticks. */
#define RESEED_LIMIT               (1024*1024)  /* Maximum number of pseudo
                                                   random data bytes between
                                                   reseed events. */
#define SEED_FILE_UPDATE_PERIOD_TICKS  (10000)  /* Minimum time between seed
                                                   file updates in ticks. */
#define SEED_FILE_FLASH_ADDRESS (FLASH_BASE+FLASH_SIZE-FLASH_PAGE_SIZE)
                                                /* Flash address where to
                                                   store the see file.
                                                   The user should make sure
                                                   the selected page is not
                                                   used for other purposes
                                                   in the application. */


/*
 *  Fortuna interface definitions:
 */

#define FORTUNA_MAX_DATA_SIZE   (1024*1024)     /* Maximum number of random data
                                                   bytes that can be requested
                                                   when calling the
                                                   FORTUNA_RandomDataGet
                                                   function. */
/*
 * Fortuna error codes:
 */
#define FORTUNA_OK                    (0)  /* Status code that indicates
                                              successfull operation. */
#define FORTUNA_NOT_ENOUGH_ENTROPY   (-1)  /* Fortuna is not ready to produce
                                              random data because it has not
                                              accumulated enough entropy. The
                                              user should make sure that the
                                              entropy sources add entropy via
                                              the FORTUNA_AddRandomEvent
                                              function. */
#define FORTUNA_INVALID_SOURCE_ID    (-2)  /* The entropy source id is invalid.
                                              i.e. outside the range
                                              0..MAX_ENTROPY_SOURCES */
#define FORTUNA_INVALID_DATA_SIZE    (-3)  /* The requested random data size is
                                              higher than FORTUNA_MAX_DATA_SIZE.
                                           */
#define FORTUNA_INVALID_SEED_FILE    (-4)  /* The seed file in flash memory is
                                              invalid. I.e. the checksum does
                                              not match the hash of the content.
                                           */
#define FORTUNA_FLASH_WRITE_ERROR    (-5)  /* Failed to write to flash memory.*/
#define FORTUNA_FLASH_ERASE_ERROR    (-6)  /* Failed to erase flash memory.*/

/*
 *  Fortuna interface functions:
 */

int    FORTUNA_Init (void);

int    FORTUNA_AddRandomEvent (int       sourceId,
                               uint32_t  eventData);

int    FORTUNA_RandomDataGet  (void* randomData,
                               int numBytes);

int    FORTUNA_Rand (uint32_t* pseudoRandomNumber);

void   FORTUNA_Shutdown (void);


/*
 *  Interface to system functions:
 */

/* Return a tick counter which is assumed to wrap around at the maximum value
   MAX_SYS_TICK_VAL. */
#define MAX_SYS_TICK_VAL  (0xFFFFFFFF)
uint32_t SysTickGet(void);


#endif /*__FORTUNA_H*/
