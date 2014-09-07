/**************************************************************************//**
 * @file fortuna_entropy_accumulator.h
 * @brief Interface to entropy accumulator of the Fortuna PRNG.
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


#ifndef __FORTUNA_ENTROPY_ACCUMULATOR_H
#define __FORTUNA_ENTROPY_ACCUMULATOR_H

/* FUNCTION Prototypes */

/* Initialize the entropy accumulator. */
void EA_Init (void);

/* Add random data to the entropy pools.*/
int  EA_AddRandomEvent (int       sourceId,
                        uint32_t  eventData);

/* Check if an entropy pools are ready with sufficient entropy data. */
bool EA_PoolReady      (int       poolId);

/* Returns a 32 bit data word from an entropy pool. */
uint32_t EA_PoolDataWordGet (int      poolId,
                             int      poolWord);

/* Produce a 32-bit hash from a data buffer of any size. */
uint32_t EA_JenkinsHash (uint8_t *buffer,
                         uint32_t nBytes);

#endif /*__FORTUNA_ENTACCUMULATOR_H*/
