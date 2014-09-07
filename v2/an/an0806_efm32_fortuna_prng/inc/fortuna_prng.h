/**************************************************************************//**
 * @file fortuna_prng.h
 * @brief Interface to core part of Fortuna PRNG.
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

#ifndef __FORTUNA_PRNG_H
#define __FORTUNA_PRNG_H

#define FORTUNA_KEY_SIZE                   (256) /* 256 bit key */

#define FORTUNA_SEED_SIZE  (FORTUNA_KEY_SIZE/32) /* Seed size in 32 bit words */
                                                         
typedef uint32_t FORTUNA_Seed_t [FORTUNA_SEED_SIZE];


/* FUNCTION Prototypes */

/* Get a new seed value from the entropy pools. */
int  PRNG_SeedGetFromPools(FORTUNA_Seed_t seed);

/* Reseed the PRNG. */
void PRNG_Reseed (FORTUNA_Seed_t seed);

/* Set key from the output of the generator itself. */
int PRNG_KeySetFromOutput(void);

/* Generate the specified amount of pseudo random data. */
int PRNG_RandomDataGet (void* randomData, int numBytes);

/* A function that clears the contents of the key. */
void PRNG_Clear(void);

#endif /*__FORTUNA_PRNG_H*/
