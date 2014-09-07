/**************************************************************************//**
 * @file fortuna_seed_manager.h
 * @brief Interface to seed manager of Fortuna PRNG.
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

#ifndef __FORTUNA_SEED_MANAGER_H
#define __FORTUNA_SEED_MANAGER_H

#include "fortuna_prng.h"

/*
 * FUNCTION PROTOTYPES
 */

/* Retrieve the content of the seed file from the Flash memory. */
int SM_SeedFileRead(FORTUNA_Seed_t seed);

/* Save the seed to the seed file in Flash memory. */
int SM_SeedFileSave(FORTUNA_Seed_t seed);

/* Update seed file with new random seed. */
int SM_SeedFileUpdate (void);


#endif /*__FORTUNA_SEED_MANAGER_H*/
